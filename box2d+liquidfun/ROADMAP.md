# LiquidFun sobre Box2D 3 (C) — Análisis y Roadmap

Este documento resume lo que encontré al revisar los dos repos y por qué
`liquidfun-c` está construido como está.

## 1. Los dos repos, tal como están hoy

**`erincatto/box2d`** (clonado y compilado en este trabajo): la rama
`main` está en **v3.2.0** (`b2GetVersion()` en `src/core.c`), no "3.0" — pero
la arquitectura de fondo (ids opacos `b2BodyId`/`b2ShapeId`/`b2WorldId`,
buffers de datos en SoA, solver TGS-soft con sub-steps, broad-phase con
árbol dinámico + graph coloring para paralelizar) es la misma familia
introducida en 3.0. Todo lo de este documento aplica igual.

Puntos de arquitectura relevantes que confirmé leyendo el código fuente:

- Casi todo el motor vive en `src/` (no en `include/`) y son ~40 archivos
  `.c` bastante densos: `solver.c`, `contact_solver.c`, `constraint_graph.c`,
  `island.c`, `broad_phase.c`, etc. No hay ganchos públicos pensados para
  "insertar" un tipo de cuerpo nuevo en el solver.
- La API pública (`include/box2d/box2d.h`) sí expone todo lo necesario para
  construir un sistema externo que *consulte* el mundo: queries de overlap
  (`b2World_OverlapAABB`), distancia punto-shape (`b2Shape_GetClosestPoint`),
  masa/inercia/centro de masa del body (`b2Body_GetMass`,
  `b2Body_GetRotationalInertia`, `b2Body_GetWorldCenter`), y aplicar impulsos
  (`b2Body_ApplyLinearImpulse`).
- Dato útil: `b2Body_GetRotationalInertia()` en Box2D 3 devuelve la inercia
  **ya referida al centro de masa** (confirmé esto en `src/body.c`, donde el
  solver la usa directamente con brazos de palanca medidos desde
  `bodySim->center`). En Box2D 2.x el equivalente público devolvía la
  inercia respecto al *origen* del body, y LiquidFun tenía que restarle el
  término de eje paralelo a mano. Acá no hace falta: una simplificación real
  a favor nuestro.
- Por defecto (sin `BOX2D_DOUBLE_PRECISION`), `b2Pos` es literalmente
  `b2Vec2` (`include/box2d/math_functions.h`), así que todo el código de
  `liquidfun-c` puede tratar posiciones como `b2Vec2` sin fricción. El modo
  "large world" (coordenadas dobles) queda pendiente — ver §4.

**`google/liquidfun`**: revisé `Box2D/Box2D/Particle/b2ParticleSystem.cpp`
(136 KB, el archivo central) y `Dynamics/b2World.cpp`. El hallazgo clave,
que es la base de todo el diseño de `liquidfun-c`:

> LiquidFun **ya era un módulo relativamente desacoplado** dentro de Box2D
> 2.x. `b2World::Step()` llama primero a `particleSystem->Solve(step)` y
> recién después al solve rígido. Y `b2ParticleSystem::Solve` encuentra los
> contactos partícula-fixture con `m_world->QueryAABB(...)` +
> `fixture->ComputeDistance(...)`, exactamente el mismo patrón
> query-de-overlap + distancia-más-cercana que expone Box2D 3 en su API
> pública. El feedback hacia el rígido se hace con
> `body->ApplyLinearImpulse(f, point, true)` — otro método que sigue
> existiendo, con la misma firma conceptual, en Box2D 3.

Dicho de otra forma: portar LiquidFun "adentro" del solver de Box2D 3 no es
fiel al diseño original — LiquidFun *nunca* estuvo metido en el solver
rígido de Box2D 2.x. Construirlo como módulo externo que usa la API pública
es la continuación natural del diseño, no un atajo.

## 2. Decisión de arquitectura

| | Módulo externo (elegido) | Fork profundo del solver |
|---|---|---|
| Dónde vive el código | Librería propia, `liquidfun-c/`, sólo incluye `box2d/box2d.h` | Parcheando `src/solver.c`, `src/constraint_graph.c`, `src/contact.c` de Box2D |
| Mantenimiento al actualizar Box2D | Alto: sólo importa que la API pública no rompa | Bajo: hay que re-mergear contra cada release, el solver interno cambia seguido |
| Fidelidad al diseño de LiquidFun | Alta (ver §1) | Parcial — LiquidFun tampoco vivía en el solver interno de 2.x |
| Acoplamiento partícula↔rígido | Un paso por frame, vía impulsos públicos (igual que LiquidFun 2.x) | Podría integrarse en la misma iteración del solver (más preciso, más difícil) |
| Esfuerzo para tener algo andando | Días | Semanas/meses — `constraint_graph.c` + `solver.c` + `contact_solver.c` son ~150 KB de código muy denso, con graph coloring para paralelismo, que hay que entender a fondo primero |

Para C, sobre Box2D 3, con un alcance razonable: **módulo externo**. Es lo
que ya construí y dejé compilando y corriendo contra Box2D 3.2 real.

## 3. Qué construí (`liquidfun-c/`)

Pipeline de `lfParticleSystem_Step()`, llamado siempre *después* de
`b2World_Step()` para el mismo `dt`:

1. `SolveZombie` — order-preserving compact (Google 1.1 style). Groups are
   contiguous slabs `[firstIndex, lastIndex)`; no swap-with-last (that broke
   slabs). Also: `RotateBuffer`, Join/Split, solid depth+`SolveSolid`,
   `SolveRigid`, ApplyForce/Impulse, QueryAABB/RayCast.
2. Por cada sub-step:
   - `BuildGrid` — hash espacial (celda = diámetro de partícula), inserción
     por listas enlazadas. Con celda == radio de interacción, un vecindario
     3×3 siempre alcanza para encontrar todo par a distancia < diámetro
     (demostración corta en los comentarios del código).
   - `FindParticleContacts` — contactos partícula-partícula vía el grid.
   - `FindBodyContacts` — por cada partícula, `b2World_OverlapAABB` +
     `b2Shape_GetClosestPoint` contra las shapes de Box2D; calcula la masa
     efectiva del contacto (`invMassA + invMassB + invIB·(r×n)²`) igual que
     lo haría un solver de contacto normal.
   - `ComputeWeight` — densidad aproximada por partícula (suma de pesos de
     contacto, partícula-partícula + partícula-cuerpo).
   - `SolveGravity`, `SolvePressure` (repulsión tipo WCSPH cuando el peso
     acumulado excede 1, aplicada tanto entre partículas como contra
     cuerpos, con impulso devuelto al rígido vía
     `b2Body_ApplyLinearImpulse`), `SolveDamping` (anti-jitter normal),
     `SolveViscous`/`SolveTensile` (sólo para partículas con esos flags),
     `SolveLinearDamping` (estabilizador numérico estándar).
   - `LimitVelocity` — tope tipo CFL (máx. medio diámetro por sub-step).
     LiquidFun tiene el mismo tipo de paso en su propio pipeline
     (`LimitVelocity`, llamado en cada sub-step); no es un parche mío, es
     una práctica estándar en estos solvers explícitos.
   - `Integrate`, `SolveDepenetration` (red de seguridad posicional contra
     penetración residual).

Todo esto **lo compilé y corrí de verdad** contra un checkout real de
`erincatto/box2d` (no contra una API inventada de memoria): un mundo con
piso, paredes, una rampa estática y una caja dinámica liviana, más un bloque
de 952 partículas de agua, 360 steps (6s). Primera versión: exploto/perdió
partículas por un solver de presión sin tope de velocidad. Iterando sobre
eso (tope de velocidad tipo CFL + clamp del término de presión + un damping
lineal chico) el resultado final es estable: el agua cae, se derrama, fluye
sobre la rampa y se asienta en un charco, sin perder partículas ni explotar.
Los PNG renderizados a partir de los CSV de cada frame están en el mensaje
de esta conversación.

## 4. Roadmap — lo que sigue

**Fase 1 — hecho.** Partículas de agua sueltas, colisión y acoplamiento
bidireccional con rigid bodies, grid espacial, demo estable.

**Fase 2 — hecho.** Grupos de partículas (`lfParticleGroupId`), centro de masa
y velocidad angular por grupo (`UpdateGroupStatistics`). *Este roadmap decía
"sigue" pero el código ya lo tenía andando y probado (tests con `BARRIER` /
`STATIC_PRESSURE` ejercitan grupos) — quedó desactualizado, corregido acá.*

**Fase 3 — hecho.** `elastic` (`SolveElastic`), `spring` (`SolveSpring`,
`CapturePairs` al crear el grupo), `powder` (`SolvePowder`), `barrier`
(`SolveBarrier`), `staticPressure` (`SolveStaticPressure`, Poisson 8
iteraciones) — todos implementados y con test dedicado en el repo hermano
(`tests/node/liquidfun.wasm.test.js`). `colorMixing` y lifetimes por partícula
siguen **sin** implementar, a propósito: no hay caso de uso en Weed todavía.

**Fase 3.5 — optimización secuencial, hecho (2026-08-23).** Antes de
paralelizar (Fase 6) valía la pena exprimir el camino secuencial primero.
Campaña de 8 hipótesis benchmarkeadas una por una (L2, escena dedicada 5k→10k+
partículas, antes/después reales, no supuestos) — log completo en
`docs/LIQUIDFUN_HYPOTHESES.md` del repo hermano `multithreadad-game-engine`:

- **SIMD explícito** (SSE2/wasm128 vía `<emmintrin.h>` — mismo truco que usa
  `contact_solver.c` de Box2D para `B2_SIMD_SSE2` en `B2_CPU_WASM`, no algo
  inventado) en `Integrate`, `SolveGravity`, `LimitVelocity`. El build ya
  compilaba con `-msimd128 -msse2` (auto-vectorización del compilador
  solamente); ahora hay intrínsecos explícitos, y un `#error` en tiempo de
  compilación si esa flag llegara a faltar — un solo camino de código, sin
  fallback escalar duplicado (una sola tool-chain, un solo target).
- **Cacheo de celda de grid por partícula** (`cellX`/`cellY`, llenado una vez
  en `BuildGrid`) — evita recomputar `floorf`+multiplicación en cada lookup
  de `FindParticleContacts`, `ForEachParticleNearShape`, `SolveBarrier`.
- **Una sola query de broad-phase compartida** entre `FindBodyContacts` y
  `SolveCollision` — el AABB "swept" (`ComputeSweptCloudAABB`) es superset
  comprobado (mismo padding) del AABB estático que `FindBodyContacts` pedía
  por su cuenta. Un `b2World_OverlapAABB` menos por sub-step.
- **`CapturePairs` acelerado por grid** en vez de O(n²) todos-contra-todos:
  ventana **5×5** (no 3×3 — el radio de captura, 1.5×diámetro, excede una
  celda, así que la garantía de suficiencia del 3×3 de `FindParticleContacts`
  no aplica acá). Medido con un microbenchmark nuevo (el paso de `CapturePairs`
  es en creación de grupo, invisible al benchmark L2 de steady-state):
  **10.6ms → 2.6ms para un grupo de ~4000 partículas (~4×)**.
- **Compactación de la sublista `STATIC_PRESSURE`**: el loop de 8 iteraciones
  Poisson filtraba la lista completa de contactos cada vez; ahora se compacta
  una vez y se itera esa lista compactada.
- **Deinterleave de posiciones movido a C**: `wasm_wrapper.c` ahora expone
  `get_particle_x_byte_offset()` / `get_particle_y_byte_offset()` (arrays ya
  separados, llenados con un loop C ajustado dentro de `step_world`), en vez
  de que el lado JS haga el deinterleave leyendo floats intercalados de a uno.
- **`strictContactCheck` configurable**: estaba hardcodeado `true` en
  `wasm_wrapper.c`; el default de la librería (y de Google) es `false`. Ahora
  es un parámetro real de `create_particle_system`, expuesto hasta
  `physics.liquidFun.strictContactCheck` en la config de escena del repo
  hermano.
- **Insertion sort en vez de `qsort`** en `RemoveSpuriousBodyContacts` se
  probó y se **rechazó**: la lista que se ordena es cada contacto vivo antes
  del filtro, no los ≤3 que sobreviven — para una laguna asentada sobre un
  piso ancho eso es cientos+ de elementos, y el O(n²) del insertion sort
  perdió contra el O(n log n) de `qsort` (medido: ~0.6% peor, consistente en
  ambas corridas pareadas). Revertido al `qsort` original. Vale la pena
  dejarlo anotado — es exactamente el tipo de "optimización" que suena obvia
  y no lo es sin medir.

Resultado acumulado: en la escena de benchmark grande (~12k partículas),
`BOX2D_MS` bajó de forma medible en cada hipótesis aceptada, sin romper
ningún test de física (wall-climb, point-rest, barrier, staticPressure finito,
etc. siguen todos pasando).

**Fase 4 — colisión continua / anti-tunneling — hecho, con una salvedad
conocida.** *Esta fase decía "hay que implementarlo todavía" pero ya está: el
roadmap quedó desactualizado, corregido acá.* `SolveCollision` +
`RayCastParticleShape` ya hacen exactamente el raycasting predictivo que esta
fase pedía — `b2Shape_RayCast(shape, p, dt*velocity)` contra el desplazamiento
completo del sub-step, no `GetClosestPoint` después de integrar. Por qué la
ventana de búsqueda (`pad = diameter` en `ForEachParticleNearShape`) alcanza:
`LimitVelocity` (que corre antes en el mismo sub-step) ya garantiza
`dt·|velocity| <= diameter` para toda partícula, así que el padding usado acá
es exactamente ese mismo límite — no es casualidad, es la misma cota CFL
cerrando el círculo.

Salvedad real, no resuelta: `SolveBarrier` corre *después* de `LimitVelocity`
y puede subir la velocidad de un par `BARRIER` por encima de esa cota CFL (no
se re-clampea). Para esas partículas puntuales (sólo las que están en un par
`BARRIER` capturado), la ventana de búsqueda de `SolveCollision` podría no ser
suficiente. Acotado y de bajo impacto (flag opt-in, pocas partículas en la
práctica), pero real — no se intentó arreglar en esta ronda (que fue de
performance, no de estas garantías de corrección).

**Fase 5 — corrección de estabilidad en esquinas.**
En el demo, alguna partícula ocasionalmente gana energía cuando queda
atrapada en la esquina donde dos shapes estáticas se tocan (p. ej. piso +
pared), porque cada contacto se resuelve por separado en vez de como un
único sistema. Es un problema conocido de cualquier resolución de contactos
"por shape independiente" en vez de un solver LCP conjunto. Con el damping
lineal que agregué queda acotado, pero una solución de fondo implica
resolver los contactos de una partícula contra *todas* las shapes
simultáneamente (Gauss-Seidel de a un par por iteración, con varias
iteraciones, en vez de aplicar cada contacto y seguir).

**Fase 6 — performance / paralelización (rediseñado, 2026-08-23).**
*Esta fase era una idea vaga ("conviene un scheduler propio"); ahora hay un
plan concreto, gracias a un cambio de restricción del lado de Weed.*

Hoy `lfParticleSystem_Step` **no puede** correr en paralelo con `b2World_Step`
del mismo frame: el mundo está bloqueado durante el step, y `FindBodyContacts`
/ `SolveCollision` hacen queries en vivo (`b2World_OverlapAABB`,
`b2Shape_RayCast`, `TestPoint`) que no son válidas mientras Box2D muta su
propio estado interno. Eso es lo que hace inseguro "un hilo extra dedicado que
corra todo el step" tal cual — no es sólo una oportunidad de optimización
perdida, sería una carrera de datos real.

**Confirmado con Weed (el repo hermano) durante esta ronda: 1-2 frames de
atraso entre las partículas y los rigid bodies es aceptable.** Eso cambia el
diseño posible — abre una ventana productor-consumidor real:

- El hilo principal (el ÚNICO que toca el `b2WorldId` vivo) copia a un
  snapshot privado lo que LiquidFun necesita (transforms/shapes cercanas a la
  nube de partículas) una vez por step, después de que `b2World_Step`
  termina.
- Un hilo LiquidFun persistente consume ese snapshot y corre
  `lfParticleSystem_Step` sobre la copia — mismo patrón de pool persistente +
  cola de tareas + semáforo que ya usa `box2d/src/scheduler.c` internamente
  (`b2CreateThread` una sola vez por hilo, sin spawn/join por frame;
  confirmado leyendo `scheduler.c`: `b2SchedulerWorkerMain` bloquea en un
  semáforo y reclama tareas de un slot array con compare-exchange).
- Los impulsos resultantes se aplican al mundo real recién en un paso
  posterior, **desde el hilo principal** — nunca dos hilos tocando el
  `b2WorldId` a la vez. El costo es la latencia de 1-2 frames ya aceptada,
  no una carrera.

**Problema que sigue abierto, no resuelto acá:** `FindBodyContacts` /
`SolveCollision` dependen de queries en vivo de Box2D. Esas no se pueden
correr desde el hilo LiquidFun contra un snapshot — hace falta que el
snapshot incluya suficiente info de shapes (posición, tipo, dimensiones) para
que LiquidFun arme su **propia** estructura de broad-phase liviana sobre la
copia, en vez de las de Box2D. Es un rediseño real y acotado de
`FindBodyContacts`/`SolveCollision`, no un cambio trivial de threading.

**Sobre reusar el scheduler de Box2D directamente:** no se puede.
`b2CreateScheduler`/`b2Thread`/`b2Semaphore` están en `src/scheduler.h` +
`src/core.h`, no en `include/box2d/box2d.h` — son privados del motor.
`liquidfun-c`, que deliberadamente sólo incluye la API pública, necesitaría su
propio pool con las mismas primitivas crudas (`pthread_create` real bajo
Emscripten con `-msimd128 -msse2` — POSIX de verdad debajo, no una
simulación). Mismo patrón, instancia separada.

**Nota aparte, para cuando se ataque el resto del step (no el threading):**
las fases embarazosamente paralelas dentro de un solo `lfParticleSystem_Step`
(`BuildGrid`, `FindParticleContacts`, los `Solve*` que sólo escriben velocidad
propia por partícula) siguen siendo un lever real para intra-step
`parallel_for` — pero los pases por-contacto (`SolvePressure`, `SolveDamping`,
etc., que leen Y escriben en `contact.a`/`contact.b`) necesitan el mismo
truco de graph coloring que usa `constraint_graph.c` de Box2D para evitar
condiciones de carrera entre partículas que comparten un contacto. Eso es
trabajo real y acotado, no una consecuencia automática de mover el step a
otro hilo.

**Fase 7 — modo "large world" (`BOX2D_DOUBLE_PRECISION`).**
Todo el código de `liquidfun-c` asume `b2Pos == b2Vec2` (cierto en el build
por defecto). Si el proyecto necesita soportar el modo de coordenadas
dobles de Box2D 3, hay que revisar cada lugar donde se mezcla aritmética de
posición (`b2Vec2`) con `b2Pos`, y usar las variantes `*Pos` correspondientes
(`b2SubPos`, etc.) en vez de asumir el alias.

## 5. Licencias

Box2D 3 es MIT. LiquidFun es zlib (mismo estilo de licencia que Box2D 2.x,
permisiva). `liquidfun-c` no contiene código de ninguno de los dos — es
implementación propia contra la API pública de Box2D — pero si en algún
momento se decide mirar más de cerca partes específicas del código de
LiquidFun (por ejemplo para portar `elastic`/`spring` con más fidelidad en
la Fase 3), conviene mantener la licencia zlib de LiquidFun en cualquier
archivo que termine derivando de ese código.
