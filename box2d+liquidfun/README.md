# liquidfun-c

Un sistema de partículas fluidas al estilo **LiquidFun**, escrito desde cero
en **C17**, funcionando como módulo independiente sobre la API pública de
**Box2D 3.x** (el repo de Erin Catto, vendored en `../box2d`).

No es un fork de Box2D ni contiene código de LiquidFun. Es una
reimplementación propia del enfoque general (partículas aceleradas por grid,
weakly-compressible, acopladas en ambas direcciones con los rigid bodies),
retomando la misma idea arquitectónica que ya usaba LiquidFun en Box2D 2.x:
el sistema de partículas vive *al lado* del mundo, no adentro de su solver, y
se comunica con los rigid bodies únicamente a través de la API pública
(`b2World_OverlapAABB`, `b2Shape_GetClosestPoint`, `b2Body_ApplyLinearImpulse`,
etc).

Ver **ROADMAP.md** para el análisis completo de arquitectura.

## Build (native, against this repo's Box2D)

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo_water
./build/demo_elastic
```

WASM is built from the parent repo (`build_wasm.bat` / `build_for_weed.bat`);
`lf_particle_system.c` is compiled into `box2d_wasm`.

## Uso básico

```c
#include <box2d/box2d.h>
#include "liquidfun/lf_particle_system.h"

b2WorldId world = b2CreateWorld(&(b2WorldDef){ .gravity = {0, -10} });

lfParticleSystemDef psDef = lfDefaultParticleSystemDef();
psDef.radius = 0.05f;
lfParticleSystem* water = lfParticleSystem_Create(world, &psDef);

b2AABB block = { {-2, 2}, {2, 5} };
lfParticleSystem_CreateParticleBox(water, block, psDef.radius * 2, lf_waterParticle, (b2Vec2){0,0});

for (int i = 0; i < 600; i++) {
    b2World_Step(world, 1.0f/60.0f, 4);
    lfParticleSystem_Step(water, 1.0f/60.0f, 2);     // SIEMPRE después de b2World_Step
}
```

Elastic (gelatin) group:

```c
lfParticleGroupDef gdef = lfDefaultParticleGroupDef();
gdef.flags = lf_elasticParticle;
gdef.position = (b2Vec2){0, 3};
gdef.radius = 0.6f;
gdef.strength = 0.4f;
lfParticleSystem_CreateParticleGroupCircle(water, &gdef);
```

Set `growable = false` (and a hard `maxParticles`) when binding position
buffers as WASM SharedArrayBuffer views — realloc would detach those views.

## Qué incluye

- Partículas de agua (`lf_waterParticle`) con presión tipo WCSPH simplificada.
- Flags `lf_viscousParticle`, `lf_tensileParticle`, `lf_wallParticle`,
  `lf_elasticParticle`.
- Grupos de partículas con shape matching 2D (centro de masa + ángulo).
- Colisión y acoplamiento bidireccional con shapes de Box2D vía API pública.
- Grid espacial por hashing; contactos partícula-cuerpo invertidos
  (un OverlapAABB del cloud, luego grid vs shapes).
- Demos headless: `examples/demo_water.c`, `examples/demo_elastic.c`.

Lo que falta (ver ROADMAP.md): `spring`/`powder`/`barrier`, color mixing,
static pressure, lifetimes, colisión continua, paralelización.
