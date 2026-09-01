// Appended to box2d_wasm.js via --post-js. Worker entry must be box2d_wasm.js
// so Emscripten pthread pool spawns box2d_wasm.js (not physics.worker.js).
// Pthread pool workers (name "em-pthread") must NOT run this app glue.

(function () {
  if (self.name === "em-pthread") {
    console.log("[physics] pthread worker — skip app glue");
    return;
  }

  console.log("[physics] post-js start, worker url:", self.location.href);

  importScripts("game-constants.js", "physics-api.js");
  console.log("[physics] game-constants + physics-api loaded");

  Module.monitorRunDependencies = function (left) {
    console.log("[physics] wasm runDependencies left:", left);
  };
  Module.onAbort = function (what) {
    console.error("[physics] wasm ABORT:", what);
  };

  self.onerror = function (e) {
    console.error("[physics] worker onerror:", e.message, e.filename, e.lineno);
  };

  const noLimitFPS = false;
  const BODY_COUNT = 20000;
  const SUBSTEPS = 2;
  const MAX_DT = 1 / 20;
  const ARENA_SLOT_COUNT = 3;
  const MAX_PARTICLES = 8192;
  const PARTICLE_RADIUS = 0.045;

  const UI_BOX_HX = 0.25;
  const UI_BOX_HY = 0.25;
  const UI_CIRCLE_R = 0.3;

  const MATERIAL = {
    density: 1.0,
    friction: 0.3,
    restitution: 0.1,
    linearDamping: 0.0,
    angularDamping: 0.0,
    gravityScale: 1.0,
  };

  let world = null;
  const pendingMessages = [];

  function flushPendingMessages() {
    while (pendingMessages.length > 0) {
      handlePhysicsMessage(pendingMessages.shift());
    }
  }

  function spawnArena(w) {
    w.createBox({
      type: BODY_TYPE.STATIC,
      x: 0,
      y: -1,
      hx: 21.333,
      hy: 1,
      ...MATERIAL,
    });
    w.createBox({
      type: BODY_TYPE.STATIC,
      x: -20.5,
      y: 11,
      hx: 1,
      hy: 12,
      ...MATERIAL,
    });
    w.createBox({
      type: BODY_TYPE.STATIC,
      x: 20.5,
      y: 11,
      hx: 1,
      hy: 12,
      ...MATERIAL,
    });
  }

  function clearDynamicBodies(w) {
    const jointCount = w.getJointCount();
    for (let h = 0; h < jointCount; h++) {
      w.destroyJoint(h);
    }
    const slotCount = w.getSlotCount();
    for (let slot = ARENA_SLOT_COUNT; slot < slotCount; slot++) {
      w.destroyBody(slot);
    }
  }

  function resetParticleSystem(w) {
    try {
      w.destroyParticleSystem();
    } catch (e) {}
    w.createParticleSystem({
      radius: PARTICLE_RADIUS,
      density: 1.0,
      maxParticles: MAX_PARTICLES,
      subSteps: 2,
    });
  }

  function clearScene(w) {
    clearDynamicBodies(w);
    resetParticleSystem(w);
  }

  function loadPreset(w, preset) {
    clearScene(w);

    switch (preset) {
      case "sandbox": {
        // Multi-material sandbox: Ramp + Water + Viscous + Powder + Elastic Jelly
        w.createBox({
          type: BODY_TYPE.STATIC,
          x: -6,
          y: 3,
          hx: 3.5,
          hy: 0.2,
          angle: -0.3,
          ...MATERIAL,
        });
        w.createBox({
          type: BODY_TYPE.STATIC,
          x: 6,
          y: 3,
          hx: 3.5,
          hy: 0.2,
          angle: 0.3,
          ...MATERIAL,
        });

        // Water
        w.createParticleBox({
          x0: -10,
          y0: 4,
          x1: -6.5,
          y1: 9,
          spacing: 0.09,
          flags: PARTICLE_FLAG.WATER,
        });

        // Viscous Slime
        w.createParticleBox({
          x0: -4,
          y0: 4,
          x1: -1.5,
          y1: 8,
          spacing: 0.09,
          flags: PARTICLE_FLAG.VISCOUS,
        });

        // Granular Powder / Sand
        w.createParticleBox({
          x0: 1.5,
          y0: 4,
          x1: 4,
          y1: 8,
          spacing: 0.09,
          flags: PARTICLE_FLAG.POWDER,
        });

        // Elastic Jelly Group (Shape-matching + Springs)
        w.createParticleGroupCircle({
          x: 8.0,
          y: 7.0,
          radius: 1.0,
          spacing: 0.09,
          flags: PARTICLE_FLAG.ELASTIC | PARTICLE_FLAG.SPRING,
          strength: 0.6,
        });

        // Dynamic floating boxes
        for (let i = 0; i < 3; i++) {
          w.createBox({
            type: BODY_TYPE.DYNAMIC,
            x: -8 + i * 1.5,
            y: 11 + i * 0.5,
            hx: 0.4,
            hy: 0.4,
            density: 0.4,
            friction: 0.3,
            restitution: 0.1,
          });
        }
        break;
      }

      case "dambreak": {
        // High water column + stacked towers of rigid boxes
        w.createParticleBox({
          x0: -11.5,
          y0: 0.2,
          x1: -4.0,
          y1: 15.0,
          spacing: 0.09,
          flags: PARTICLE_FLAG.WATER,
        });

        // Tower 1
        for (let y = 0; y < 8; y++) {
          w.createBox({
            type: BODY_TYPE.DYNAMIC,
            x: 2.0,
            y: 0.5 + y * 0.9,
            hx: 0.4,
            hy: 0.4,
            density: 0.6,
            friction: 0.4,
          });
        }

        // Tower 2
        for (let y = 0; y < 6; y++) {
          w.createBox({
            type: BODY_TYPE.DYNAMIC,
            x: 6.0,
            y: 0.5 + y * 0.9,
            hx: 0.4,
            hy: 0.4,
            density: 0.6,
            friction: 0.4,
          });
        }
        break;
      }

      case "jelly_drop": {
        // Funnel ramps
        w.createBox({
          type: BODY_TYPE.STATIC,
          x: -6,
          y: 8,
          hx: 5.0,
          hy: 0.2,
          angle: -0.4,
          ...MATERIAL,
        });
        w.createBox({
          type: BODY_TYPE.STATIC,
          x: 6,
          y: 5,
          hx: 5.0,
          hy: 0.2,
          angle: 0.4,
          ...MATERIAL,
        });

        // Multiple elastic jelly shapes
        w.createParticleGroupCircle({
          x: -8.0,
          y: 13.0,
          radius: 1.2,
          spacing: 0.09,
          flags: PARTICLE_FLAG.ELASTIC | PARTICLE_FLAG.SPRING,
          strength: 0.7,
        });
        w.createParticleGroupCircle({
          x: -4.0,
          y: 15.0,
          radius: 0.9,
          spacing: 0.09,
          flags: PARTICLE_FLAG.ELASTIC | PARTICLE_FLAG.SPRING,
          strength: 0.5,
        });
        w.createParticleGroupBox({
          x0: -1.0,
          y0: 16.0,
          x1: 1.5,
          y1: 18.0,
          spacing: 0.09,
          flags: PARTICLE_FLAG.ELASTIC | PARTICLE_FLAG.SPRING,
          strength: 0.8,
        });
        break;
      }

      case "sand_funnel": {
        // Funnel
        w.createBox({
          type: BODY_TYPE.STATIC,
          x: -5.5,
          y: 6,
          hx: 5.0,
          hy: 0.2,
          angle: -0.6,
          ...MATERIAL,
        });
        w.createBox({
          type: BODY_TYPE.STATIC,
          x: 5.5,
          y: 6,
          hx: 5.0,
          hy: 0.2,
          angle: 0.6,
          ...MATERIAL,
        });

        // Granular Sand block in funnel
        w.createParticleBox({
          x0: -4.0,
          y0: 8.0,
          x1: 4.0,
          y1: 15.0,
          spacing: 0.09,
          flags: PARTICLE_FLAG.POWDER,
        });
        break;
      }

      case "wave_tank": {
        // Wide water pool
        w.createParticleBox({
          x0: -11.5,
          y0: 0.2,
          x1: 11.5,
          y1: 5.0,
          spacing: 0.09,
          flags: PARTICLE_FLAG.WATER,
        });

        // Floating dynamic crates
        for (let i = -8; i <= 8; i += 3) {
          w.createBox({
            type: BODY_TYPE.DYNAMIC,
            x: i,
            y: 6.0,
            hx: 0.5,
            hy: 0.5,
            density: 0.35,
            friction: 0.3,
          });
        }
        break;
      }
    }

    const ready = w.getReadyPayload();
    postMessage({
      type: "SCENE_UPDATED",
      ...ready,
    });
  }

  function handlePhysicsMessage(data) {
    if (!data || !data.type) {
      return;
    }

    if (!world) {
      console.log("[physics] queue message (world not ready):", data.type);
      pendingMessages.push(data);
      return;
    }

    try {
      switch (data.type) {
        case "CREATE_BOX": {
          const handle = world.createBox({
            type: BODY_TYPE.DYNAMIC,
            x: data.x,
            y: data.y,
            hx: data.hx ?? UI_BOX_HX,
            hy: data.hy ?? UI_BOX_HY,
            ...MATERIAL,
          });
          postMessage({
            type: "BODY_CREATED",
            slot: handle.slot,
            shapeType: SHAPE_TYPE.BOX,
          });
          break;
        }

        case "CREATE_CIRCLE": {
          const handle = world.createCircle({
            type: BODY_TYPE.DYNAMIC,
            x: data.x,
            y: data.y,
            radius: data.radius ?? UI_CIRCLE_R,
            ...MATERIAL,
          });
          postMessage({
            type: "BODY_CREATED",
            slot: handle.slot,
            shapeType: SHAPE_TYPE.CIRCLE,
          });
          break;
        }

        case "SPAWN_PARTICLES": {
          const { x, y, radius = 0.8, flags = PARTICLE_FLAG.WATER, spacing = 0.09 } = data;
          const count = world.createParticleBox({
            x0: x - radius,
            y0: y - radius,
            x1: x + radius,
            y1: y + radius,
            spacing: spacing,
            flags: flags >>> 0,
          });
          postMessage({
            type: "PARTICLES_SPAWNED",
            count: world.getParticleCount(),
            added: count,
          });
          break;
        }

        case "SPAWN_PARTICLE_GROUP": {
          const {
            x,
            y,
            radius = 0.9,
            shape = "circle",
            flags = PARTICLE_FLAG.ELASTIC | PARTICLE_FLAG.SPRING,
            strength = 0.6,
            spacing = 0.09,
          } = data;
          let id;
          if (shape === "box") {
            id = world.createParticleGroupBox({
              x0: x - radius,
              y0: y - radius,
              x1: x + radius,
              y1: y + radius,
              spacing: spacing,
              flags: flags >>> 0,
              strength: strength,
            });
          } else {
            id = world.createParticleGroupCircle({
              x: x,
              y: y,
              radius: radius,
              spacing: spacing,
              flags: flags >>> 0,
              strength: strength,
            });
          }
          postMessage({
            type: "PARTICLE_GROUP_SPAWNED",
            groupId: id,
            count: world.getParticleCount(),
          });
          break;
        }

        case "EMIT_STREAM": {
          const { x, y, vx = 0, vy = -2, flags = PARTICLE_FLAG.WATER, count = 4 } = data;
          for (let i = 0; i < count; i++) {
            const rx = (Math.random() - 0.5) * 0.2;
            const ry = (Math.random() - 0.5) * 0.2;
            world.createParticleBox({
              x0: x + rx - 0.04,
              y0: y + ry - 0.04,
              x1: x + rx + 0.04,
              y1: y + ry + 0.04,
              spacing: 0.08,
              flags: flags >>> 0,
            });
          }
          break;
        }

        case "LOAD_PRESET": {
          loadPreset(world, data.preset || "sandbox");
          break;
        }

        case "CLEAR_SCENE": {
          clearScene(world);
          const ready = world.getReadyPayload();
          postMessage({
            type: "SCENE_CLEARED",
            bodyCount: world.getSlotCount(),
            jointCount: world.getJointCount(),
            ...ready,
          });
          break;
        }

        case "CLEAR_PARTICLES": {
          resetParticleSystem(world);
          postMessage({
            type: "PARTICLES_CLEARED",
            count: 0,
          });
          break;
        }

        case "CAST_RAY": {
          const { ox, oy, dx, dy, requestId } = data;
          const hit = world.castRayClosest(ox, oy, dx, dy) === 1;
          const result = {
            type: "RAY_RESULT",
            requestId,
            hit,
            ox,
            oy,
            dx,
            dy,
          };
          if (hit) {
            const hits = world._queryHits;
            result.fraction = hits[1];
            result.slot = hits[0];
            result.px = hits[2];
            result.py = hits[3];
            result.nx = hits[4];
            result.ny = hits[5];
          }
          postMessage(result);
          break;
        }

        default:
          postMessage({
            type: "ERROR",
            message: `Unknown message type: ${data.type}`,
          });
      }
    } catch (err) {
      console.error("[physics] handlePhysicsMessage error:", err);
      postMessage({
        type: "ERROR",
        message: err?.message ?? String(err),
      });
    }
  }

  self.onmessage = function (event) {
    handlePhysicsMessage(event.data);
  };

  Module.onRuntimeInitialized = function () {
    console.log("[physics] onRuntimeInitialized");
    try {
      const { PhysicsWorld } = createPhysicsApi(Module);
      console.log("[physics] bindBuffers", BODY_COUNT);
      world = new PhysicsWorld(0.0, -9.8, {
        lengthUnitsPerMeter: 1,
        contactHertz: 30,
        contactDampingRatio: 10,
        contactSpeed: 3,
        maximumLinearSpeed: 400,
        box2dWorkerCount: 4,
      });
      world.bindBuffers(BODY_COUNT);

      spawnArena(world);
      console.log("[physics] arena spawned");

      resetParticleSystem(world);
      loadPreset(world, "sandbox");

      const ready = world.getReadyPayload();
      postMessage({
        type: "READY",
        ...ready,
      });
      console.log("[physics] READY posted, bodyCount:", ready.bodyCount, "particles:", ready.particleCount);

      flushPendingMessages();
    } catch (err) {
      console.error("[physics] init failed:", err);
      postMessage({
        type: "ERROR",
        message: err?.message ?? String(err),
      });
      return;
    }

    let physicsFrames = 0;
    let physicsFpsLast = performance.now();
    let lastStepTime = performance.now();

    function reportPhysicsFps() {
      physicsFrames++;
      const now = performance.now();
      const elapsed = now - physicsFpsLast;
      if (elapsed >= 500) {
        postMessage({
          type: "FPS",
          worker: "physics",
          fps: Math.round((physicsFrames * 1000) / elapsed),
          particleCount: world ? world.getParticleCount() : 0,
        });
        physicsFrames = 0;
        physicsFpsLast = now;
      }
    }

    function loop() {
      const now = performance.now();
      let dt = (now - lastStepTime) / 1000;
      lastStepTime = now;
      if (dt > MAX_DT) {
        dt = MAX_DT;
      }

      world.step(dt, SUBSTEPS);

      reportPhysicsFps();
      if (noLimitFPS) {
        setTimeout(loop, 2);
      } else {
        requestAnimationFrame(loop);
      }
    }
    requestAnimationFrame(loop);
  };

  console.log("[physics] post-js end, onRuntimeInitialized registered");
})();
