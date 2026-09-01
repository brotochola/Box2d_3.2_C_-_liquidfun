// render.worker.js
importScripts("game-constants.js", "world-bounds.js");

self.onerror = function (e) {
  console.error("[render.worker] onerror:", e.message, e.filename, e.lineno, e);
  postMessage({ type: "ERROR", worker: "render", message: e.message });
};

let canvas = null;
let ctx = null;
let slotHighWater = 0;
let bodyCapacity = 0;
let jointCount = 0;
let jointStride = 8;
let px = null;
let py = null;
let rotation = null;
let meta = null;
let jointMeta = null;
let rayOverlay = null;
let particlePos = null;
let particleFlags = null;
let particleCountView = null;
let particleCapacity = 0;
let particleRadius = 0.045;
let sabBuffer = null;

const scheduleFrame =
  typeof requestAnimationFrame === "function"
    ? requestAnimationFrame
    : (cb) => setTimeout(cb, 16);

const ARENA = [
  { x: 0, y: -1, hx: 14, hy: 1 },
  { x: -13, y: 12, hx: 1, hy: 14 },
  { x: 13, y: 12, hx: 1, hy: 14 },
];

const COLORS = {
  background: "#090d16",
  static: "#334155",
  dynamic: "#cbd5e1",
  water: "#38bdf8",
  viscous: "#c084fc",
  tensile: "#2dd4bf",
  elastic: "#4ade80",
  powder: "#facc15",
  spring: "#fb7185",
  jointDistance: "#e2e8f0",
  jointRevolute: "#94a3b8",
  jointPrismatic: "#64748b",
  jointWeldA: "#fb923c",
  jointWeldB: "#22d3ee",
};

let renderFrames = 0;
let renderFpsLast = performance.now();

function reportRenderFps() {
  renderFrames++;
  const now = performance.now();
  const elapsed = now - renderFpsLast;
  if (elapsed >= 500) {
    postMessage({
      type: "FPS",
      worker: "render",
      fps: Math.round((renderFrames * 1000) / elapsed),
    });
    renderFrames = 0;
    renderFpsLast = now;
  }
}

function drawStaticBox(cx, cy, hx, hy, width, height, fillStyle) {
  const bl = worldToScreen(cx - hx, cy - hy, width, height);
  const tr = worldToScreen(cx + hx, cy + hy, width, height);
  ctx.fillStyle = fillStyle;
  ctx.fillRect(bl.sx, tr.sy, tr.sx - bl.sx, bl.sy - tr.sy);
}

function readMetaSlot(slot) {
  const base = slot * 4;
  return {
    shapeType: meta[base],
    halfW: meta[base + 1],
    halfH: meta[base + 2],
    flags: meta[base + 3],
  };
}

function readJointSlot(slot) {
  const base = slot * jointStride;
  return {
    type: jointMeta[base + JOINT_LAYOUT.TYPE],
    flags: jointMeta[base + JOINT_LAYOUT.FLAGS],
    ax: jointMeta[base + JOINT_LAYOUT.AX],
    ay: jointMeta[base + JOINT_LAYOUT.AY],
    bx: jointMeta[base + JOINT_LAYOUT.BX],
    by: jointMeta[base + JOINT_LAYOUT.BY],
    rotC: jointMeta[base + JOINT_LAYOUT.ROT_C],
    rotS: jointMeta[base + JOINT_LAYOUT.ROT_S],
  };
}

function drawWorldLine(ax, ay, bx, by, width, height, strokeStyle, lineWidth = 1) {
  const a = worldToScreen(ax, ay, width, height);
  const b = worldToScreen(bx, by, width, height);
  ctx.strokeStyle = strokeStyle;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  ctx.moveTo(a.sx, a.sy);
  ctx.lineTo(b.sx, b.sy);
  ctx.stroke();
}

function drawWorldPoint(x, y, radius, width, height, fillStyle) {
  const { sx, sy } = worldToScreen(x, y, width, height);
  const { scaleX } = worldScale(width, height);
  ctx.fillStyle = fillStyle;
  ctx.beginPath();
  ctx.arc(sx, sy, Math.max(1, radius * scaleX), 0, Math.PI * 2);
  ctx.fill();
}

function drawWorldRect(cx, cy, hx, hy, angle, width, height, fillStyle) {
  const { sx, sy } = worldToScreen(cx, cy, width, height);
  const { scaleX, scaleY } = worldScale(width, height);
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(angle);
  ctx.fillStyle = fillStyle;
  ctx.fillRect(-hx * scaleX, -hy * scaleY, hx * 2 * scaleX, hy * 2 * scaleY);
  ctx.restore();
}

function drawDistanceJoint(joint, width, height) {
  drawWorldLine(joint.ax, joint.ay, joint.bx, joint.by, width, height, COLORS.jointDistance, 1.5);
  drawWorldPoint(joint.ax, joint.ay, 0.06, width, height, COLORS.jointDistance);
  drawWorldPoint(joint.bx, joint.by, 0.06, width, height, COLORS.jointDistance);
}

function drawRevoluteJoint(joint, width, height) {
  drawWorldPoint(joint.ax, joint.ay, 0.12, width, height, COLORS.jointRevolute);
  drawWorldLine(joint.ax, joint.ay, joint.bx, joint.by, width, height, COLORS.jointRevolute, 1);
}

function drawPrismaticJoint(joint, width, height) {
  const axisLen = 0.5;
  const axisX = joint.rotC;
  const axisY = joint.rotS;
  drawWorldLine(
    joint.ax - axisX * axisLen,
    joint.ay - axisY * axisLen,
    joint.ax + axisX * axisLen,
    joint.ay + axisY * axisLen,
    width,
    height,
    COLORS.jointPrismatic,
    2,
  );
  drawWorldLine(joint.ax, joint.ay, joint.bx, joint.by, width, height, COLORS.jointPrismatic, 1);
  drawWorldPoint(joint.ax, joint.ay, 0.05, width, height, COLORS.jointPrismatic);
  drawWorldPoint(joint.bx, joint.by, 0.05, width, height, "#38bdf8");
}

function drawWeldJoint(joint, width, height) {
  const angleA = Math.atan2(joint.rotS, joint.rotC);
  drawWorldRect(joint.ax, joint.ay, 0.12, 0.06, angleA, width, height, COLORS.jointWeldA);
  drawWorldRect(joint.bx, joint.by, 0.12, 0.06, angleA, width, height, COLORS.jointWeldB);
}

function drawJoint(slot, width, height) {
  const joint = readJointSlot(slot);
  if (joint.flags & JOINT_FLAG.DISABLED) {
    return;
  }

  switch (joint.type) {
    case JOINT_TYPE.DISTANCE:
      drawDistanceJoint(joint, width, height);
      break;
    case JOINT_TYPE.REVOLUTE:
      drawRevoluteJoint(joint, width, height);
      break;
    case JOINT_TYPE.PRISMATIC:
      drawPrismaticJoint(joint, width, height);
      break;
    case JOINT_TYPE.WELD:
      drawWeldJoint(joint, width, height);
      break;
    default:
      drawWorldLine(joint.ax, joint.ay, joint.bx, joint.by, width, height, COLORS.jointDistance);
      break;
  }
}

function drawDynamicBody(slot, width, height, scaleX, scaleY) {
  const { shapeType, halfW, halfH } = readMetaSlot(slot);
  const { sx, sy } = worldToScreen(px[slot], py[slot], width, height);
  const angle = rotation[slot];

  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(angle);
  ctx.fillStyle = COLORS.dynamic;

  if (shapeType === SHAPE_TYPE.CIRCLE) {
    ctx.beginPath();
    ctx.arc(0, 0, halfW * scaleX, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "rgba(0,0,0,0.4)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(halfW * scaleX, 0);
    ctx.stroke();
  } else {
    const screenHx = halfW * scaleX;
    const screenHy = halfH * scaleY;
    ctx.fillRect(-screenHx, -screenHy, screenHx * 2, screenHy * 2);
    ctx.strokeStyle = "rgba(0,0,0,0.4)";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-screenHx, -screenHy, screenHx * 2, screenHy * 2);
  }

  ctx.restore();
}

function frame() {
  if (!ctx || !px || !meta) {
    scheduleFrame(frame);
    return;
  }

  try {
    const width = canvas.width;
    const height = canvas.height;
    const { scaleX, scaleY } = worldScale(width, height);

    ctx.fillStyle = COLORS.background;
    ctx.fillRect(0, 0, width, height);

    // Draw static arena walls
    for (const wall of ARENA) {
      drawStaticBox(
        wall.x,
        wall.y,
        wall.hx,
        wall.hy,
        width,
        height,
        COLORS.static,
      );
    }

    // Draw joints
    if (jointMeta) {
      for (let slot = 0; slot < jointCount; slot++) {
        drawJoint(slot, width, height);
      }
    }

    // Draw rigid bodies
    for (let slot = 0; slot < slotHighWater; slot++) {
      const { flags } = readMetaSlot(slot);
      if (flags & META_FLAG.DISABLED) {
        continue;
      }
      if (flags & META_FLAG.STATIC) {
        continue;
      }
      drawDynamicBody(slot, width, height, scaleX, scaleY);
    }

    // Draw LiquidFun particles
    if (particlePos && particleCountView) {
      const rawCount = particleCountView[0];
      const pCount = Math.max(0, Math.min(rawCount, particleCapacity));
      const r = particleRadius || 0.045;
      const radiusPx = Math.max(1.5, r * scaleX);
      const twoPi = Math.PI * 2;

      const groups = {
        water: [],
        viscous: [],
        tensile: [],
        elastic: [],
        powder: [],
        spring: [],
      };

      for (let i = 0; i < pCount; i++) {
        const x = particlePos[i * 2];
        const y = particlePos[i * 2 + 1];
        const { sx, sy } = worldToScreen(x, y, width, height);

        let key = "water";
        if (particleFlags) {
          const flag = particleFlags[i];
          if (flag & PARTICLE_FLAG.SPRING) {
            key = "spring";
          } else if (flag & PARTICLE_FLAG.ELASTIC) {
            key = "elastic";
          } else if (flag & PARTICLE_FLAG.POWDER) {
            key = "powder";
          } else if (flag & PARTICLE_FLAG.VISCOUS) {
            key = "viscous";
          } else if (flag & PARTICLE_FLAG.TENSILE) {
            key = "tensile";
          }
        }
        groups[key].push(sx, sy);
      }

      for (const key in groups) {
        const coords = groups[key];
        if (coords.length === 0) continue;
        ctx.fillStyle = COLORS[key] || COLORS.water;
        ctx.beginPath();
        for (let k = 0; k < coords.length; k += 2) {
          const sx = coords[k];
          const sy = coords[k + 1];
          ctx.moveTo(sx + radiusPx, sy);
          ctx.arc(sx, sy, radiusPx, 0, twoPi);
        }
        ctx.fill();
      }
    }

    // Ray overlay
    if (rayOverlay) {
      drawWorldLine(
        rayOverlay.ox,
        rayOverlay.oy,
        rayOverlay.ex,
        rayOverlay.ey,
        width,
        height,
        "#facc15",
        2,
      );
      if (rayOverlay.hit) {
        drawWorldPoint(rayOverlay.px, rayOverlay.py, 0.1, width, height, "#f87171");
        const nx = rayOverlay.nx ?? 0;
        const ny = rayOverlay.ny ?? 0;
        drawWorldLine(
          rayOverlay.px,
          rayOverlay.py,
          rayOverlay.px + nx * 0.5,
          rayOverlay.py + ny * 0.5,
          width,
          height,
          "#f87171",
          2,
        );
      }
    }

    reportRenderFps();
  } catch (err) {
    console.error("[render.worker] frame error:", err);
  }

  scheduleFrame(frame);
}

self.onmessage = (event) => {
  const data = event.data;
  if (!data) return;

  try {
    if (data.type === "SET_SLOT_HIGH_WATER") {
      slotHighWater = data.count;
      return;
    }

    if (data.type === "SLOT_HIGH_WATER") {
      slotHighWater = Math.max(slotHighWater, data.slot + 1);
      return;
    }

    if (data.type === "RAY_OVERLAY") {
      if (!data.visible) {
        rayOverlay = null;
      } else {
        rayOverlay = {
          ox: data.ox,
          oy: data.oy,
          ex: data.ex,
          ey: data.ey,
          hit: data.hit,
          px: data.px,
          py: data.py,
          nx: data.nx,
          ny: data.ny,
        };
      }
      return;
    }

    if (data.type === "SCENE_UPDATED") {
      slotHighWater = data.bodyCount ?? slotHighWater;
      jointCount = data.jointCount ?? jointCount;
      if (data.particlePosByteOffset && sabBuffer) {
        particlePos = new Float32Array(sabBuffer, data.particlePosByteOffset, particleCapacity * 2);
        particleCountView = new Int32Array(sabBuffer, data.particleCountByteOffset, 1);
        if (data.particleFlagsByteOffset) {
          particleFlags = new Uint32Array(sabBuffer, data.particleFlagsByteOffset, particleCapacity);
        }
      }
      return;
    }

    if (data.type !== "INIT") {
      return;
    }

    canvas = data.canvas;
    slotHighWater = data.bodyCount;
    bodyCapacity = data.bodyCapacity;
    jointCount = data.jointCount ?? 0;
    const sab = data.sab;
    sabBuffer = sab;
    const offsets = data.channelOffsets;
    const metaStride = data.metaStride;
    const metaBaseIndex = data.metaBaseIndex;
    jointStride = data.jointStride ?? 8;
    const jointBaseIndex = data.jointBaseIndex ?? 0;

    px = new Float32Array(sab, offsets[STATE_CHANNELS.X] * 4, bodyCapacity);
    py = new Float32Array(sab, offsets[STATE_CHANNELS.Y] * 4, bodyCapacity);
    rotation = new Float32Array(sab, offsets[STATE_CHANNELS.ROTATION] * 4, bodyCapacity);
    meta = new Float32Array(sab, metaBaseIndex * 4, bodyCapacity * metaStride);
    if (jointCount > 0) {
      jointMeta = new Float32Array(sab, jointBaseIndex * 4, jointCount * jointStride);
    }

    particleCapacity = data.particleCapacity ?? 8192;
    particleRadius = data.particleRadius || 0.045;
    if (data.particlePosByteOffset) {
      particlePos = new Float32Array(sab, data.particlePosByteOffset, particleCapacity * 2);
      particleCountView = new Int32Array(sab, data.particleCountByteOffset, 1);
      if (data.particleFlagsByteOffset) {
        particleFlags = new Uint32Array(sab, data.particleFlagsByteOffset, particleCapacity);
      }
    }

    ctx = canvas.getContext("2d");
    scheduleFrame(frame);
  } catch (err) {
    console.error("[render.worker] onmessage error:", err);
    postMessage({ type: "ERROR", worker: "render", message: err?.message ?? String(err) });
  }
};
