// Shared world viewport bounds for render + main-thread pointer mapping.
// 16:9 aspect ratio matching 960x540 canvas (scaleX === scaleY = 22.5 px/meter).
var WORLD_MIN_X = -21.333;
var WORLD_MAX_X = 21.333;
var WORLD_MIN_Y = -1.5;
var WORLD_MAX_Y = 22.5;

function worldToScreen(x, y, width, height) {
  const sx = ((x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X)) * width;
  const sy =
    height - ((y - WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y)) * height;
  return { sx, sy };
}

function screenToWorld(sx, sy, width, height) {
  const x =
    WORLD_MIN_X + (sx / width) * (WORLD_MAX_X - WORLD_MIN_X);
  const y =
    WORLD_MAX_Y - (sy / height) * (WORLD_MAX_Y - WORLD_MIN_Y);
  return { x, y };
}

function worldScale(width, height) {
  const scaleX = width / (WORLD_MAX_X - WORLD_MIN_X);
  const scaleY = height / (WORLD_MAX_Y - WORLD_MIN_Y);
  return { scaleX, scaleY, scale: Math.min(scaleX, scaleY) };
}
