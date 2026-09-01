// Shared layout/constants for physics + render workers.
var BODY_TYPE = Object.freeze({
  STATIC: 0,
  DYNAMIC: 1,
  KINEMATIC: 2,
});

var SHAPE_TYPE = Object.freeze({
  BOX: 0,
  CIRCLE: 1,
  POLYGON: 2,
});

var META_FLAG = Object.freeze({
  STATIC: 1,
  DISABLED: 2,
});

var STATE_CHANNELS = Object.freeze({
  X: 0,
  Y: 1,
  ROTATION: 2,
  VX: 3,
  VY: 4,
  ANG_VEL: 5,
  ROT_C: 6,
  ROT_S: 7,
  /** @deprecated use X */
  PX: 0,
  /** @deprecated use Y */
  PY: 1,
});

var STATE_CHANNEL_COUNT = 8;

var JOINT_TYPE = Object.freeze({
  DISTANCE: 0,
  PRISMATIC: 3,
  REVOLUTE: 4,
  WELD: 5,
});

var JOINT_FLAG = Object.freeze({
  ACTIVE: 1,
  DISABLED: 2,
});

var JOINT_LAYOUT = Object.freeze({
  TYPE: 0,
  FLAGS: 1,
  AX: 2,
  AY: 3,
  BX: 4,
  BY: 5,
  ROT_C: 6,
  ROT_S: 7,
});

var EVENT_HEADER = Object.freeze({
  OVERLAP_COUNT: 0,
  RAY_HIT_COUNT: 1,
  CONTACT_BEGIN_COUNT: 2,
  CONTACT_END_COUNT: 3,
  CONTACT_HIT_COUNT: 4,
  SENSOR_BEGIN_COUNT: 5,
  SENSOR_END_COUNT: 6,
  MOVER_PLANE_COUNT: 7,
});

var DEFAULT_FILTER_MASK = 0;

var PARTICLE_FLAG = Object.freeze({
  WATER: 0,
  ZOMBIE: 1 << 0,
  WALL: 1 << 1,
  VISCOUS: 1 << 2,
  TENSILE: 1 << 3,
  ELASTIC: 1 << 4,
  POWDER: 1 << 5,
  SPRING: 1 << 6,
  BARRIER: 1 << 7,
  STATIC_PRESSURE: 1 << 8,
});

var PARTICLE_GROUP_FLAG = Object.freeze({
  SOLID: 1 << 0,
  RIGID: 1 << 1,
});
