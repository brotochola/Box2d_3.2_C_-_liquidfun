// Linked with --js-library when building for WeedJS (this folder's CMakeLists.txt): emscripten library entries WeedJS replaces.
//
// emscripten_get_now: emscripten's own reads performance.timeOrigin at every call (a pthreads build needs one clock across
// workers). Box2D's profile timers call it hundreds of times a step, and in Chrome each read of performance.timeOrigin
// makes a 12-byte heap number: in WeedJS's Predator it was 576 KB/s of the physics worker's garbage and 7 % of its
// thread (WeedJS E-235). A realm's time origin never changes, so it is read once, when the module loads; the clock
// returns the same values.
addToLibrary({
  $weedTimeOrigin: 0,
  $weedTimeOrigin__postset: 'weedTimeOrigin = performance.timeOrigin;',
  emscripten_get_now__deps: ['$weedTimeOrigin'],
  emscripten_get_now: () => weedTimeOrigin + performance.now(),
});
