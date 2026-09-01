// Appended to box2d_wasm.js via --post-js when building for WeedJS.
// Worker entry must be box2d_wasm.js so Emscripten pthread pool spawns this file.
// Pthread pool workers (name "em-pthread") must NOT run app glue.
//
// Weed loads weedjs_post.js then physics_host.impl.js from the same directory
// (engine src/box2d/). Host speaks Weed init/start and calls weedjsDoStep in-process.

(function () {
  if (self.name === "em-pthread") {
    console.log("[physics] pthread worker — skip app glue");
    return;
  }
  importScripts("weedjs_post.js");
  importScripts("physics_host.impl.js");
})();
