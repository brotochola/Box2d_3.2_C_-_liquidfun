# Box2d_3.2_C_-_liquidfun

I forked Erin Catto's Box2D 3.2 (the C version), and ported LiquidFun 1.1 (C++ version) to C and integrated it to Box2D. Also the repo has everything ready to compile it to WASM with SharedArrayBuffer multi-threaded physics and zero-copy HTML5 Canvas rendering.

---

## Features

- **Box2D 3.2 Engine (C)**: Modern C17 Box2D physics solver by Erin Catto.
- **LiquidFun 1.1 Port to C**: Clean C reimplementation with:
  - Spatial hash grid neighborhood search ($O(N)$).
  - SPH density proxy weight computation & pressure relaxation solver.
  - Multi-material support: Water, Viscous fluids, Granular Powder/Sand, Elastic shape matching groups, Restoring Spring pairs, and Rigid particle groups.
  - Public API two-way rigid body coupling.
- **WebAssembly + SharedArrayBuffer (SAB)**:
  - Compiled with Emscripten using `-pthread`, `-msimd128`, and `-ffast-math`.
  - Zero-copy state export directly into WASM Heap SharedArrayBuffer.
- **Multi-Threaded Web Demo**:
  - Main thread: UI & FPS monitoring.
  - Physics worker (`box2d_wasm.js` + `physics_post.js`): steps simulation and particle dynamics.
  - Render worker (`render.worker.js`): reads SAB and draws particles/bodies onto `OffscreenCanvas` with batched paths.

---

## Quick Start

### 1. Run the Web Demo

SharedArrayBuffer requires cross-origin isolation (`COOP` and `COEP` headers). Run the included Node.js server:

```bash
node node_server.js
```

Open [http://localhost:8000/](http://localhost:8000/) in your browser.

*(If running under Apache / XAMPP, the included `.htaccess` file automatically sets the required headers).*

---

### 2. Building the WebAssembly Binary

To rebuild `box2d_wasm.js` and `box2d_wasm.wasm` from C sources:

**Windows**:
```bat
build_wasm.bat
```
*(Shortcut: `build.bat`)*

To clean and reconfigure:
```bat
build_wasm.bat clean
build_wasm.bat
```

**Linux / macOS**:
```bash
./build_wasm.sh
```

---

## Repository Structure

```
Box2d_3.2_C_-_liquidfun/
├── box2d/                    # Box2D 3.2 C source & public headers
│   ├── include/box2d/        # Public C API headers
│   └── src/                  # Core solver, wasm_wrapper.c, state_export.c
├── box2d+liquidfun/          # LiquidFun 1.1 C port
│   ├── include/liquidfun/    # lf_particle_system.h
│   ├── src/                  # lf_particle_system.c
│   └── examples/             # Native C demo source files
├── wasm/                     # WebAssembly CMake configuration
│   └── CMakeLists.txt
├── box2d_wasm.js             # Precompiled WASM JavaScript runtime
├── box2d_wasm.wasm           # Precompiled WebAssembly binary
├── index.html                # Web demo entry point & UI
├── physics_post.js           # Physics worker setup and demo scene
├── render.worker.js          # Canvas 2D render worker (reads SAB)
├── physics-api.js            # JavaScript API wrapper for Box2D & LiquidFun
├── game-constants.js         # Material flags and geometry definitions
├── world-bounds.js           # Screen to physics world coordinate conversion
├── node_server.js            # Local dev server with COOP/COEP headers
├── build_wasm.bat            # Automated WASM build script
├── LIQUIDFUN_SUMMARY.md      # Detailed technical summary & benchmark notes
└── README.md
```

---

## Documentation

For an in-depth breakdown of the particle solver mathematics, spatial hash grid, and benchmark-driven optimizations, refer to [LIQUIDFUN_SUMMARY.md](LIQUIDFUN_SUMMARY.md).
