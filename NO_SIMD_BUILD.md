# SIMD and no-SIMD builds

The WebAssembly module (Box2D 3.2 + liquidfun-c) builds two ways from the same sources:

| | SIMD (default) | no-SIMD |
|---|---|---|
| Selected by | nothing: it is the default | CMake `-DBOX2D_WASM_NO_SIMD=ON`, or `weedjs\build_for_weed.bat nosimd` |
| Instructions | wasm SIMD (`-msimd128 -msse2`) | not a single SIMD instruction |
| For | every engine with wasm SIMD. The faster build, the one WeedJS ships | engines without wasm SIMD, such as Firefox/GeckoView on 32-bit ARM |
| WeedJS build output | `weedjs/box2dWasm.js` + `.wasm`, copied into WeedJS's `src/box2d/` | `weedjs/nosimd/box2dWasm.js` + `.wasm`, never copied |

Both builds are part of the normal WeedJS build: `weedjs\build_for_weed.bat` with no arguments builds both.
It copies the SIMD module into WeedJS only after both modules have built and passed their checks.

## Rules for every change

Every change to the C code or to the build must keep both builds working. In practice:

1. **Build both before you commit.** Run `weedjs\build_for_weed.bat` with no arguments.
   `build_for_weed.bat simd` builds only the shipped module, for quick iterations. Never make it the last
   build before a commit.
2. **There is one switch.** Only `BOX2D_WASM_NO_SIMD` selects the no-SIMD build.
   - It defines `BOX2D_DISABLE_SIMD`, Box2D's own macro, in every target. C code tells the two builds
     apart with `BOX2D_DISABLE_SIMD`, never with `__wasm_simd128__` or a new macro.
   - A compile flag that only the SIMD build may have goes in `BOX2D_WASM_SIMD_FLAGS`.
   - Everything that differs between the two builds is in one block near the top of
     `wasm/CMakeLists.txt`.
3. **SIMD code always comes with its scalar twin, chosen at compile time.** There are three places:

   | Code | SIMD build | no-SIMD build | When you add SIMD code there |
   |---|---|---|---|
   | Box2D (`box2d/src`: `simd.h`, `contact_solver*.c`, `manifold.c`) | `B2_SIMD_SSE2`, selected by `core.h` | `B2_SIMD_NONE`: Box2D's own scalar code, kept by upstream | Follow upstream's pattern: the SIMD code under `B2_SIMD_SSE2` / `B2_SIMD_AVX2`, the scalar version in the `#else` (as in `b2FindMaxSeparation` in `manifold.c`). |
   | liquidfun-c (`box2d+liquidfun/src/lf_particle_system.c`) | SSE2 intrinsics from `<emmintrin.h>` | the same calls, from `lf_sse2_scalar.h` (plain C) | Write each intrinsic once. A new intrinsic needs a plain C version in `lf_sse2_scalar.h` that follows Emscripten's own emulation (`emsdk/upstream/emscripten/system/include/compat/*mmintrin.h`). It also needs a case in `box2d+liquidfun/tests/sse2_scalar/cases.h`. The no-SIMD build fails without both. |
   | The wrapper (`box2d/src/wasm_wrapper.c`) | the `LF_WASM_SSE2` blocks | the plain loop after each block | Every `LF_WASM_SSE2` block needs a plain loop that does the same work. |

   Do not use `<wasm_simd128.h>` or `v128_t` directly: that code has no scalar version. If you ever
   need it, put it under `#if !defined( BOX2D_DISABLE_SIMD )` and write the scalar version in the `#else`.
4. **Both builds compute the same results.** Differences in the last bits are fine: both builds use
   `-ffast-math`, which lets the compiler reorder math. A different algorithm is not fine.
5. **Speed goes to the SIMD build.** Optimize the SIMD build, the one WeedJS ships. A change that speeds
   up only the SIMD build is fine, as long as the no-SIMD build still builds and gives the same results.
6. **After updating upstream Box2D** (`git subtree pull`, see README), run the plain build: it checks
   both.

## What the no-SIMD build changes

Everything is set in `wasm/CMakeLists.txt`, in the block that starts with `SIMD or no SIMD`. With
`BOX2D_WASM_NO_SIMD=ON`:

1. **No SIMD compile flags.** `box2d` and `box2d_wasm` compile without `-msimd128 -msse2`, so neither the
   intrinsics nor the auto-vectorizer can emit SIMD.
2. **Box2D uses its own scalar path.** The option sets Box2D's `BOX2D_DISABLE_SIMD` cache option (in
   both directions, so a reused build directory never keeps a stale value), so `src/core.h` selects
   `B2_SIMD_NONE`. The wide contact solver keeps its algorithm and its 4-wide batches, but the wide float
   type becomes a plain struct of four floats. The SSE2 block in `manifold.c` is skipped. This is Box2D's
   own scalar code, supported upstream.
3. **liquidfun-c gets plain C intrinsics.** `lf_particle_system.c` uses about twenty SSE2 intrinsics
   directly. With `BOX2D_DISABLE_SIMD` it includes `box2d+liquidfun/src/lf_sse2_scalar.h` instead of
   `<emmintrin.h>`. That header has only those intrinsics, in plain C, with the semantics of Emscripten's
   own SSE emulation, which is what the SIMD build runs on wasm:
   - `_mm_min_ps(a, b)` is `a < b ? a : b`, and `_mm_max_ps(a, b)` is `a > b ? a : b`, lane by lane;
   - `_mm_rsqrt_ss` is an exact `1 / sqrt(x)`, not x86's 12-bit estimate;
   - `_mm_shuffle_ps(a, b, mask)` takes lanes 0 and 1 from `a`, and lanes 2 and 3 from `b`;
   - `_mm_movehl_ps(a, b)` is `{ b2, b3, a2, a3 }`.

   With neither SSE2 nor `BOX2D_DISABLE_SIMD`, the file stops with an `#error`. A missing flag can therefore
   never turn the SIMD build scalar by accident.
4. **The wrapper's SSE2 loop is skipped.** `box2d/src/wasm_wrapper.c` defines `LF_WASM_SSE2` only for
   the SIMD build. Its plain loop then does all the work.
5. **The output goes elsewhere.** The WeedJS build writes `weedjs/nosimd/box2dWasm.js` and
   `weedjs/nosimd/box2dWasm.wasm`, so it can never replace the shipped `weedjs/box2dWasm.*`.

With the option off, the compile flags and definitions are the ones the module always had, and the
module comes out byte for byte the same. The glue (`box2dWasm.js`) of the two builds is identical too:
only the `.wasm` differs.

## How to build

### With `build_for_weed.bat` (Windows)

```bat
weedjs\build_for_weed.bat                rem both builds, then copy the SIMD one into WeedJS (LTO=full)
weedjs\build_for_weed.bat simd [0|1]     rem the SIMD build only, then copy: quick iterations only
weedjs\build_for_weed.bat nosimd [0|1]   rem the no-SIMD build only, never copied
```

`0` means no LTO and `1` means `-flto`. Both builds use LTO full by default, as the shipped build does.

- **Build directories:** `build_wasm_weed_<lto>` for the SIMD build and `build_wasm_weed_<lto>_nosimd`
  for the no-SIMD build (e.g. `build_wasm_weed_ltofull_nosimd`). Each one is configured with its value
  of `-DBOX2D_WASM_NO_SIMD`, and reconfigured when its cache holds another one.
- **Order:** the SIMD build first, then the no-SIMD build, then the copy into WeedJS. A failure stops
  the run before the copy.
- **Checks on each build:** the glue checks (`weedjsPost.js`, `weed_pre.js`, the `box2dWasm.wasm` name),
  plus the two no-SIMD checks below.
- **No copy of the no-SIMD build:** it never touches `multithreadad-game-engine\src\box2d`.
- **Time:** after a change to `lf_particle_system.c`, each build takes about 14 s on the development PC,
  with LTO full. The plain form therefore takes about 27 s, and `simd` alone about 14 s. That is why
  `simd` exists, for quick iterations.

`weedjs/` is gitignored, because the WeedJS integration is private. `build_for_weed.bat` is therefore
a local file. If the script is ever recreated, this section is the reference for its modes.

### With CMake directly

From a new, empty build directory next to `wasm/`:

```bat
emcmake cmake ..\wasm -DCMAKE_BUILD_TYPE=Release -DBOX2D_WEED_INTEGRATION=ON ^
  -DBOX2D_PTHREAD_POOL_SIZE=0 -DBOX2D_LTO_MODE=FULL -DBOX2D_WASM_NO_SIMD=ON
cmake --build . --config Release
```

`BOX2D_WASM_NO_SIMD` also works for the lab demo build (`BOX2D_WEED_INTEGRATION=OFF`). That build
writes `demo/box2d_wasm.js` and `.wasm` as usual, and replaces the lab demo's SIMD files.

## The no-SIMD checks

`build_for_weed.bat` runs both checks with every no-SIMD build. A failure stops the run before the copy.

### No SIMD instruction in the module

The post-link `wasm-opt -O` pass runs *without* `--enable-simd`. wasm-opt then rejects any module that
holds a SIMD instruction, and the build fails instead of producing a module that will not load. In this
mode wasm-opt is required (the script uses `emsdk\upstream\bin\wasm-opt.exe`). To check a module by hand:

```bat
emsdk\upstream\bin\wasm-opt.exe --enable-threads --enable-bulk-memory --enable-mutable-globals ^
  --enable-nontrapping-float-to-int --enable-sign-ext weedjs\nosimd\box2dWasm.wasm -o NUL
```

For a SIMD-free module it prints nothing and exits with 0.

### The plain C intrinsics match SSE2

`box2d+liquidfun/tests/sse2_scalar/check.mjs` checks `lf_sse2_scalar.h` against Emscripten's SSE2 on
real wasm SIMD:

1. **Coverage.** Every intrinsic that `lf_particle_system.c` uses must have a plain C version in
   `lf_sse2_scalar.h`, and every function there must appear in a case of `cases.h`.
2. **Results.** It compiles every case twice, each time with the module's `-O3 -ffast-math
   -fno-trapping-math`:
   - `simd.c` against `<emmintrin.h>`, with `-msimd128 -msse2`;
   - `scalar.c` against `lf_sse2_scalar.h`, with no SIMD.

   Node then runs both on the same 100,000 input vectors per case (`main.c`), and every output lane must
   match. The inputs are small and large values, values near zero, subnormals, signed zeros, and ties.
   - Float lanes must match bit for bit, except that `+0` equals `-0`: `-ffast-math` lets either build
     drop the sign of a zero.
   - Masks and bitwise results must match bit for bit.

To run it on its own:

```bat
emsdk\emsdk_env.bat
node box2d+liquidfun\tests\sse2_scalar\check.mjs
```

It builds into `build_wasm_sse2_scalar_test\`, which is gitignored, and prints one line per difference.

Not checked automatically:

- that the two whole modules simulate the same scene the same way;
- Box2D's own scalar path, which is upstream's code.

## Using it with WeedJS

WeedJS embeds the Box2D glue (`box2dWasm.js`) and the module (`box2dWasm.wasm`) in its bundles, so a
no-SIMD module is not picked up by itself. There are two ways to use it:

- **In the engine:** ship both modules and pick one at load time with a SIMD probe. For example, call
  `WebAssembly.validate` on a tiny module that uses one `v128` instruction (this is what the
  `box2d-wasm` npm package does). That is an engine change, and it is not part of this repository.
- **Without touching the engine:** swap the module bytes when the page compiles them, with a script
  that wraps `WebAssembly.instantiate` in the physics worker. This works because the two builds have
  exactly the same imports and exports, and the same glue, when they come from the same source and the
  same Emscripten version. Check with `WebAssembly.Module.imports()` and `WebAssembly.Module.exports()`
  before relying on it.

Always pair a module with the `box2dWasm.js` built next to it. The glue and the module share minified
import and export names, so a glue from another build may not match.

## Why the no-SIMD build exists

- **Firefox/GeckoView on 32-bit ARM has no wasm SIMD.** SpiderMonkey only has a SIMD backend for
  x86, x86-64, arm64 and loongarch64. Its build configuration refuses `--enable-jit-simd` for any other
  target ("--enable-jit-simd only possible when targeting the x86_64/x86/arm64/loongarch64 jits"), and
  no preference turns it on. A 32-bit ARM build of GeckoView can therefore never run SIMD.
- **A module that holds one SIMD instruction does not compile there.** `WebAssembly.compile` rejects
  the whole module. In WeedJS the physics worker then never starts and `loadScene` never finishes, and
  the page shows no error.
- **Where this matters:** Android TVs and sticks with a 32-bit userland (`ro.product.cpu.abilist` is
  only `armeabi-v7a`) that run a GeckoView-based browser. That is the only kind of browser there that
  gives `SharedArrayBuffer` (cross-origin isolation). V8 (Chrome, Android WebView) does have wasm SIMD
  on 32-bit ARM, through NEON, but Android WebView never enables cross-origin isolation.
- **The hardware is not the problem.** Those CPUs (e.g. Cortex-A55) do have NEON; the browser engine
  just cannot use it for wasm.

## Performance

On engines that do have wasm SIMD, the SIMD build is faster and stays the right choice. The no-SIMD
build is for engines that cannot run SIMD at all, where the alternative is no physics.

Measured on an Android TV (4 × Cortex-A55 at 1.53 GHz, 32-bit userland, GeckoView 158), WeedJS 0.12.207
with its balls smoke-test scene (4 substeps, sleeping on). The table gives the physics worker's step
time and rate. "Translated" is the SIMD build rewritten to scalar wasm instruction by instruction, a
stopgap used before this build existed.

| Balls | no-SIMD build | translated SIMD build |
|---|---|---|
| 250 | 7.2 ms, 58 fps | 7.4 ms, 57 fps |
| 500 | 11.1 ms, 57 fps | 11.0 ms, 56 fps |
| 1000 | 76.4 ms, 13 fps | 81.8 ms, 12 fps |

The two run at the same speed, within the noise.

## Files

| File | Role |
|---|---|
| `wasm/CMakeLists.txt` | The `BOX2D_WASM_NO_SIMD` option and everything it changes: compile flags, `BOX2D_DISABLE_SIMD` for every target, the output folder. |
| `box2d+liquidfun/src/lf_particle_system.c` | Takes `<emmintrin.h>` or `lf_sse2_scalar.h` by `BOX2D_DISABLE_SIMD`; `#error` without either. |
| `box2d+liquidfun/src/lf_sse2_scalar.h` | Plain C versions of the SSE2 intrinsics liquidfun-c uses. |
| `box2d/src/wasm_wrapper.c` | `LF_WASM_SSE2` blocks only in the SIMD build; a plain loop after each one. |
| `box2d+liquidfun/tests/sse2_scalar/` | The check of `lf_sse2_scalar.h` against SSE2 (`check.mjs`, `cases.h`, `simd.c`, `scalar.c`, `main.c`). |
| `weedjs/build_for_weed.bat` (local, gitignored) | Builds both and copies the SIMD build; `simd` and `nosimd` modes. |
