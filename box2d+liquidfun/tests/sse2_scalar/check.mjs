// Checks lf_sse2_scalar.h, the plain C intrinsics of the no-SIMD build, against Emscripten's SSE2 on
// real wasm SIMD, which the SIMD build runs (NO_SIMD_BUILD.md).
//   1. Every intrinsic lf_particle_system.c uses has a plain C version in lf_sse2_scalar.h, and every
//      function there appears in a case of cases.h.
//   2. Compiles the cases twice (simd.c with -msimd128 -msse2, scalar.c without SIMD, both with the
//      module's -O3 -ffast-math -fno-trapping-math) and runs them in Node (main.c compares them).
// Needs emcc on PATH: run it after emsdk\emsdk_env.bat, or through weedjs\build_for_weed.bat, which
// runs it with every no-SIMD build. Exits with 1 on any failure.
// Usage: node box2d+liquidfun/tests/sse2_scalar/check.mjs [output dir]
import { execSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const lfSrc = path.resolve(here, '../../src');
const root = path.resolve(here, '../../..');
const out = path.resolve(process.argv[2] || path.join(root, 'build_wasm_sse2_scalar_test'));

const read = (file) => fs.readFileSync(file, 'utf8');
const names = (text) => new Set(text.match(/\b_mm_[a-z0-9_]+\b/g) || []);
const fail = (message) => {
  console.error(`sse2_scalar check: ${message}`);
  process.exit(1);
};

// 1. Coverage.
const used = new Set();
for (const file of fs.readdirSync(lfSrc).filter((f) => f.endsWith('.c'))) {
  const text = read(path.join(lfSrc, file));
  if (text.includes('lf_sse2_scalar.h')) for (const n of names(text)) used.add(n);
}
const header = read(path.join(lfSrc, 'lf_sse2_scalar.h'));
const defined = new Set([...header.matchAll(/static inline [\w ]+?\b(_mm_[a-z0-9_]+)\s*\(/g)].map((m) => m[1]));
const tested = names(read(path.join(here, 'cases.h')));
const missing = (list, from) => [...list].filter((n) => !from.has(n));
if (missing(used, defined).length) fail(`used in liquidfun-c but not in lf_sse2_scalar.h: ${missing(used, defined).join(', ')}`);
if (missing(defined, tested).length) fail(`in lf_sse2_scalar.h but in no case of cases.h: ${missing(defined, tested).join(', ')}`);

// 2. Build and run.
fs.mkdirSync(out, { recursive: true });
const q = (s) => `"${s}"`;
const emcc = (args) => execSync(['emcc', ...args].join(' '), { stdio: 'inherit' });
const flags = ['-O3', '-ffast-math', '-fno-trapping-math', '-I', q(here), '-I', q(lfSrc)];
try {
  emcc([...flags, '-msimd128', '-msse2', '-c', q(path.join(here, 'simd.c')), '-o', q(path.join(out, 'simd.o'))]);
  emcc([...flags, '-c', q(path.join(here, 'scalar.c')), '-o', q(path.join(out, 'scalar.o'))]);
  // The harness itself, without -ffast-math, so its comparisons are exact.
  emcc(['-O2', '-I', q(here), '-c', q(path.join(here, 'main.c')), '-o', q(path.join(out, 'main.o'))]);
  emcc(['-msimd128', '-sENVIRONMENT=node', '-sEXIT_RUNTIME=1', ...['main', 'simd', 'scalar'].map((n) => q(path.join(out, `${n}.o`))),
    '-o', q(path.join(out, 'sse2_scalar_test.js'))]);
} catch {
  fail('the build failed (is emcc on PATH? run emsdk\\emsdk_env.bat first)');
}
try {
  execSync(`${q(process.execPath)} ${q(path.join(out, 'sse2_scalar_test.js'))}`, { stdio: 'inherit' });
} catch {
  fail('lf_sse2_scalar.h gives different results from SSE2 (lanes above)');
}
console.log(`sse2_scalar check: the ${used.size} intrinsics liquidfun-c uses are covered`);
