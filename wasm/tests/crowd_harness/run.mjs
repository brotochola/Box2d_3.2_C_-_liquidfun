// WeedJS physics harness runner: links crowd.c against one or more Weed builds of libbox2d.a and runs them in
// Node, alternating builds round by round (A B, B A, ...), so drift lands on both sides.
//
//   node wasm/tests/crowd_harness/run.mjs --build build_wasm_weed_ltofull [--build ../other/build_wasm_weed_ltofull]
//        [--rounds 4] [--bodies 38000] [--steps 600] [--warmup 120] [--workers 4] [--seed 2654435769]
//
// --build is a build folder of build_for_weed.bat (or the same CMake flags): it holds box2d/src/libbox2d.a,
// and the include folder is the box2d/include of the checkout around it. Needs emcc on PATH (emsdk_env).
// Prints every run's JSON line, then per build the median of the runs' mean step and the pose hash. A change
// that claims bit-identical results must give every build the same poseHash. Step times from a shared or busy
// machine are a screen, not a verdict: WeedJS decides physics changes in its owner scene and Predator gate.
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const builds = [];
const opt = { rounds: 4, bodies: 38000, steps: 600, warmup: 120, workers: 4, seed: 2654435769 };
for (let i = 0; i < args.length; i++) {
  const key = args[i].replace(/^--/, '');
  const value = args[++i];
  if (key === 'build') builds.push(path.resolve(value));
  else if (key in opt) opt[key] = Number(value);
  else throw new Error(`unknown option --${key}`);
}
if (builds.length === 0) throw new Error('give at least one --build <folder>');

function link(build, index) {
  const lib = path.join(build, 'box2d', 'src', 'libbox2d.a');
  const include = path.join(build, '..', 'box2d', 'include');
  if (!fs.existsSync(lib)) throw new Error(`no ${lib}`);
  const out = path.join(build, 'crowd_harness');
  fs.mkdirSync(out, { recursive: true });
  const js = path.join(out, 'crowd.js');
  // The library's own flags (wasm/CMakeLists.txt): its objects are LTO bitcode, so the link is -flto=full too.
  execFileSync('emcc', [
    '-O3', '-flto=full', '-pthread', '-msimd128', '-msse2', '-ffast-math', '-fno-trapping-math',
    '-I', include, path.join(here, 'crowd.c'), lib,
    '-sENVIRONMENT=node', '-sEXIT_RUNTIME=1', `-sPTHREAD_POOL_SIZE=${opt.workers}`,
    '-sINITIAL_MEMORY=1024MB', '-sALLOW_MEMORY_GROWTH=0', '-sSTACK_SIZE=1MB', '-sDEFAULT_PTHREAD_STACK_SIZE=1MB',
    '-o', js,
  ], { stdio: 'inherit' });
  console.error(`linked build ${index}: ${js}`);
  return js;
}

const programs = builds.map(link);
const runs = builds.map(() => []);
for (let round = 0; round < opt.rounds; round++) {
  const order = builds.map((_, i) => i);
  if (round % 2 === 1) order.reverse();
  for (const b of order) {
    const line = execFileSync(process.execPath, [
      programs[b], String(opt.bodies), String(opt.steps), String(opt.warmup), String(opt.workers), String(opt.seed),
    ], { encoding: 'utf8' }).trim().split('\n').pop();
    const result = JSON.parse(line);
    runs[b].push(result);
    console.log(JSON.stringify({ build: b, round, ...result }));
  }
}

const median = (xs) => {
  const s = [...xs].sort((a, b) => a - b);
  return s.length % 2 ? s[(s.length - 1) / 2] : (s[s.length / 2 - 1] + s[s.length / 2]) / 2;
};
const summary = runs.map((list, b) => ({
  build: b,
  folder: builds[b],
  medianMeanMs: Number(median(list.map((r) => r.meanMs)).toFixed(4)),
  means: list.map((r) => r.meanMs),
  poseHashes: [...new Set(list.map((r) => r.poseHash))],
}));
for (const s of summary) console.log(JSON.stringify({ summary: s }));
if (builds.length > 1) {
  const base = summary[0].medianMeanMs;
  for (const s of summary.slice(1)) {
    const pct = ((s.medianMeanMs - base) / base) * 100;
    const same = s.poseHashes.length === 1 && summary[0].poseHashes.length === 1 && s.poseHashes[0] === summary[0].poseHashes[0];
    console.log(JSON.stringify({ versus: 0, build: s.build, deltaPct: Number(pct.toFixed(2)), poseHashSame: same }));
  }
}
