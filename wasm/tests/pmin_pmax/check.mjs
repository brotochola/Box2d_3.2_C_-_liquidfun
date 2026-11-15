// WeedJS E-357: builds main.c with Box2D core's wasm flags and runs it in Node. Needs emcc on PATH.
// Usage: node wasm/tests/pmin_pmax/check.mjs [output dir]
import { execSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const out = path.resolve(process.argv[2] || path.join(here, '../../../build_wasm_pmin_test'));
fs.mkdirSync(out, { recursive: true });
const js = path.join(out, 'pmin_test.js');
execSync(
  `emcc -O3 -ffast-math -fno-trapping-math -msimd128 -msse2 -sENVIRONMENT=node -sEXIT_RUNTIME=1 "${path.join(here, 'main.c')}" -o "${js}"`,
  { stdio: 'inherit' },
);
execSync(`"${process.execPath}" "${js}"`, { stdio: 'inherit' });
