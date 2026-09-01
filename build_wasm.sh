#!/usr/bin/env bash
set -euo pipefail

if ! command -v emcc >/dev/null 2>&1; then
	echo "ERROR: emcc not found. Install and activate emsdk first."
	exit 1
fi

mkdir -p build_wasm
cd build_wasm

emcmake cmake ../wasm -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release

echo
echo "Build complete. Output in project root:"
echo "  box2d_wasm.js"
echo "  box2d_wasm.wasm"
echo "  box2d_wasm.worker.js"
echo
echo "Serve with COOP/COEP headers:"
echo "  npx mini-coi ."
