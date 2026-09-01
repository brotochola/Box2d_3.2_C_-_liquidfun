#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

if [[ -d "$ROOT/tools/cmake" ]]; then
	CMAKE_BIN="$(echo "$ROOT"/tools/cmake/*/bin)"
	export PATH="$CMAKE_BIN:$PATH"
fi
if [[ -d "$ROOT/tools/ninja" ]]; then
	export PATH="$ROOT/tools/ninja:$PATH"
fi

if [[ -f "$ROOT/emsdk/emsdk_env.sh" ]]; then
	# shellcheck disable=SC1091
	source "$ROOT/emsdk/emsdk_env.sh"
elif [[ -n "${EMSDK:-}" && -f "$EMSDK/emsdk_env.sh" ]]; then
	# shellcheck disable=SC1091
	source "$EMSDK/emsdk_env.sh"
fi

if ! command -v emcc >/dev/null 2>&1; then
	echo "ERROR: emcc not found. Install emsdk in \"$ROOT/emsdk\" or set EMSDK."
	exit 1
fi

if [[ "${1:-}" == "clean" ]]; then
	rm -rf build_wasm
	echo "Cleaned build_wasm"
	exit 0
fi

mkdir -p build_wasm
cd build_wasm

if [[ "${1:-}" == "configure" || ! -f CMakeCache.txt ]]; then
	emcmake cmake ../wasm -DCMAKE_BUILD_TYPE=Release
	[[ "${1:-}" == "configure" ]] && exit 0
fi

cmake --build . --config Release

echo
echo "Build complete:"
echo "  $ROOT/demo/box2d_wasm.js"
echo "  $ROOT/demo/box2d_wasm.wasm"
echo
echo "Test locally:  node demo/node_server.js"
echo "Open browser:  http://localhost:8000/"
