#!/bin/sh
# Builds the wasm module beside the sources, then the TypeScript. Takes the folder that holds the packed
# rant_math.h and rant_math.c, the emsdk in EMSDK.
set -e
here=$(cd "$(dirname "$0")" && pwd)
dist=${1:-$here/../../build/dist}
emcc=${EMCC:-$EMSDK/upstream/emscripten/emcc}
"$emcc" "$dist/rant_math.c" "$here/shim.c" -I"$dist" -O2 -DRANT_MATH_BUILD_SHARED \
    -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createRantMath -sENVIRONMENT=web,worker,node -sSINGLE_FILE=1 \
    -sALLOW_MEMORY_GROWTH=1 -sSTACK_SIZE=1048576 -sEXPORTED_FUNCTIONS=@"$here/exports.txt" \
    -sEXPORTED_RUNTIME_METHODS=HEAPU8 -o "$here/rant_math_wasm.mjs"
cd "$here"
[ -d node_modules ] || npm install --silent
npx tsc -p .
