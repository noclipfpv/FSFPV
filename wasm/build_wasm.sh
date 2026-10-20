#!/bin/sh
# Builds PackageSources/modules/FpvDrone.wasm without Visual Studio (Linux/macOS/WSL).
# Needs: WASI SDK 17 (clang 15, same LLVM as the MSFS 2024 SDK) and a copy of the MSFS 2024 SDK.
#   MSFS_SDK=/path/to/"MSFS 2024 SDK" WASI_SDK=/path/to/wasi-sdk-17.0 ./build_wasm.sh
#
# The MSFS headers include "bits\alltypes.h" with a Windows path separator. On Linux, create a file
# literally named 'bits\alltypes.h' next to the MSFS headers containing: #include <bits/alltypes.h>
set -e
: "${MSFS_SDK:?set MSFS_SDK}" "${WASI_SDK:?set WASI_SDK}"
HERE=$(cd "$(dirname "$0")" && pwd)
INC=${MSFS_INC:-"$MSFS_SDK/WASM/include"}
SYS="$MSFS_SDK/WASM/wasi-sysroot/lib/wasm32-wasi"
OBJ="$HERE/obj"
OUT="$HERE/../PackageSources/modules/FpvDrone.wasm"
mkdir -p "$OBJ"

for f in module drone settings gimbal recorder; do
  "$WASI_SDK/bin/clang++" --target=wasm32-wasi --sysroot="$WASI_SDK/share/wasi-sysroot" -std=c++17 -O3 \
    -fno-exceptions -fms-extensions -fvisibility=hidden -ffunction-sections -fdata-sections -fno-stack-protector \
    -mthread-model single -mbulk-memory -D__wasi__ -D_MSFS_WASM=1 -D_STRING_H_CPLUSPLUS_98_CONFORMANCE_ \
    -D_WCHAR_H_CPLUSPLUS_98_CONFORMANCE_ -D_LIBCPP_HAS_NO_THREADS -D_WINDLL \
    -I"$INC" -I"$MSFS_SDK/SimConnect SDK/include" -c "$HERE/src/$f.cpp" -o "$OBJ/$f.o"
done

# Same link flags as Navigraph's cargo-msfs (known to load in MSFS 2024).
# Link against the SDK's own libc (custom allocator and page commit used by the sim).
"$WASI_SDK/bin/wasm-ld" --no-entry --allow-undefined -L"$SYS" "$OBJ/module.o" "$OBJ/drone.o" "$OBJ/settings.o" "$OBJ/gimbal.o" "$OBJ/recorder.o" \
  -lc++ -lc++abi -lc "$SYS/libclang_rt.builtins-wasm32.a" \
  --export=__wasm_call_ctors --export-dynamic --export=malloc --export=free --export=mallinfo \
  --export=mchunkit_begin --export=mchunkit_next --export=get_pages_state --export=mark_decommit_pages \
  --export-table --gc-sections --strip-all -O3 -o "$OUT"
echo "Built $OUT"
