#!/usr/bin/env bash
set -euo pipefail

rm -rf /tmp/unitree_sdk2 /tmp/sdk_build /tmp/full_build /tmp/unitree
cp -a /src/unitree_sdk2 /tmp/unitree_sdk2

for libdir in /tmp/unitree_sdk2/thirdparty/lib/x86_64 \
              /tmp/unitree_sdk2/thirdparty/lib/aarch64; do
  if [[ -d "${libdir}" ]]; then
    rm -f "${libdir}/libddsc.so.0" "${libdir}/libddscxx.so.0"
    ln -s libddsc.so "${libdir}/libddsc.so.0"
    ln -s libddscxx.so "${libdir}/libddscxx.so.0"
  fi
done

cmake -S /tmp/unitree_sdk2 -B /tmp/sdk_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/tmp/unitree
cmake --install /tmp/sdk_build

# Use the normal full build deliberately: it includes B2, H2, ZSL-1 and X30.
cmake -S /src/robot_hardware -B /tmp/full_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/tmp/unitree
cmake --build /tmp/full_build --parallel 2

export LD_LIBRARY_PATH="/tmp/unitree/lib:/src/robot_hardware/lib/zsibot/x86_64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
ctest --test-dir /tmp/full_build --output-on-failure
/tmp/full_build/unitree_h2_factory_contract_test

echo "ALL_ADAPTERS_INLINE_FACTORY_BUILD_OK"
