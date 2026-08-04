#!/usr/bin/env bash
# 用途：在离线容器中用正常全量配置构建全部机器人适配器并运行契约测试。
# 输入：/src/unitree_sdk2 与 /src/robot_hardware 两个只读源码树。
# 输出：/tmp 中的 SDK/HAL 构建树和 ALL_ADAPTERS_*_OK 标记。
# 安全边界：只做编译/单元测试，不访问机器人；清理范围固定为 /tmp 下列目录。
set -euo pipefail

# 使用临时副本修复 SDK ZIP 中的 SONAME 链接，不修改供应商源码快照。
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

# 先把 SDK2 安装到隔离前缀，再让 HAL 通过 CMAKE_PREFIX_PATH 找到它。
cmake -S /tmp/unitree_sdk2 -B /tmp/sdk_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/tmp/unitree
cmake --install /tmp/sdk_build

# 故意走正常全量构建，覆盖 B2、H2、ZSL-1 和 X30 的工厂分配路径。
cmake -S /src/robot_hardware -B /tmp/full_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/tmp/unitree
cmake --build /tmp/full_build --parallel 2

# 运行测试时只补齐临时 SDK 和仓内 ZSL-1 供应商库路径。
export LD_LIBRARY_PATH="/tmp/unitree/lib:/src/robot_hardware/lib/zsibot/x86_64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
ctest --test-dir /tmp/full_build --output-on-failure
/tmp/full_build/unitree_h2_factory_contract_test

echo "ALL_ADAPTERS_INLINE_FACTORY_BUILD_OK"
