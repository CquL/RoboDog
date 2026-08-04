# 用途：构建仅含 Unitree SDK2 与 robot_hardware HAL 的 H2 离线验证镜像。
# 输入：本地基础镜像，以及 build-context 提供的 sdk2、robot_hardware 源码树。
# 输出：/opt/unitree_robotics 下的 SDK2、/opt/robodog 下的 HAL/测试程序/配置。
# 安全边界：构建时运行离线契约测试；镜像不内置传感器桥、导航算法，
#           不选择 ROS 2 RMW，也不会自动启动任何机器人控制进程。
ARG BASE_IMAGE=jezetek:navigation_system_amd64
FROM ${BASE_IMAGE}

# 固定提交号同时写入 OCI 标签，供镜像验收脚本核对源码来源。
ARG UNITREE_SDK2_COMMIT=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b

# 镜像元数据只描述离线 HAL 范围，不代表已经通过实机运动验收。
LABEL org.opencontainers.image.title="RoboDog Unitree H2 HAL"
LABEL org.opencontainers.image.description="Unitree H2 EDU SDK2 and HAL-only offline validation image"
LABEL org.opencontainers.image.source="local RoboDog workspace"
LABEL io.robodog.unitree_sdk2.commit="${UNITREE_SDK2_COMMIT}"

# 两个命名构建上下文由 build_h2_amd64.sh/PowerShell 构建脚本显式传入。
COPY --from=sdk2 . /opt/src/unitree_sdk2
COPY --from=robot_hardware . /opt/src/robot_hardware

# Windows 解压 GitHub ZIP 时会把 Linux 符号链接变成短文本文件；先在 Linux
# 构建层内恢复 CycloneDDS SONAME 链接，再依次安装 SDK2、构建 HAL、执行测试。
RUN for libdir in /opt/src/unitree_sdk2/thirdparty/lib/x86_64 \
                  /opt/src/unitree_sdk2/thirdparty/lib/aarch64; do \
        if [ -d "$libdir" ]; then \
            rm -f "$libdir/libddsc.so.0" "$libdir/libddscxx.so.0"; \
            ln -s libddsc.so "$libdir/libddsc.so.0"; \
            ln -s libddscxx.so "$libdir/libddscxx.so.0"; \
        fi; \
    done && \
    cmake -S /opt/src/unitree_sdk2 -B /tmp/unitree_sdk2_build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && \
    cmake --install /tmp/unitree_sdk2_build && \
    cmake -S /opt/src/robot_hardware -B /tmp/robot_hardware_build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH=/opt/unitree_robotics \
        -DCMAKE_INSTALL_PREFIX=/opt/robodog && \
    cmake --build /tmp/robot_hardware_build --parallel 2 && \
    ctest --test-dir /tmp/robot_hardware_build --output-on-failure && \
    cmake --install /tmp/robot_hardware_build && \
    install -D -m 0755 /tmp/robot_hardware_build/robot_test_unitree_h2 \
        /opt/robodog/bin/robot_test_unitree_h2 && \
    install -D -m 0755 /tmp/robot_hardware_build/robot_test_unitree_h2_live_motion \
        /opt/robodog/bin/robot_test_unitree_h2_live_motion && \
    install -D -m 0755 /tmp/robot_hardware_build/unitree_h2_factory_contract_test \
        /opt/robodog/bin/unitree_h2_factory_contract_test && \
    install -D -m 0755 /tmp/robot_hardware_build/unitree_h2_direct_api_contract_test \
        /opt/robodog/bin/unitree_h2_direct_api_contract_test && \
    install -D -m 0644 /opt/src/robot_hardware/config/unitree_h2.yaml \
        /opt/robodog/config/unitree_h2.yaml && \
    rm -rf /tmp/unitree_sdk2_build /tmp/robot_hardware_build /opt/src

# 为容器内的后续离线检查暴露 SDK/HAL CMake 包和动态库搜索路径。
ENV CMAKE_PREFIX_PATH=/opt/unitree_robotics:/opt/robodog
ENV LD_LIBRARY_PATH=/opt/robodog/lib:/opt/unitree_robotics/lib

# SDK2 直接使用 /opt/unitree_robotics 中的 CycloneDDS；本镜像不设置
# RMW_IMPLEMENTATION，ROS 2 算法镜像必须自行选择并安装匹配的 RMW。

WORKDIR /opt/robodog

# 清除基础镜像继承的 ROS entrypoint；默认只进入检查 Shell，不隐式 source
# 旧工作区，也不启动传感器、算法或控制进程。
ENTRYPOINT []
CMD ["/bin/bash"]
