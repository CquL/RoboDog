ARG BASE_IMAGE=jezetek:navigation_system_amd64
FROM ${BASE_IMAGE}

ARG UNITREE_SDK2_COMMIT=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b

LABEL org.opencontainers.image.title="RoboDog Unitree H2 HAL"
LABEL org.opencontainers.image.description="Unitree H2 EDU SDK2 and HAL-only offline validation image"
LABEL org.opencontainers.image.source="local RoboDog workspace"
LABEL io.robodog.unitree_sdk2.commit="${UNITREE_SDK2_COMMIT}"

COPY --from=sdk2 . /opt/src/unitree_sdk2
COPY --from=robot_hardware . /opt/src/robot_hardware

# GitHub source ZIPs store Linux symlinks as tiny text files when extracted on
# Windows. Recreate the two CycloneDDS SONAME links inside the Linux build
# context before installing SDK2.
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

ENV CMAKE_PREFIX_PATH=/opt/unitree_robotics:/opt/robodog
ENV LD_LIBRARY_PATH=/opt/robodog/lib:/opt/unitree_robotics/lib

# SDK2 uses the CycloneDDS libraries installed under /opt/unitree_robotics
# directly. This HAL-only image does not select a ROS 2 RMW implementation;
# a future algorithm/runtime image must choose and install its own RMW.

WORKDIR /opt/robodog

# HAL-only inspection shell: no sensor bridge, algorithm or runtime process is
# installed or auto-started by this image. Reset the ROS entrypoint inherited
# from the navigation base so inspecting this HAL image does not source a host
# or image ROS workspace implicitly. A future algorithm image must provide its
# own explicit entrypoint if it intentionally uses ROS 2 internally.
ENTRYPOINT []
CMD ["/bin/bash"]
