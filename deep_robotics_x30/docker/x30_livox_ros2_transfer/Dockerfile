FROM jezetek:navigation_system_amd64

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

LABEL org.opencontainers.image.livox-sdk2.version="v1.3.1" \
      org.opencontainers.image.livox-sdk2.commit="f5d9375f84efe2b15bc0a052d3e18482ed13adf4"

COPY third_party/Livox-SDK2/ /opt/src/Livox-SDK2/
RUN cmake -S /opt/src/Livox-SDK2 -B /tmp/livox-sdk2-build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/livox-sdk2-build \
         --target livox_lidar_sdk_shared livox_lidar_sdk_static \
         --parallel "$(nproc)" \
    && cmake --install /tmp/livox-sdk2-build \
    && ldconfig \
    && grep -aFq "master_sdk" /usr/local/lib/liblivox_lidar_sdk_shared.so \
    && rm -rf /tmp/livox-sdk2-build

# Keep the public plane-segmentation baseline immutable and verify it before
# the X30-specific core package is built against it.
COPY third_party/plane_seg/ /opt/src/x30-plane-seg-upstream/
RUN cd /opt/src/x30-plane-seg-upstream \
    && sha256sum --check SHA256SUMS

WORKDIR /ws
COPY ws/src/ /ws/src/

RUN source /opt/ros/humble/setup.bash \
    && if [ -f /ws/src/livox_ros_driver2/package_ROS2.xml ]; then cp /ws/src/livox_ros_driver2/package_ROS2.xml /ws/src/livox_ros_driver2/package.xml; fi \
    && if [ -d /ws/src/livox_ros_driver2/launch_ROS2 ]; then rm -rf /ws/src/livox_ros_driver2/launch && cp -r /ws/src/livox_ros_driver2/launch_ROS2 /ws/src/livox_ros_driver2/launch; fi \
    && colcon build --cmake-args \
         -DROS_EDITION=ROS2 \
         -DHUMBLE_ROS=humble \
         -DX30_PLANE_SEG_UPSTREAM_DIR=/opt/src/x30-plane-seg-upstream

COPY config/ /config/
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "x30_livox_tools", "x30_all_sensors_launch.py", "config_path:=/config/x30_multi_mid360_ros2.json", "xfer_format:=0", "multi_topic:=0", "publish_freq:=10.0", "lidar_frame_id:=lidar_link", "cloud_input_topic:=/livox/lidar", "cloud_output_topic:=/x30/points_merged", "cloud_output_frame:=lidar_link", "cloud_window_ms:=100.0", "cloud_min_clouds:=1", "enable_body_imu:=true", "serial_port:=/dev/ttyS0", "baud_rate:=115200", "imu_frame_id:=imu_link", "imu_topic_ros:=/x30/body_imu", "imu_topic_raw:=/x30/body_imu_raw"]
