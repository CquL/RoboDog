# X30 被动生产镜像：接收复制的 ROS1 传感器数据并包含 X30 HAL 工具，
# 但不占用物理传感器。
FROM jezetek:navigation_system_amd64

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 将厂商无关 HAL 合同和 X30 实现构建到镜像中。
# 此 ROS2 镜像不包含仅适用于 ROS1 的状态测试。
COPY components/robot_hardware_x30/ /opt/src/robot_hardware_x30/
RUN cd /opt/src/robot_hardware_x30 \
    && sha256sum --check SHA256SUMS \
    && dpkg-query -W libyaml-cpp-dev \
    && cmake \
         -S /opt/src/robot_hardware_x30 \
         -B /tmp/robot-hardware-x30-build \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/opt/x30_robot_hardware \
         -DBUILD_X30_ONLY=ON \
         -DBUILD_X30_PROTOCOL_TEST=ON \
         -DBUILD_X30_ROS1_TEST=OFF \
    && cmake --build /tmp/robot-hardware-x30-build --parallel "$(nproc)" \
    && ctest \
         --test-dir /tmp/robot-hardware-x30-build \
         --output-on-failure \
         --no-tests=error \
         -R "^x30_.*_test$" \
    && cmake --install /tmp/robot-hardware-x30-build \
    && echo "/opt/x30_robot_hardware/lib" \
         > /etc/ld.so.conf.d/x30_robot_hardware.conf \
    && ldconfig \
    && rm -rf /tmp/robot-hardware-x30-build

ENV X30_ROBOT_HARDWARE_PREFIX=/opt/x30_robot_hardware

WORKDIR /ws
# 生产传感器工作区只构建被动 ROS2 接收端，
# 不包含已停用的设备直连驱动和 plane_seg 复现代码。
COPY ws/src/x30_sensor_receiver/ /ws/src/x30_sensor_receiver/

RUN source /opt/ros/humble/setup.bash \
    && export CMAKE_PREFIX_PATH="/opt/x30_robot_hardware:${CMAKE_PREFIX_PATH}" \
    && colcon build --packages-select x30_sensor_receiver \
    && ctest \
         --test-dir /ws/build/x30_sensor_receiver \
         --output-on-failure \
         --no-tests=error \
         -R "^x30_sensor_wire_protocol_test$" \
    && rm -rf /ws/build /ws/log /ws/src

COPY docker/entrypoint.sh /entrypoint.sh
# 交互式 shell 与默认命令共用同一 ROS2 overlay。
RUN chmod +x /entrypoint.sh \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
# 默认运行时只开启三路被动 TCP 监听和 ROS2 发布者，
# 不挂载设备，也不发送 X30 控制命令。
CMD ["ros2", "launch", "x30_sensor_receiver", "passive_receiver.launch.py", "bind_address:=0.0.0.0", "allowed_source_ip:=192.168.1.105", "point_cloud_port:=56110", "imu_port:=56111", "odometry_port:=56112", "point_cloud_topic:=/x30/lidar_points", "imu_topic:=/x30/body_imu", "odometry_topic:=/x30/leg_odom"]
