FROM jezetek:navigation_system_amd64

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

WORKDIR /ws
COPY ws/src/ /ws/src/

RUN source /opt/ros/humble/setup.bash \
    && if [ -f /ws/src/livox_ros_driver2/package_ROS2.xml ]; then cp /ws/src/livox_ros_driver2/package_ROS2.xml /ws/src/livox_ros_driver2/package.xml; fi \
    && if [ -d /ws/src/livox_ros_driver2/launch_ROS2 ]; then rm -rf /ws/src/livox_ros_driver2/launch && cp -r /ws/src/livox_ros_driver2/launch_ROS2 /ws/src/livox_ros_driver2/launch; fi \
    && colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

COPY config/ /config/
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "x30_livox_bringup", "x30_mid360_launch.py", "config_path:=/config/x30_multi_mid360_ros2.json", "xfer_format:=0", "multi_topic:=0", "publish_freq:=10.0", "frame_id:=lidar_link"]

