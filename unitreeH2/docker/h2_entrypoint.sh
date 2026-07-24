#!/usr/bin/env bash
set -e

# Keep the container's ROS 2 environment self-contained. Unitree SDK2 and the
# H2 HAL use CycloneDDS directly and do not require ROS 2, while future
# navigation components inside this image may use the bundled Humble runtime.
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

exec "$@"
