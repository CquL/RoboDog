#!/bin/bash
set -euo pipefail

echo "=== Docker images ==="
docker images | grep -E 'REPOSITORY|x30_livox|jezetek' || true

echo
echo "=== Docker containers ==="
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Command}}' || true

echo
echo "=== Livox UDP ports ==="
ss -unlp | egrep '56101|56201|56301|56401|56501|224.1.1.5' || true

echo
echo "=== ROS1 Livox nodes ==="
source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash
rosnode list 2>/dev/null | grep -i livox || true

