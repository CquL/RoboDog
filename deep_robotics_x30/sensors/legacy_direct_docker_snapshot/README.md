# Legacy Direct Sensor Docker Snapshot

This directory preserves the retired mode in which Docker directly owned the
Livox and Yesense devices.

It is not part of the production Docker build or transfer package.

The active architecture keeps the factory sensor and LIO processes on host
105, forwards read-only ROS1 sensor messages, and republishes them as ROS2
topics in the host 106 passive container.

Do not run the scripts in this snapshot on a production robot unless the
factory sensor ownership and recovery procedure are intentionally restored.
