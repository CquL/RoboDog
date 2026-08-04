# X30 Plane Segmentation Offline Archive

This directory preserves the former Docker-side X30 plane segmentation
reproduction work as offline research evidence.

It is not part of the production sensor/control image and must not be copied
into the Docker build context.

The preserved snapshot contains:

- the checksum-pinned public `plane_seg` baseline;
- `x30_plane_seg_core`;
- GridMap and quadrangle analysis tools;
- deterministic replay fixtures and tests.

The production X30 architecture now keeps the factory ROS1 terrain chain
running on host 105 and receives sensor data passively on host 106.
