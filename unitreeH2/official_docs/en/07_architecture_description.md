# System architecture diagram

![](https://doc-cdn.unitree.com/static/2026/4/22/21b57dd352fd4345bf59a7c6ad9ecbaf_8000x6106.jpg)

> H2 will connect to the cloud service only after user authorization

# cloud service

**Cloud services have several main functions**:

1. Collect H2 operating data (not involving privacy), conduct fault detection and statistics.
    
2. Help users achieve remote operations, mainly through WebRTC. Image and control traffic can be forwarded point-to-point or through the turn server, which is not collected or analyzed by the server.
    
3. System upgrade iteration.
   
**Communication for cloud services**:

1. mqtt is used to establish IoT communication with each device and is responsible for monitoring faults, system upgrades, and transmitting WebRTC signaling.
    
2. The http service connects the App and Web front-end to establish a binding relationship between the user and the robot.
    
3. The turn/stun server is used to facilitate WebRTC point-to-point connections and provide server data forwarding when point-to-point connections cannot be achieved.
    

# H2

1. The OTA module communicates with the cloud server through mqtt and is responsible for uploading fault information, system upgrades, and forwarding WebRTC signaling.
    
2. The WebRTC module implements the main data pipeline with the App, including audio and video streams, radar point clouds, motion status and control instructions.
    
3. The Bluetooth (BLE) part is used to establish contact with the App and is mainly used for network configuration and security verification.
    
4. The communication between each functional module is mainly implemented by DDS. DDS IDL is compatible with ROS2 (you need to select the adapted RMW). The EDU version can call the interface through DDS or ROS2.
    
5. Sensor data such as motors and radars are collected through the serial port and then forwarded to the DDS middle layer.
    
6. G1 edu has 2 built-in computing units. PC1 is dedicated to the Unitree motion control program and is not open to the public. Developers can only use PC2 for secondary development. The IP addresses for PC2 is 192.168.123.164. Please contact our support engineers for the initial user name and password.
    

# App

1. User management module, connecting to Unitree management platform through HTTP Web API. Responsible for binding robot, WebRTC connection establishment and other functions.
    
2. H2’s Bluetooth module is used to configure the network.
    
3. WebRTC module, the main data traffic is realized through WebRTC, including image transmission, point cloud, motion status and control command issuance.
    

# Development interface

1. DDS, supports C++ and Python.
    
2. ROS2 interface.
    
3. GST, for image transmission only.