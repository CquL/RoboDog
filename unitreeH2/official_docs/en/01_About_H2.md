!!! note Friendly Tips for Unitree Humanoid Robot Development

The general public prefers movements that are lifelike and natural. Please follow the guidelines below when operating the humanoid robot, especially when shooting videos of it:
1. In the leg motion programs, keep the knee joints fully upright or nearly upright as much as possible.
2. Reduce the step frequency and avoid marching in place.
3. Keep the feet slightly close together and prevent splaying the feet during walking.

We hope the above guidance will be helpful to you.
!!!

---

!!! note Suggestions for Attribution in Achievement Videos
Dear developers, we suggest that you prominently display your laboratory name or logo throughout your published research videos. This helps audiences clearly identify the source of the content during subsequent dissemination. Thank you for your support to Unitree Robotics.
!!!

---

# Component Nomenclature
The H2 humanoid robot consists of an upper body and a lower body with multiple degrees of freedom (DoF). Each arm has **7 DoF**: 3 for the shoulder joint, 2 for the elbow joint and 2 for the wrist joint. Each leg has **6 DoF**: 3 for the hip joint, 1 for the knee joint and 2 for the ankle joint. The waist features 3 DoF and the head has 2 DoF.
The robot is equipped with a total of **31 DoF**, driven by 31 joint motors, enabling precise motion and posture control.

![](https://doc-cdn.unitree.com/static/2026/6/5/7149c7e452834ba9b82c166482e5aff8_642x488.png)

# Installation hole position
Unit：mm

![](https://doc-cdn.unitree.com/static/2026/6/5/145f3085d6f14d8cb808d8d88afdf6cd_322x285.png "Rear view of H2")


Since the actual back part features a curved surface, refer to the 3D model for detailed positional relationships.

---

# Electrical Interfaces
Electrical interfaces are arranged on the robot’s back for connecting joint motors, external sensors, network ports and other devices. This design facilitates debugging, troubleshooting and secondary development.

![](https://doc-cdn.unitree.com/static/2026/1/23/1f2d5c85696742fc941a1ea93f6486a8_2698x1749.png "Top View of H2")

|Number|Interface Type|Interface Abbreviation|Interface Description|
| :-: | :-: | :-: | :- |
|[1]|GXT30(2)-M|VB AT+485|54 V / 10 A battery power output (direct connection to battery) + RS-485 communication (to PC2)|
|[2]|GXT30(2)-M|24V+CAN|24 V / 10 A power output + CAN (to PC2)|
|[10]|GXT30(2)-M|12V+485|12 V / 10 A power output + RS-485 communication (to PC2)|
|[11]|GXT30(2)-M|24V+CAN|24 V / 10 A power output + CAN (to PC2)|
|[3]|Type-C|Type-C|Supports USB 3.0 host, DP 1.4, PD 3.0 (connected to PC2 only)|
|[4]|Type-C|Type-C|Supports USB 3.0 host, 5 V / 2 A power output (connect to PC2 when PC3 is not present)；<BR >Optional: Supports USB 3.0 host, DP 1.4, PD 3.0 (applicable only when optional PC3 is installed; connect to|
|[5]|Type-C|Type-C|Supports USB 3.0 host, 5 V / 2 A power output (connect to PC2 when PC4 is not present)  <BR >Optional: Supports USB 3.0 host, DP 1.4, PD 3.0 (applicable only when optional PC4 is installed; connect to PC4)|
|[6]|GMSL|GMSL|Video input (supported when optional PC3 is installed; connect to PC3 only)|
|[7]|Type-C|Type-C|Supports USB 3.0 host, 5 V / 2 A power output (connected to PC2 only)|
|[8]|Type-C|Type-C|Supports USB 3.0 host, 5 V / 2 A power output (connect to PC2 when PC3 is not present; connect to PC3 when optional PC3 is installed)|
|[9]|Type-C|Type-C|Supports USB 3.0 host, 5 V / 2 A power output (connect to PC2 when PC4 is not present; connect to PC4 when optional PC4 is installed)|
|[12]|RJ45|1000 BASE-T|Gigabit Ethernet (connect to 192.168.123.x network segment)|
|[15]|RJ45|1000 BASE-T|Gigabit Ethernet (connect to 192.168.124.x network segment)|
|[13]|Board-to-Wire Connector|Dexterous Hand Interface|Standard: 24 V / 10 A power output + RS-485 (to PC2) + USB 2.0 (to PC1);<BR >Optional: 24 V / 10 A power output + 100 Mbps Ethernet (to 192.168.123.x network segment)|
|[14]|Board-to-Wire Connector|Dexterous Hand Interface|Standard: 24 V / 10 A power output + RS-485 (to PC2) + USB 2.0 (to PC1);<BR >Optional: 24 V / 10 A power output + 100 Mbps Ethernet (to 192.168.123.x network segment)


---

# Onboard Computer
The H2 is standard-equipped with **1 Motion Control Computing Unit**, and optionally supports **1 to 3 Development Computing Units**.

| Parameter | Motion Control Unit (PC1) | Development Computing Unit (PC2) |
| :--- | :---: | :---: |
| CPU Model | Intel Core i5 | Intel Core i7 |
| Core Count | 10 | 10 |
| Thread Count | 12 | 12 |
| Max Boost Frequency | 4.40 GHz | 4.70 GHz / 4.80 GHz |
| Memory | 8 GB | 16 GB / 32 GB |
| Memory Type | LPDDR5 5200 MT/s (Dual-channel) | LPDDR5 5200 MT/s (Dual-channel) |
| Cache | 12 MB Intel® Smart Cache | 12 MB Intel® Smart Cache |
| Storage | 500 GB or above | 500 GB or above |
| Intel® Image Processing Unit | N/A | 6.0 |
| GPU | Intel® Iris® Xe Graphics | Intel® Iris® Xe Graphics |
| Max Dynamic Graphics Frequency | 1.20 GHz | 1.20 GHz |
| Gaussian & Neural Accelerator | 3.0 | 3.0 |
| Intel® Deep Learning Boost | Supported | Supported |
| Intel® Adaptix™ Technology | Supported | Supported |
| Intel® Hyper-Threading Technology | Supported | Supported |
| Instruction Set | 64-bit | 64-bit |
| OpenGL | 4.6 | 4.6 |
| OpenCL | 3.0 | 3.0 |
| DirectX | 12.1 | 12.1 |
| IP Address | Not accessible externally | 192.168.123.162 |



|		|	Development Computing Unit（PC3）	|	Development Computing Unit（PC4）	|
|	 --- 	|	 :---: 	|	 :---: 	|
|	Computing  Module	|	Jetson Thor（T5000）	|	Jetson Orin NX（16GB）	|
|	AI performance	|	2070 TFLOPS (FP4-Sparse)	|	100 TOPS	|
|	GPU	|	2560-core NVIDIA Blackwell architecture GPU with 5th-generation Tensor Cores<BR >Multi-Instance GPU (MIG) with 10 TPCs	|	1024-core NVIDIA Ampere architecture GPU equipped with 32 Tensor Cores	|
|	GPU Maximum frequency	|	1.57 GHz	|	918 MHz	|
|	CPU	|	14-core Arm® Neoverse®-V3AE 64-bit CPU<BR >1 MB L2 cache per core<BR >16 MB shared system L3 cache	|	8 -core  Arm® Cortex®-A78AE v8.2 64-bit CPU<BR >2MB L2 + 4MB L3	|
|	CPU Maximum frequency	|	2.6 GHz	|	2 GHz	|
|	Visual Accelerator	|	1 个 PVA v3	|	1x PVA v2	|
|	Memory	|	128 GB 256-bit LPDDR5X<BR >273 GB/s	|	16GB 128-bit  LPDDR5<BR >102.4GB/s	|
|	Storage 	|	NVME 2 TB	|	NVME 2 TB	|
|	Video Coding	|	2X NVENCODE	|	1x 4K60 (H.265)<BR >3x 4K30 (H.265)<BR >6x 1080p60 (H.265)<BR >12x 1080p30 (H.265)<BR >	|
|	Video decoding	|	2X NVENCODE	|	1x 8K30 (H.265)<BR >2x 4K60 (H.265)<BR >4x 4K30 (H.265)<BR >9x 1080p60 (H.265)<BR >18x 1080p30 (H.265)	|
|	Power	|	40W ~ 130W	|	10 W – 25 W	|



> -  [Motion Control Computing Unit] Dedicated to Unitree motion control programs and **not open to public access**. Developers may only conduct secondary development using the [Development Computing Unit]..
> - [Development Computing Unit PC2] Its IP addresses vary by network port: 192.168.123.162 and 192.168.124.162. Default username: unitree; Default password: `Unitree0408` (try `Unitree#24226` for early firmware versions).
> -  [Development Computing Unit PC3 and PC4] Optional accessories. Their IP addresses differ by network port: PC3 uses 192.168.123.163 and 192.168.124.163; PC4 uses 192.168.123.164 and 192.168.124.164. The default username for both is unitree, with the default password `123`.
> - CPU modules may be upgraded to newer versions with equivalent or higher performance upon delivery.
---

# Robot Specifications
Please refer to: [H2 Specifications](https://www.unitree.com/H2)

!!! note
The product is continuously updated and optimized. Specifications are subject to minor changes. Please refer to the actual delivered product.
!!!
