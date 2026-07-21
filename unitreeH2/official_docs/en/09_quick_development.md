This article introduces how to quickly get started with H2 development using the Unitree SDK2.

**Note:** The names and number of examples in the screenshots on this page will change as Unitree SDK2 is updated. Please refer to the latest version.

## Environment Dependencies

!!! note Note
You can also use the onboard user development unit (Ubuntu 22.04). Default username: `unitree`. Default password: `123`.
!!!

It is recommended to use the following configuration to set up the H2 development environment.

| **Specification** | **Parameters** |
| -- | -- |
| Operating System | Ubuntu 20.04/22.04 [1] |
| ROS2 | Foxy/Humble [2] |
| Hardware Form | Installation on physical robot [3] |

!!! note
[1] Currently, Unitree SDK2 can only run on Linux systems. Ubuntu is recommended. **Windows and macOS are not supported**.
[2] Unitree SDK2 does not depend on ROS2. The dependencies listed here are for Unitree ROS2.
[3] If you are not familiar with network penetration and configuration of VMWare and WSL, we recommend not using such platforms.
!!!

!!! note
To ensure a smooth development experience, in addition to the hardware dependencies above, we also recommend that you have at least the following basic knowledge.
- Basic Linux terminal commands and tools.
  - H2 requires development on Linux, meaning you will interact with the terminal most of the time. Basic commands like `cd`, `cat`, and basic tools like `apt`, `git`, `ifconfig`, etc., will help you use the basic functions of the Linux system.
- Basic programming knowledge in C++ or Python.
  - H2 supports development using these two languages. We recommend that you have the ability to understand and write code in at least one of these languages, helping you get started more quickly.
- Basic knowledge of ROS2 and related robotics concepts.
  - Although Unitree SDK2 has no dependency on ROS2, we still recommend that you learn about ROS2. In the process, you will not only learn how to use ROS2 and greatly expand your experience, but also gain additional foundational robotics knowledge to facilitate your subsequent development.
!!!

## Obtain the SDK

[Unitree SDK2 (GitHub)](https://github.com/unitreerobotics/unitree_sdk2)

In addition to SDK2, you can also obtain the [Python](https://github.com/unitreerobotics/unitree_sdk2_python) and [ROS2](https://github.com/unitreerobotics/unitree_ros2) SDKs here. You can follow the tutorials in the ReadMe file in the root directory of the SDK to install these SDKs.

You can use `git` to clone our open-source SDK on GitHub. For example:

``` bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
```

![](https://doc-cdn.unitree.com/static/2025/12/22/fefa6682981443f286877436d0b57d7e_922x149.png)

You can also use these two methods to obtain Unitree Python SDK2 and Unitree ROS2.

## Install the SDK

Enter the Unitree SDK2 folder. Build the examples in the SDK.

``` bash
cd unitree_sdk2
mkdir build
cd build
cmake ..
make
```

!!! note **What do these steps do?**
1. Use `cd` to enter the `unitree_sdk2` directory.
2. Use `mkdir` to create a new folder named `build` (build artifacts; the name and location of this folder are fixed, and all program artifacts will be placed here).
3. Use `cd` to enter the newly created `build` folder.
4. Use `cmake` (`..` refers to the parent directory, i.e., the project root) to read the compilation configuration from `CMakeLists.txt` and recursively read the `CMakeLists.txt` configuration files in each folder of the project, analyzing the project structure, dependencies, compilation options, etc.
5. Compile the program according to the configuration file, compiling `.cpp` program files into executable artifacts.
!!!

![](https://doc-cdn.unitree.com/static/2025/12/22/04217768afe548f5a9d1b114eadc6842_1288x1074.png)

After success, you can see all artifacts included in the SDK under `unitree_sdk2/build/bin`.

![](https://doc-cdn.unitree.com/static/2026/1/8/1cb42a87e77a4ed0a8569a54a419fbf8_862x843.png)

By the way, if you want to use this version of Unitree SDK2 in any project anywhere, you can choose to run `sudo make install` after `make` to install **this version** of the Unitree API into your system (optional).

!!! note If I install this version of the Unitree API into my system, how do I update it?
After cloning the new version of Unitree SDK2, just run `sudo make install` again.
Be especially careful: after you have run `sudo make install` to install the Unitree API into the system, `cmake` will **prefer the version you installed** for compilation, rather than the new version you downloaded. Pay special attention to this.
!!!

## Connect to H2

(Waiting for update)

## Set up the Network

Enter the Ubuntu settings interface, select the "Network" section on the left, and enter the wired network configuration on the right.

![](https://doc-cdn.unitree.com/static/2025/12/22/d5931429adfe4265982efd1944221aa6_977x818.png)

Enter the "IPv4" option and select "Manual". Configure the network as shown in the figure. Finally, click "Apply" in the top right corner.

![](https://doc-cdn.unitree.com/static/2025/12/22/41b76cf8487347a3bab7dedcb0064624_977x822.png)

!!! note Test the connection with H2

Use the `ping` tool to test the connection between your device and another device on the local network. To check the connection with H2, try pinging its motherboard IP address: `192.168.123.161`. Let the program try 5–6 times to ensure the connection is correct, then press Control (CTRL) + C (Cancel) to terminate the `ping` program and get a summary.

``` bash
ping 192.168.123.161
```

![](https://doc-cdn.unitree.com/static/2025/12/22/ce60d2163c784a0fb37e4c7ca459c92e_766x240.png)

!!!

## Run Examples

We choose a low‑level example to run. First, use the `ifconfig` tool to view the current network interface.

![](https://doc-cdn.unitree.com/static/2025/12/22/f7c052e32eaa46bda59d167941d6285f_709x489.png)

!!! note Enter debug mode
To run low‑level examples, you first need to put the H2 into damping mode and suspend it. Then press L2+R2 on the remote controller to enter debug mode. At this point, pressing L2+A will trigger a diagnostic action, which means you have successfully entered debug mode.
!!!

Run the low‑level example with the command. Pay attention to safety before running.

``` bash
./h2_ankle_swing_example network_interface_name
```

You can observe that the H2's legs and wrists start moving.