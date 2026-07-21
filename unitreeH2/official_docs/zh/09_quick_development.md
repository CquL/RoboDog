本文介绍如何利用Unitree SDK2快速上手H2的开发。

注：本页面截图中例程的名称和数量会随着Unitree SDK2的更新而改变。请以最新版为准。

## 环境依赖

!!! note Note
您也可以使用板载用户开发单元（Ubuntu 22.04）。默认用户名：unitree。默认密码：123。
!!!

推荐使用以下配置构建H2的开发环境。

| **规格** | **参数** |
| -- | -- |
| 操作系统 | Ubuntu 20.04/22.04 [1]|
| ROS2 | Foxy/Humble [2] |
| 硬件形式 | 实机安装 [3] |

!!! note 注释
[1] 目前，Unitre SDK2只能在Linux环境中运行。推荐使用Ubuntu。**不支持Windows和MacOS**。
[2] Unitree SDK2不依赖ROS2。此处的依赖为Unitree ROS2的依赖。
[3] 若您对VMWare和WSL的网络穿透与配置不熟悉，建议不要使用这类平台。
!!!

!!! tip 提示
为了保证您的开发体验流畅顺利，除了以上硬件依赖外，我们还推荐您至少具备以下基础知识。
- Linux终端的基础命令和工具。
  - H2需要在Linux上进行开发，这意味着大部分时间您都需要和终端进行互动。一些基础的命令如cd、cat及一些基础工具如apt、git、ifconfig等能够帮助您使用Linux系统的基础功能。
- C++或Python编程基础。
  - H2支持使用这两种语言来进行开发。建议您具备至少其中一种语言的代码理解与编写能力，帮助您更快速地上手开发。
- ROS2及其相关机器人基础知识
  - 虽然Unitree SDK2与ROS2之间没有依赖关系，但还是建议您了解一下ROS2。在这个过程中，您不仅能学习ROS2的相关使用方法、大大扩张您的经验池，也能让您在实践中了解到与机器人相关的拓展基础知识，方便您后续的开发更加顺利。
!!!

## 获取SDK

[Unitree SDK2获取（Github）](https://github.com/unitreerobotics/unitree_sdk2)

除了SDK2外，还可以在此获取[Python](https://github.com/unitreerobotics/unitree_sdk2_python)和[ROS2](https://github.com/unitreerobotics/unitree_ros2)的SDK。您可按照SDK根目录下ReadMe中的教程来安装这些SDK。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     

您可以使用git来克隆我们在Github上开源的SDK。比如：

``` bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
```

![](https://doc-cdn.unitree.com/static/2025/12/22/fefa6682981443f286877436d0b57d7e_922x149.png)

您也可以使用这两种方法获取Unitree Python SDK2和Unitree ROS2。

## 安装SDK

进入Unitree SDK2文件夹。编译SDK中的例程。

``` bash
cd unitree_sdk2
mkdir build
cd build
cmake ..
make
```


!!! note **这些步骤做了什么？**
1. 使用cd命令进入了unitree_sdk2目录
2. 使用mkdir新建了一个文件夹build（构建产物，此文件夹的名称和位置固定的，程序的产物都会放在此处）
3. 使用cd命令进入了刚刚创建的build文件夹。
4. 使用cmake（..代表上级文件夹，即为项目的根目录）读取CMakeLists.txt中关于编译的配置，并递归读取项目每个文件夹中的CMakeLists配置文件，分析项目结构、依赖关系、编译选项等。
5. 按照配置文件编译程序，将.cpp程序文件编译为可执行的产物。
!!!

![](https://doc-cdn.unitree.com/static/2025/12/22/04217768afe548f5a9d1b114eadc6842_1288x1074.png)

成功后，你可以在`unitree_sdk2/build/bin`下看到SDK中包含的所有产物。

![](https://doc-cdn.unitree.com/static/2026/1/8/1cb42a87e77a4ed0a8569a54a419fbf8_862x843.png)

对了，如果您想在任何位置的项目中都能使用这一版本的Unitree SDK2，可以选择在`make`之后使用`sudo make install`将**这一版本**的Unitree API安装到您的系统中（非必选）。

!!! note 如果我将这一版本的Unitree API安装到系统中了，我该如何更新呢？
克隆新版本的Unitree SDK2之后，再执行sudo make install即可。
特别需要注意，当你执行过sudo make install将Unitree API安装到系统后，cmake将**优先使用你安装的这个版本来进行编译**，而不是你下载的新版本。需要特别注意。
!!!

## 连接H2

(等待更新)

## 设置网络

进入Ubuntu的设置界面，在左侧选择选择“网络”板块，右侧进入有线网络配置。

![](https://doc-cdn.unitree.com/static/2025/12/22/d5931429adfe4265982efd1944221aa6_977x818.png)

进入“IPV4“选项，选择“手动”。按图中的设置配置网络。最后点击右上角的“应用”。

![](https://doc-cdn.unitree.com/static/2025/12/22/41b76cf8487347a3bab7dedcb0064624_977x822.png)

!!! note 测试与H2之间的连接

使用工具ping可以测试您的设备与局域网中某个设备的连接情况。要检查与H2之间的连接情况，可以尝试ping它的主板IP地址：`192.168.123.161`。等待程序尝试5～6次保证连接无误的情况下，按下Control(CTRL) + C(Cancel)来终止ping程序，得到一个总结。

``` bash
ping 192.168.123.161
```

![](https://doc-cdn.unitree.com/static/2025/12/22/ce60d2163c784a0fb37e4c7ca459c92e_766x240.png)

!!!

## 例程运行

我们选择一个底层例程运行。首先，我们使用`ifconfig`工具查看当前的网卡。

![](https://doc-cdn.unitree.com/static/2025/12/22/f7c052e32eaa46bda59d167941d6285f_709x489.png)

!!! note 进入调试模式
要使用底层例程，首先需要将H2进入阻尼模式并吊起。随后按下遥控器上的L2+R2进入调试模式。此时按下L2+A，会进入诊断动作，即视为进入调试模式成功。
!!!

使用命令运行底层例程。运行前注意安全。

``` bash
./h2_ankle_swing_example 网卡名称
```

可以观察到，H2的腿部与手腕开始运动了。