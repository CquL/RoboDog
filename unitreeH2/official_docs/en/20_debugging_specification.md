
<style>
  img[alt="1"]{
  height:600px;
    }
  </style>

You can use your own PC (with **Ubuntu system**) to connect to the H2’s shoulder Ethernet port via an Ethernet cable to establish communication between the User PC and the H2. Your program can be deployed either on your own PC or on the onboard secondary development board of the H2, controlling the H2’s motion via the DDS network.

**Step 1**: Secure the H2 on the protective stand, ensuring the four swivel casters at the bottom of the stand are locked.

![](https://doc-cdn.unitree.com/static/2026/3/18/9e209aa8def9451dbc172b9f3effc682_821x1269.png)

**Step 2**: Connect the User PC to the device’s RJ45 port via an Ethernet cable. For details, refer to the electrical interface description in the [About H2](https://support.unitree.com/home/zh/H2_developer/About_H2) section. Then you can proceed with communication debugging.

!!! note
In the current system version, as soon as the H2 is powered on, the built-in motion control program starts automatically, even if you are not operating the remote controller. This program periodically sends zero‑velocity commands. However, if you perform low‑level development with the SDK in this state, command conflicts may occur, causing the H2 to jitter.

Therefore, when using the SDK for development and debugging, make sure the H2 has entered debug mode to stop the motion control program from sending commands, thus avoiding potential command conflicts. You can press **L2 + A** to confirm whether debug mode has been entered.

If the behavior after pressing L2 + A does not match the tutorial video, press and hold **L2 + R2** multiple times to ensure the robot enters debug mode.
!!!