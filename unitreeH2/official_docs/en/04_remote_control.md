<style>
table th:first-of-type {
    width: 50%;
}
table th:nth-of-type(2) {
    width: 50%;
}
</style>


# Concept description

<table>
<tr>
<th bgcolor="#D3D3D3"  style="width: 16%;">Concept</th>
<th bgcolor="#D3D3D3">Description</th>
</tr>
  <tr>
    <td>Zero Torque Mode</td>
    <td>All motors of the robot stop active motion, and there is no damping feeling when swinging.</td>
  </tr>
<tr>
<td>Damping Mode</td>
<td>All motors of the robot stop active motion, and there is a clear damping feeling when swinging, which can enter the ready mode.</td>
</tr>
  <tr>
<td>Ready Mode</td>
<td>The robot will slowly swing out the preparatory posture before the motion mode within 5 seconds.</td>
  </tr>

<tr>
<td>Motion Mode</td>
<td>A mode in which the robot can be controlled to move by a remote control.</td>
  </tr>
  <tr>
<td>Continuous Walking Mode</td>
<td>The robot is always in a stepping state.</td>
  </tr>
  <tr>
<td>Standing Mode</td>
<td>In this mode, when the joystick instruction is zero, the robot stops stepping and enters the standing state; when the joystick instruction is not zero, or the robot is disturbed and difficult to maintain balance, the robot will start to take steps.</td>
  </tr>
  <tr>
<td>Debug Mode</td>
<td>For low-level development:When using the SDK for development or debugging, always verify that G1 is in debug mode (damping or zero-torque). Enter debug mode by pressing L2 + R2 on the remote); this halts the motion-control program and prevents potential command conflicts.
To confirm debug mode is active, press L2 + A.
In an emergency during debugging, press L2 + B on the remote to switch the device to damping mode.</td>
</table>


# Operating Instructions
![](https://doc-cdn.unitree.com/static/2026/5/26/a479e509e435463ebb67aa022e9c8e4e_1127x601.png)







