The Low-level communication implements data interaction between the user-side PC (PC2/external PC) and the robot.
The Low-level communication adopts the DDS protocol （**Relevant knowledge about DDS can be checked for reference**[《DDS Services Interface》](https://support.unitree.com/home/en/G1_developer/dds_services_interface)).

- Subscribe to topics `rt/lowstate`(Type: `unitree_hg::msg::dds_::LowState_`) Get the current status of H2.

- Post a topic `rt/lowcmd`(Type: `unitree_hg::msg::dds_::LowCmd_`) Control equipment such as full-body joint motors (excluding dexterous hands) and batteries.

<p align="center">
  <img src="https://doc-cdn.unitree.com/static/2026/3/18/78960ff2e59e49208b795fcf3f55cbf0_1244x843.png" width="80%"/>
</p>

# Interface Description
Subscribe to or publish topics using the method described in Document [DDS Services Interface](https://support.unitree.com/home/en/G1_developer/dds_services_interface) . Topic information is stored in structures defined by IDL, with commonly used structures listed below:


| Structure name          | Description       |
| ------------------- | --------------- |
| `IMUState_`         | H2 IMU Status     |
| `LowCmd_`           | H2 Low-level control     |
| `LowState_`         | H2 Low-level status     |
| `MotorCmd_`         | H2 Motor Control     |
| `MotorState_`       | H2 Motor Status     |

# Introduction 

## IMU Status
* `unitree_hg::msg::dds_::IMUState_`

  ```c++
  struct IMUState_ {
    float quaternion[4];                                   // Quaternion QwQxQyQz
    float gyroscope[3];                                    // Gyroscope (angular velocity) omega_xyz
    float accelerometer[3];                                // Acceleration acc_xyz
    float rpy[3];                                          // Euler angles
    short temperature;                                     // IMU Temperature
  };
  ```
## Low-level control 
* `unitree_hg::msg::dds_::LowCmd_`

  ```c++
  struct LowCmd_ {
    octet mode_pr;                                         // Parallel mechanism（Ankles and waist）Control Mode(Default 0) 0:PR, 1:AB
    octet mode_machine;                                    // H2 Model
    unitree_hg::msg::dds_::MotorCmd_ motor_cmd[35];        // All motor control commands for the body
    unsigned long reserve[4];                              // Reserve
    unsigned long crc;                                     // Verification
  };
  ```
## Low-level status
* `unitree_hg::msg::dds_::LowState_`

  ```c++
  struct LowState_ {
    unsigned long version[2];                              // Version   
    octet mode_pr;                                         // Parallel mechanism（Ankles and waist）Control Mode(Default 0) 0:PR, 1:AB
    octet mode_machine;                                    // H2 Model
    unsigned long tick;                                    // Timer increments every 1ms
    unitree_hg::msg::dds_::IMUState_ imu_state;            // IMU Status
    unitree_hg::msg::dds_::MotorState_ motor_state[35];    // Status of all body motors
    octet wireless_remote[40];                             // Original raw data of Unitree physical remote controller
    unsigned long reserve[4];                              // Reserve
    unsigned long crc;                                     // Checksum
  };
  ```
## Motor Control
* `unitree_hg::msg::dds_::MotorCmd_`

  ```c++
  struct MotorCmd_ {
    octet mode;                                            // Motor control mode 0:Disable, 1:Enable
    float q;                                               // Target position of joints
    float dq;                                              // Target joint velocity
    float tau;                                             // Joint feedforward torque
    float kp;                                              // Joint stiffness coefficient
    float kd;                                              // Joint damping coefficient
    unsigned long reserve[3];                              // Retain
  };
  ```
## Motor Status 
* `unitree_hg::msg::dds_::MotorState_`

  ```c++
  struct MotorState_ {
    octet mode;                                            // Current motor mode
    float q;                                               // Joint feedback position (rad)
    float dq;                                              // Joint feedback speed (rad/s)
    float ddq;                                             // Joint feedback acceleration (rad/s^2)
    float tau_est;                                         // Joint feedback torque   
    float q_raw;                                           // Reserve
    float dq_raw;                                          // Reserve
    float ddq_raw;                                         // Reserve
    short temperature[2];                                  // Motor temperature (surface and winding temperature)
    unsigned long sensor[2];                               // Sensor data
    float vol;                                             // Terminal voltage of the motor
    unsigned long motorstate;                              // Motor status
    unsigned long reserve[4];                              // Reserve
  };
  ```








