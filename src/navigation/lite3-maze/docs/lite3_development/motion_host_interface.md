# Motion Host Communication Interface

Source: [Motion Host Communication Interface](https://alidocs.dingtalk.com/i/p/OlnXRxOLAljEbGLp/docs/kDnRL6jAJM3d9LPdSG4dpq6AWyMoPYe1)

Upadte Date: 2026-03-09

Version: V1.1.0-0


---


## 1 UDP Commands


Communication with the motion host adopts UDP protocol, and the raw data is stored by little-endian.

When developers send messages to the motion host, the target port is 43893 and please refer to [Lite3_MotionSDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK/tree/main) section 4.1 to obtain the target IP.

By default, the motion host only reports data to the perception host. If developers need to receive data reported by the motion host, please refer to [Lite3_MotionSDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK/tree/main) section 4 and section 5 to connect to the motion host via SSH, modify the data receiving address to the static IP of the development host.


### 1.1 Protocol Format

This section introduces the raw data format transmitted by UDP protocol. Based on whether complex data is carried, UDP commands can be divided into two categories: simple commands and complex commands.


#### 1.1.1 Simple Commands

The format of a simple command is `[xxxx yyyy zzzz]`, with each letter representing a byte. 

The raw data of a simple command is stored in CommandHead structure.


```c++
struct CommandHead{
    uint32_t code;
    uint32_t paramters_size;
    uint32_t type;
};
```

`xxxx` represents the value of `CommandHead.code`, indicating the command code. 

`yyyy` stands for the value of `CommandHead.paramters_size`, indicating the value of the command code. When the command code does not have a valid value, yyyy = 0. 

`zzzz` refers to the value of `CommandHead.type`, used to differentiate various command type. The type value for simple command is 0, i.e., zzzz = 0. 


```c++
// Example Program
struct CommandHead command_head = {0};
command_head.code = 1;           // Command Code
command_head.paramters_size = 0;// Command Value
command_head.type = 0;           // Command Type
sendto(sfd,&command_head,sizeof(command_head), Target Address, Address Length)
```


#### 1.1.2 Complex Commands

Complex commands carry specific data content in the format of `[xxxx yyyy zzzz bbbbbbbb……]`, with each letter representing a byte.

The raw data for complex command is stored in `Command` structure.


```c++
struct CommandHead{
    uint32_t code;
    uint32_t paramters_size;
    uint32_t type;
};

const uint32_t kDataSize = 256;

struct Command{
    CommandHead head;
    uint32_t data[kDataSize];
};
```

`xxxx` represents the value of `Command.head.code`, indicating the command code. 

`yyyy` stands for the value of `Command.head.paramters_size`, indicating the length of `bbbbbbbb…….`

`zzzz` refers to the value of `Command.head.type`, used to differentiate various command type. The type value for complex command is 1, i.e., zzzz = 1. 

`bbbbbbbb……` defines the value of `Command.data`, representing the data carried by the command. 

Example of usage:


```c++
// Example Program
int32_t to_be_send_data[64];
struct Command command = {0};
command.head.code = 52; // Command Code
command.head.paramters_size = sizeof(to_be_send_data);// Data Length
command.type = 1;         // Command Type
memcpy(&command.data,&to_be_send_data,sizeof(to_be_send_data));
sendto(sfd,&command,sizeof(command.head)+command.head.paramters_size, Target Address, Address Length);
```


#### 1.1.3 Receiving Complex Commands

Taking joint velocity as an example, the usage is as follows:


```c++
struct RobotJointVel{
    double joint_vel[12];
}; // Take joint velocity as an example
struct CommandMessage{
    CommandHead command;
    uint8_t data_buffer[1024];
};
CommandMessage cm;
size_t recv_size = 0;
recv_size = recvfrom(server_fd,&cm,sizeof(cm),0,(sockaddr*)&client_addr,&sockaddr_in_size);
if(cm.command.type == 1){ // Check if it is a complex command
    if(recv_size == cm.command.paramters_size + sizeof(cm.command)){
        uint8_t* temp_values = new uint8_t[cm.command.paramters_size];
        memcpy(temp_values,cm.data_buffer,cm.command.paramters_size);
        if(cm.command.code == 0x0903){ // Check the command code and parse the data
            RobotJointVel *joint_vel = (RobotJointVel *)temp_values;
        }
    }
}
```


### 1.2 Control Command Set

Developers control the robot and achieve corresponding functions by directly sending commands to the motion host through UDP protocol. However, sending messages does not change the original control logic.

Jueying Lite3 can receive messages from multiple controllers at the same time. The command of "0x2*******" can be sent by multiple clients, but the downstream data, originating from the wide camera, can only be sent to the client that initially connected to the robot.


#### 1.2.1 Heartbeat

The heartbeat command is used to verify the stability of the connection and should be sent at a minimal frequency of 2Hz.


<table><tr><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Value</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td></tr>
<tr><td><p>0x21040001</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td></tr>
</table>

[Notes] The minus sign `-` means the command value is meaningless, i.e., the command value will not act on the command.


#### 1.2.2 Commands for Switching Between Basic States

The switch between robot's basic states is achieved by giving corresponding commands, as shown in the following figure (the corresponding code indicated in parentheses is status value).


> Interactive state diagram: view the original for its layout and transitions. Its source data is preserved in originals/motion_host_interface.json. [View original diagram](https://alidocs.dingtalk.com/i/p/OlnXRxOLAljEbGLp/docs/kDnRL6jAJM3d9LPdSG4dpq6AWyMoPYe1)

Some of the commands are shown in the table below:


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Value</strong></p>
</td><td><p><strong>Type</strong></p>
</td><td><p><strong>Function</strong></p>
</td></tr>
<tr><td><p>Stand/Sit</p>
</td><td><p>0x21010202</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch between sitting state and standing state</p>
</td></tr>
<tr><td><p>STOP</p>
</td><td><p>0x21020C0E</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Enables software-based emergency stop</p>
</td></tr>
<tr><td><p>Reset to Zero</p>
</td><td><p>0x21010C05</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Initialize the robot&#x27;s joints</p>
</td></tr>
<tr><td><p>Enter AI</p>
</td><td><p>0x21010528</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Enter AI State</p>
</td></tr>
<tr><td><p>Exit AI</p>
</td><td><p>0x2101052B</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Exit AI State</p>
</td></tr>
</table>


#### 1.2.3 Axis Command

Axis command is the command output from x-axis and y-axis of the two joysticks of the controller, with a value range of: `[-32767,32767]`. If the motion host has not received any axis command for more than 250 ms, the command will be considered as invalid by the system and the robot will stop moving.


##### 1.2.3.1 Axis Command in Pose Mode

In pose mode, the client can change the robot's postures by sending axis command to the motion host. If the motion host doesn't receive any axis command for more than one second, the system will consider the command invalid and the robot will return to its normal standing position. If the axis command value is within the dead zone range, it is considered as 0.


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Value</strong></p>
</td><td><p><strong>Dead Zone Range</strong></p>
</td><td><p><strong>Function</strong></p>
</td></tr>
<tr><td><p>Adjust Roll Angle</p>
</td><td><p>0x21010131</p>
</td><td><p>0</p>
</td><td><p>[-12553,12553]</p>
</td><td><p>Positive value rolls to the right</p>
</td></tr>
<tr><td><p>Adjust Pitch Angle</p>
</td><td><p>0x21010130</p>
</td><td><p>0</p>
</td><td><p>[-6553,6553]</p>
</td><td><p>Positive value lowers the head</p>
</td></tr>
<tr><td><p>Adjust Body Height</p>
</td><td><p>0x21010102</p>
</td><td><p>0</p>
</td><td><p>[-20000,20000]</p>
</td><td><p>Positive value raises the body</p>
</td></tr>
<tr><td><p>Adjust Yaw Angle</p>
</td><td><p>0x21010135</p>
</td><td><p>0</p>
</td><td><p>[-9553,9553]</p>
</td><td><p>Positive value rotates to the right</p>
</td></tr>
</table>


##### 1.2.3.2 Axis Command in Move Mode and AI State

In move mode and AI state, the client can change robot's moving direction and speed by sending axis commands to the motion host. The issue frequency of axis command should not be lower than 20Hz. When the axis command value is within the dead zone, it is considered as 0, and the robot stops moving. The positive and negative values of the axis command determine the direction of the speed.


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Value</strong></p>
</td><td><p><strong>Dead Zone Range</strong></p>
</td><td><p><strong>Function</strong></p>
</td></tr>
<tr><td><p>Translation</p>
<p>(Left/Right) </p>
</td><td><p>0x21010131</p>
</td><td><p>0</p>
</td><td><p>[-12553,12553]</p>
</td><td><p>Specifies the expected linear velocity of the robot along the y-axis, with positive value indicating moving to the right. </p>
</td></tr>
<tr><td><p>Translation</p>
<p>(Forward/Backward)</p>
</td><td><p>0x21010130</p>
</td><td><p>0</p>
</td><td><p>[-6553,6553]</p>
</td><td><p>Specifies the expected linear velocity of the robot along the x-axis, with positive value indicating moving forward. </p>
</td></tr>
<tr><td><p>Turning Left/Right</p>
</td><td><p>0x21010135</p>
</td><td><p>0</p>
</td><td><p>[-9553,9553]</p>
</td><td><p>Specifies the expected angular velocity of the robot, with positive value indicating turning to the right.</p>
</td></tr>
</table>


##### 1.2.3.3 Stop Command

In move mode, if the axis command is 0, the robot will stop moving.


#### 1.2.4 Commands for Switching Between Motion Modes


<table><tr><td><p><strong>Command Name</strong></p>
</td><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Value</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td></tr>
<tr><td><p>Pose Mode</p>
</td><td><p>0x21010D05</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td></tr>
<tr><td><p>Move Mode</p>
</td><td><p>0x21010D06</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td></tr>
</table>

[Notes] This command is invalid when the robot is in AI state.


#### 1.2.5 Commands for Switching Between Different Gaits

Robot's gait can be adjusted when it is not in AI state and is in move mode.


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Value</strong></p>
</td><td><p><strong>Type</strong></p>
</td><td><p><strong>Function</strong></p>
</td></tr>
<tr><td><p>Flat gait in Slow gear</p>
</td><td><p>0x21010300</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to a low-speed gait</p>
</td></tr>
<tr><td><p>Flat gait in Medium gear</p>
</td><td><p>0x21010307</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to a medium-speed gait</p>
</td></tr>
<tr><td><p>Flat gait in Fast gear</p>
</td><td><p>0x21010303</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to a high-speed gait</p>
</td></tr>
<tr><td><p>Flat gait in Crawl gear</p>
</td><td><p>0x21010406</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to a low-speed crawling gait, or switch from a low-speed crawling gait to a normal low-speed gait</p>
</td></tr>
<tr><td><p>RUG gait in Grip gear</p>
</td><td><p>0x21010402</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to a grip gait</p>
</td></tr>
<tr><td><p>RUG gait in General gear</p>
</td><td><p>0x21010401</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to a general gait</p>
</td></tr>
<tr><td><p>RUG gait in H-Step gear</p>
</td><td><p>0x21010407</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to a high-lift stepping gait.</p>
</td></tr>
</table>


#### 1.2.6 Action Commands

When the robot is not in AI state and is in standing or sitting state, action commands can be given to make it perform corresponding action commands.


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Value</strong></p>
</td><td><p><strong>Type</strong></p>
</td><td><p><strong>Conditions for executing actions</strong></p>
</td></tr>
<tr><td><p>Twist</p>
</td><td><p>0x21010204</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>In Torque-Control state (standing state)</p>
</td></tr>
<tr><td><p>Turn over</p>
</td><td><p>0x21010205</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>In sitting state</p>
</td></tr>
<tr><td><p>Moonwalk</p>
</td><td><p>0x2101030C</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>In Torque-Control state (standing state)</p>
</td></tr>
<tr><td><p>Backflip</p>
</td><td><p>0x21010502</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>In sitting state</p>
</td></tr>
<tr><td><p>Hello</p>
</td><td><p>0x21010507</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>In sitting state</p>
</td></tr>
<tr><td><p>Long Jump</p>
</td><td><p>0x2101050B</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>In sitting state</p>
</td></tr>
<tr><td><p>Twist Jump</p>
</td><td><p>0x2101020D</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>In Torque-Control state (standing state)</p>
</td></tr>
<tr><td rowspan="2"><p>Stop Action</p>
</td><td rowspan="2"><p>0x21010C0B</p>
</td><td><p>0</p>
</td><td rowspan="2"><p>-</p>
</td><td rowspan="2"><p>In performing continuous actions such as Twist and Moonwalk</p>
</td></tr>
<tr><td><p></p>
</td><td><p></p>
</td><td><p>1</p>
</td><td><p></p>
</td><td><p></p>
</td></tr>
</table>

[Notes] When sending the Stop Action command (0x21010C0B), command value 0 and 1 must be sent simultaneously to ensure that the robot can correctly execute the stop action command.


#### 1.2.7 Commands for Switching Between Control Modes

The control mode determines the source of speed commands that the robot responds to. In navigation mode, the robot responds to speed commands issued by the perception host, while in manual mode, the robot responds to speed commands issued by the controller.


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Value</strong></p>
</td><td><p><strong>Type</strong></p>
</td><td><p><strong>Function</strong></p>
</td></tr>
<tr><td><p>Navigation</p>
</td><td><p>0x21010C03</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from manual mode to navigation mode</p>
</td></tr>
<tr><td><p>Manual</p>
</td><td><p>0x21010C02</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from navigation mode to manual mode</p>
</td></tr>
</table>

[Notes] When the robot is in AI state, switching to navigation mode is not supported.


#### 1.2.8 Commands for Saving Data

In the event of a malfunction, data saving function allows the robot to save data from the previous 100 seconds, and then the robot will will have a soft emergency stop and quit the motion program.


<table><tr><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Value</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td></tr>
<tr><td><p>0x21010C01</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td></tr>
</table>


#### 1.2.9 Commands for Keep Stepping Mode

When keep stepping mode is enabled, the robot will keep stepping even if no axis commands are received when the robot is not in AI state.


<table><tr><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Value</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td></tr>
<tr><td><p>0x21010C06</p>
</td><td><p>-1=Enable, 2=Disable</p>
</td><td><p>0</p>
</td></tr>
</table>


#### 1.2.10 Voice Commands and Speaker Commands

The App recognizes user voice and sends voice commands with corresponding command values to control robot movement.


<table><tr><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Value</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td></tr>
<tr><td><p>0x21010C0A</p>
</td><td><p>See the table below</p>
</td><td><p>0</p>
</td></tr>
</table>

The command functions corresponding to the values of the voice commands are shown in the table below:


<table><tr><td><p><strong>Value</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>0</p>
</td><td><p>Stop voice command</p>
</td></tr>
<tr><td><p>1</p>
</td><td><p>Stand up</p>
</td></tr>
<tr><td><p>2</p>
</td><td><p>Sit down</p>
</td></tr>
<tr><td><p>3</p>
</td><td><p>Move forward</p>
</td></tr>
<tr><td><p>4</p>
</td><td><p>Move backward</p>
</td></tr>
<tr><td><p>5</p>
</td><td><p>Translate to the left</p>
</td></tr>
<tr><td><p>6</p>
</td><td><p>Translate to the right</p>
</td></tr>
<tr><td><p>7</p>
</td><td><p>Stop</p>
</td></tr>
<tr><td><p>8</p>
</td><td><p>Head down</p>
</td></tr>
<tr><td><p>9</p>
</td><td><p>Head up</p>
</td></tr>
<tr><td><p>11</p>
</td><td><p>Look left</p>
</td></tr>
<tr><td><p>12</p>
</td><td><p>Look right</p>
</td></tr>
<tr><td><p>13</p>
</td><td><p>Turn left by 90° </p>
</td></tr>
<tr><td><p>14</p>
</td><td><p>Turn right by 90°</p>
</td></tr>
<tr><td><p>15</p>
</td><td><p>Turn back by 180° </p>
</td></tr>
<tr><td><p>22</p>
</td><td><p>Greeting(perform the action Hello)</p>
</td></tr>
</table>

Speaker Command:


<table><tr><td><p><strong>Command Name</strong></p>
</td><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Value</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td></tr>
<tr><td><p>Speaker Command</p>
</td><td><p>0x2101030D</p>
</td><td><p>0= Turn off the speaker</p>
<p>1= Turn on the speaker</p>
<p>2= Check speaker status</p>
</td><td><p>0</p>
</td></tr>
</table>

If the speaker command value is set to 2, which means checking the speaker status, the motion host will report the speaker status:


<table><tr><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td><td><p><strong>Content</strong></p>
</td></tr>
<tr><td><p>0x11050f08</p>
</td><td><p>0</p>
</td><td><p>0=The speaker is off</p>
<p>1= The speaker is on</p>
</td></tr>
</table>

[Notes] After receiving a voice command, the robot will repeatedly execute the corresponding action in a loop until it receives the command (0x21010C0A, 0, 0), at which point the action will stop.


#### 1.2.11 Commands for Perception Settings

Developers can send the following commands to the motion host to enable or disable related functions.


<table><tr><td><p><strong>Command Name</strong></p>
</td><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Value</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td></tr>
<tr><td><p>Turn off all AI options</p>
</td><td><p>0x21012109</p>
</td><td><p>0x00</p>
</td><td><p>0</p>
</td></tr>
<tr><td><p>Enable auto stop function</p>
</td><td><p>0x21012109</p>
</td><td><p>0x20</p>
</td><td><p>0</p>
</td></tr>
<tr><td><p>Enable tracking function</p>
</td><td><p>0x21012109</p>
</td><td><p>0xC0</p>
</td><td><p>0</p>
</td></tr>
</table>

In addition, developers can send the following commands to the perception host, with the target IP and port as `192.168.1.103:43899`.


<table><tr><td><p><strong>Command Name</strong></p>
</td><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Value</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td></tr>
<tr><td><p>Enable obstacle avoidance function</p>
</td><td><p>0x21012109</p>
</td><td><p>0x40</p>
</td><td><p>0</p>
</td></tr>
</table>

If the IP of motion host is `192.168.1.120`, the address for streaming the video from robot's wide-angle camera is: `rtsp://192.168.1.120:8554/test`. If the IP of motion host is `192.168.2.1`, the address for streaming the video from robot's wide-angle camera is: `rtsp://192.168.2.1:8554/test`.

[Notes] When using the functions described in this section, ensure that the robot is not in AI state.


#### 1.2.12 Speed Commands

The speed commands are complex commands, and the data values each speed command carries are all double-precision floating-point type, which need to be sent to the robot when it is in navigation mode.


<table><tr><td><p><strong>Command Name</strong></p>
</td><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td><td><p><strong>Value Range</strong></p>
</td></tr>
<tr><td><p>Angular velocity (rad/s)</p>
</td><td><p>0x0141</p>
</td><td><p>1</p>
</td><td><p>[-1.5,1.5]</p>
</td></tr>
<tr><td><p>Linear velocity in robot x-axis (m/s)</p>
</td><td><p>0x0140</p>
</td><td><p>1</p>
</td><td><p>[-1.0,1.0]</p>
</td></tr>
<tr><td><p>Linear velocity in robot y-axis (m/s)</p>
</td><td><p>0x0145</p>
</td><td><p>1</p>
</td><td><p>[-0.5,0.5]</p>
</td></tr>
</table>


#### 1.2.13 Commands for Switching AI Gaits

Robot's AI gait can be adjusted when it is in AI state.


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Value</strong></p>
</td><td><p><strong>Type</strong></p>
</td><td><p><strong>Function</strong></p>
</td></tr>
<tr><td><p>Basic</p>
</td><td><p>0x2101052A</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to the AI basic gait.</p>
</td></tr>
<tr><td><p>Fast</p>
</td><td><p>0x2101052E</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to the AI fast gait.</p>
</td></tr>
<tr><td><p>Jump</p>
</td><td><p>0x21010529</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to the AI jump gait</p>
</td></tr>
<tr><td><p>Stand</p>
</td><td><p>0x2101052C</p>
</td><td><p>-</p>
</td><td><p>0</p>
</td><td><p>Switch the robot from the current gait to the AI stand gait</p>
</td></tr>
</table>

[Notes] AI Jump and AI Stand gaits are intended for posture adjustment only. Do not control the robot’s movement while it is in these gaits.


#### 1.2.14 AI Action Commands

Robot's AI gait can be adjusted when it is in AI state.


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Value</strong></p>
</td><td><p><strong>Type</strong></p>
</td><td><p><strong>Function</strong></p>
</td><td><p><strong>Conditions of Use</strong></p>
</td></tr>
<tr><td rowspan="4"><p>AI action</p>
</td><td rowspan="4"><p>0x2101030E</p>
</td><td><p>4</p>
</td><td><p>0</p>
</td><td><p>Performing handstand action</p>
</td><td><p>In AI Stand gait</p>
</td></tr>
<tr><td><p></p>
</td><td><p></p>
</td><td><p>64</p>
</td><td><p>0</p>
</td><td><p>Performing backflip action</p>
</td><td><p>In AI Jump gait</p>
</td></tr>
<tr><td><p></p>
</td><td><p></p>
</td><td><p>128</p>
</td><td><p>0</p>
</td><td><p>Performing leap action</p>
</td><td><p>In AI Jump gait</p>
</td></tr>
<tr><td><p></p>
</td><td><p></p>
</td><td><p>0</p>
</td><td><p>0</p>
</td><td><p>Stop performing the AI action</p>
</td><td><p>When performing jump action</p>
</td></tr>
</table>

[Notes] Handstand is a time-limited action with a maximum duration of 10 seconds per execution. After the performance ends, the robot will automatically return to the quadruped standing posture.


### 1.3 Command Set for Information Reception

Users can obtain the information reported by the motion host according to the following command format.


#### 1.3.1 Robot State


<table><tr><td><p><strong>Command Code</strong></p>
</td><td><p><strong>Command Type</strong></p>
</td><td><p><strong>Content</strong></p>
</td><td><p><strong>Frequency</strong></p>
</td></tr>
<tr><td><p>0x0901</p>
</td><td><p>1</p>
</td><td><p><code>RobotStateUpload</code> structure (details below) </p>
</td><td><p>50 Hz</p>
</td></tr>
</table>


```c++
struct RobotStateUpload{
  int robot_basic_state;		
  int robot_gait_state;
  int robot_policy_state;
  double rpy[3];
  double rpy_vel[3];
  double xyz_acc[3];
  double pos_world[3];
  double vel_world[3];
  double vel_body[3];
  unsigned touch_down_and_stair_trot;
  bool is_charging;
  unsigned error_state;
  int robot_motion_state;
  double battery_level;
  int task_state;
  bool is_robot_need_move;
  bool zero_position_flag;
  bool is_after_first_start;
  bool is_voice_ctrl_enable;
  double ultrasound[2];
};
```


<table><tr><td><p><strong>Field</strong></p>
</td><td><p><strong>Type</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>robot_basic_state</p>
</td><td><p>int</p>
</td><td><p>Robot&#x27;s current basic state</p>
</td></tr>
<tr><td><p>robot_gait_state</p>
</td><td><p>int</p>
</td><td><p>Robot&#x27;s current gait</p>
</td></tr>
<tr><td><p>robot_policy_state</p>
</td><td><p>int</p>
</td><td><p>Robot&#x27;s current AI gait</p>
</td></tr>
<tr><td><p>rpy[3]</p>
</td><td><p>double</p>
</td><td><p>IMU angle</p>
</td></tr>
<tr><td><p>rpy_vel[3]</p>
</td><td><p>double</p>
</td><td><p>IMU angular velocity</p>
</td></tr>
<tr><td><p>xyz_acc[3]</p>
</td><td><p>double</p>
</td><td><p>IMU acceleration</p>
</td></tr>
<tr><td><p>pos_world[3]</p>
</td><td><p>double</p>
</td><td><p>Robot&#x27;s posture in world coordinate system</p>
</td></tr>
<tr><td><p>vel_world[3]</p>
</td><td><p>double</p>
</td><td><p>Robot&#x27;s velocity in world coordinate system</p>
</td></tr>
<tr><td><p>vel_body[3]</p>
</td><td><p>double</p>
</td><td><p>Robot velocity in body coordinate system</p>
</td></tr>
<tr><td><p>touch_down_and_stair_trot</p>
</td><td><p>unsigned</p>
</td><td><p>Invalid data, used as a placeholder only </p>
</td></tr>
<tr><td><p>is_charging</p>
</td><td><p>bool</p>
</td><td><p>Invalid data, used as a placeholder only</p>
</td></tr>
<tr><td><p>error_state</p>
</td><td><p>unsigned</p>
</td><td><p>Invalid data, used as a placeholder only</p>
</td></tr>
<tr><td><p>robot_motion_state</p>
</td><td><p>int</p>
</td><td><p>Robot&#x27;s motion state</p>
</td></tr>
<tr><td><p>battery_level</p>
</td><td><p>double</p>
</td><td><p>Battery power percentage in decimal form</p>
</td></tr>
<tr><td><p>task_state</p>
</td><td><p>int</p>
</td><td><p>Invalid data, used as a placeholder only</p>
</td></tr>
<tr><td><p>is_robot_need_move</p>
</td><td><p>bool</p>
</td><td><p>Balance state when the robot is subjected to external force</p>
</td></tr>
<tr><td><p>zero_position_flag</p>
</td><td><p>bool</p>
</td><td><p>Flag indicating the state of Reset-to-Zero</p>
</td></tr>
<tr><td><p>is_after_first_start</p>
</td><td><p>bool</p>
</td><td><p>First-time start flag</p>
</td></tr>
<tr><td><p>is_voice_ctrl_enable</p>
</td><td><p>bool</p>
</td><td><p>Voice control function flag</p>
</td></tr>
<tr><td><p>ultrasound[2]</p>
</td><td><p>double</p>
</td><td><p>Ultrasonic data</p>
</td></tr>
</table>

The values in the field `robot_basic_state` correspond to the following robot basic states:


<table><tr><td><p><strong>Variable Value</strong></p>
</td><td><p><strong>Robot&#x27;s basic state</strong></p>
</td></tr>
<tr><td><p>1</p>
</td><td><p>Sitting State</p>
</td></tr>
<tr><td><p>4</p>
</td><td><p>Prepare State</p>
</td></tr>
<tr><td><p>5</p>
</td><td><p>Sit-to-Stand State</p>
</td></tr>
<tr><td><p>6</p>
</td><td><p>Torque-control State</p>
</td></tr>
<tr><td><p>7</p>
</td><td><p>Stand-to-Sit State</p>
</td></tr>
<tr><td><p>8</p>
</td><td><p>Lose Control Protection State</p>
</td></tr>
<tr><td><p>9</p>
</td><td><p>Posture Adjustment State</p>
</td></tr>
<tr><td><p>11</p>
</td><td><p>Flipping over State</p>
</td></tr>
<tr><td><p>16</p>
</td><td><p>AI State</p>
</td></tr>
<tr><td><p>17</p>
</td><td><p>Reset-to-Zero State</p>
</td></tr>
<tr><td><p>18</p>
</td><td><p>Backflip State(be performing Backflip action)</p>
</td></tr>
<tr><td><p>20</p>
</td><td><p>Hello State(be performing Hello action)</p>
</td></tr>
</table>

[Notes] The transition relations for basic states can be referred to in 1.2.2.

The values of the field `robot_gait_state` correspond to the following robot gaits:


<table><tr><td><p><strong>Variable Value</strong></p>
</td><td><p><strong>Gait</strong></p>
</td></tr>
<tr><td><p>0</p>
</td><td><p>Flat gait in Slow gear</p>
</td></tr>
<tr><td><p>2</p>
</td><td><p>RUG gait in General gear</p>
</td></tr>
<tr><td><p>4</p>
</td><td><p>Flat gait in Medium gear</p>
</td></tr>
<tr><td><p>5</p>
</td><td><p>Flat gait in Fast gear</p>
</td></tr>
<tr><td><p>6</p>
</td><td><p>RUG gait in Grip gear</p>
</td></tr>
<tr><td><p>13</p>
</td><td><p>RUG gait in H-Step gear</p>
</td></tr>
<tr><td><p>12</p>
</td><td><p>Moonwalk</p>
</td></tr>
</table>

The values of the field `robot_policy_stat` correspond to the following robot AI gaits:


<table><tr><td><p><strong>Variable Value</strong></p>
</td><td><p><strong>AI Gait</strong></p>
</td></tr>
<tr><td><p>0</p>
</td><td><p>AI Basic</p>
</td></tr>
<tr><td><p>16</p>
</td><td><p>AI Jump</p>
</td></tr>
<tr><td><p>18</p>
</td><td><p>AI Stand</p>
</td></tr>
<tr><td><p>20</p>
</td><td><p>AI Fast</p>
</td></tr>
</table>

The field `rpy[3]` may contain the following information:


```c++
double rpy[3] = {roll,pitch,yaw};
```


<table><tr><td><p><strong>Element</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>roll</p>
</td><td><p>Roll angle (°) of IMU in world coordinate system</p>
</td></tr>
<tr><td><p>pitch</p>
</td><td><p>Pitch angle (°) of IMU in world coordinate system</p>
</td></tr>
<tr><td><p>yaw</p>
</td><td><p>Yaw angle (°) of IMU in world coordinate system</p>
</td></tr>
</table>

The field `rpy_vel[3]` may contain the following information:


```c++
double rpy_vel[3] = {roll_vel,pitch_vel,yaw_vel};
```


<table><tr><td><p><strong>Element</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>roll_vel</p>
</td><td><p>Roll angular velocity (rad/s) of IMU in world coordinate system</p>
</td></tr>
<tr><td><p>pitch_vel</p>
</td><td><p>Pitch angular velocity (rad/s) of IMU in world coordinate system</p>
</td></tr>
<tr><td><p>yaw_vel</p>
</td><td><p>Yaw angular velocity (rad/s) of IMU in world coordinate system</p>
</td></tr>
</table>

The field `xyz_acc[3]` may contain the following information:


```c++
double xyz_acc[3] = { x_acc,y_acc,z_acc };
```


<table><tr><td><p><strong>Element</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>x_acc</p>
</td><td><p>Acceleration (m/s²) of IMU on x-axis in world coordinate system</p>
</td></tr>
<tr><td><p>y_acc</p>
</td><td><p>Acceleration (m/s²) of IMU on y-axis in world coordinate system</p>
</td></tr>
<tr><td><p>z_acc</p>
</td><td><p>Acceleration (m/s²) of IMU on z-axis in world coordinate system</p>
</td></tr>
</table>

The field `pos_world[3]` may contain the following information:


```c++
double pos_world[3] = {x,y,yaw};
```


<table><tr><td><p><strong>Element</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>x</p>
</td><td><p>X-coordinate value (m) of the robot in world coordinate system</p>
</td></tr>
<tr><td><p>y</p>
</td><td><p>Y-coordinate value (m) of the robot in world coordinate system</p>
</td></tr>
<tr><td><p>yaw</p>
</td><td><p>Yaw angle (rad) of the robot in world coordinate system</p>
</td></tr>
</table>

The field `vel_world[3]` may contain the following information:


```c++
double vel_world[3] = {x_vel,y_vel,yaw_vel};
```


<table><tr><td><p><strong>Element</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>x_vel</p>
</td><td><p>Linear velocity (m/s) on the x-axis of the world coordinate system</p>
</td></tr>
<tr><td><p>y_vel</p>
</td><td><p>Linear velocity (m/s) on the y-axis of the world coordinate system</p>
</td></tr>
<tr><td><p>yaw_vel</p>
</td><td><p>Yaw angular velocity (rad/s) in the world coordinate system</p>
</td></tr>
</table>

The field `vel_body[3]` may contain the following information:


```c++
double vel_body[3] = {x_vel,y_vel,yaw_vel};
```


<table><tr><td><p><strong>Element</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>x_vel</p>
</td><td><p>Linear velocity (m/s) on the x-axis of the body coordinate system</p>
</td></tr>
<tr><td><p>y_vel</p>
</td><td><p>Linear velocity (m/s) on the y-axis of the body coordinate system</p>
</td></tr>
<tr><td><p>yaw_vel</p>
</td><td><p>Yaw angular velocity (rad/s) in the body coordinate system</p>
</td></tr>
</table>

The meanings of the values in the field `robot_motion_state` are as follows: 


<table><tr><td><p><strong>Variable Value</strong></p>
</td><td><p><strong>Robot&#x27;s motion state</strong></p>
</td></tr>
<tr><td><p>0</p>
</td><td><p>In the state corresponding to the value of <code>robot_basic_state</code></p>
</td></tr>
<tr><td><p>1</p>
</td><td><p>Stepping with the gait corresponding to the value of <code>robot_gait_state</code>/</p>
<p>Stepping with the AI gait corresponding to the value of <code>robot_policy_state</code></p>
</td></tr>
<tr><td><p>2</p>
</td><td><p>The robot is performing Twist</p>
</td></tr>
<tr><td><p>4</p>
</td><td><p>The robot is performing Twist Jump</p>
</td></tr>
<tr><td><p>11</p>
</td><td><p>The robot is performing Long Jump</p>
</td></tr>
</table>

[Notes] The robot's basic status `robot_basic_state`, gait state `robot_gait_state`/AI gait state `robot_policy_state`, and motion state `robot_motion_state` together define the robot's status. Please refer to Appendix 2.2 for details. 

The values in the field `is_robot_need_move` have the following meanings: 


<table><tr><td><p><strong>Variable Value</strong></p>
</td><td><p><strong>Balanced state</strong></p>
</td></tr>
<tr><td><p>0</p>
</td><td><p>Able to maintain balance</p>
</td></tr>
<tr><td><p>1</p>
</td><td><p>Unable to maintain balance, and needs to step to adjust its posture</p>
</td></tr>
</table>

The values in the field `zero_position_flag` have the following meanings: 


<table><tr><td><p><strong>Variable Value</strong></p>
</td><td><p><strong>Zero position</strong></p>
</td></tr>
<tr><td><p>0</p>
</td><td><p>Reset-to-Zero uncompleted or exited</p>
</td></tr>
<tr><td><p>1</p>
</td><td><p>Reset-to-Zero completed</p>
</td></tr>
</table>

The values in the field `is_after_first_start` have the following meanings: 


<table><tr><td><p><strong>Variable Value</strong></p>
</td><td><p><strong>Status</strong></p>
</td></tr>
<tr><td><p>0</p>
</td><td><p>Not first start</p>
</td></tr>
<tr><td><p>1</p>
</td><td><p>First start</p>
</td></tr>
</table>

The values in the field `is_voice_control_enable` have the following meanings: 


<table><tr><td><p><strong>Variable Value</strong></p>
</td><td><p><strong>Voice control function state</strong></p>
</td></tr>
<tr><td><p>0</p>
</td><td><p>Off</p>
</td></tr>
<tr><td><p>1</p>
</td><td><p>On</p>
</td></tr>
</table>

The field `ultrasound[2]` may contain the following information:


```c++
double ultrasound[2] = {forward_distance,backward_distance};
```


<table><tr><td><p><strong>Element</strong></p>
</td><td><p><strong>Meaning</strong></p>
</td></tr>
<tr><td><p>forward_distance</p>
</td><td><p>Distance between the robot and the front obstacle (m)</p>
</td></tr>
<tr><td><p>backward_distance</p>
</td><td><p>Distance between the robot and the rear obstacle (m)</p>
</td></tr>
</table>

[Notes] The valid range of ultrasound is [0.28m, 4.50m]. When the distance to the obstacle is less than 0.28m, it is displayed as 0.28m. When the distance to the obstacle is greater than 4.50m, it is displayed as 4.50m.


#### 1.3.2 Joint Information

Information of robot joints includes angles, angular velocities and other information. The command codes are as follows:


<table><tr><td><p><strong>Command</strong></p>
</td><td><p><strong>Code</strong></p>
</td><td><p><strong>Type</strong></p>
</td><td><p><strong>Content</strong></p>
</td><td><p><strong>Frequency</strong></p>
</td></tr>
<tr><td><p>Angle</p>
</td><td><p>0x0902</p>
</td><td><p>1</p>
</td><td><p><code>RobotJointAngle</code> structure (details below) </p>
</td><td><p>100 Hz</p>
</td></tr>
<tr><td><p>Angle velocity</p>
</td><td><p>0x0903</p>
</td><td><p>1</p>
</td><td><p><code>RobotJointVel</code> structure (details below) </p>
</td><td><p>100 Hz</p>
</td></tr>
</table>


```c++
/// Robot's joint angle 0x0902
struct RobotJointAngle{
double joint_angle[12]; /// Unit:rad
};
/// Robot's joint angular velocity 0x0903
struct RobotJointVel{
double joint_vel[12]; /// Unit:rad/s
};
```

Each leg of Jueying Lite3 consists of 3 joints: HipX (the hip joint for abduction and adduction), HipY (the hip joint for flexion and extension), and Knee.

The order of the 12 joints is FL_HipX, FL_HipY, FL_Knee, FR_HipX, FR_HipY, FR_Knee, HL_HipX, HL_HipY, HL_Knee, HR_HipX, HR_HipY, HR_Knee (F, H, L and R means front, hind, left and right respectively).


---


## 2 Appendix


### 2.1 UDP Message Parsing

In this section, the joint angle data uploaded by the robot is analyzed as an example.


```c++
/* Raw data, total length of 108 bytes
 * 0209 0000 6000 0000 0100 0000 2efd 3ccf 
 * 47e4 e1bf f130 4992 f49e e7bf c3d7 eefe 
 * 8567 f13f 9ae4 b66d 6583 f1bf 24d8 f33c 
 * 3d00 f13f 8c16 222a 0b1f 0140 e8ba aaaa 
 * 3803 ef3f d009 0000 60bd d2bf db04 3353 
 * 1150 e73f 7d0b 0000 72f0 e53f 0a3d 0cc3 
 * f071 e73f c3d7 eefe 7b52 f93f
*/
/**********************************************************************************/
///Content analysis
// 0209 0000 	// xxxx command code is 0x0902
// 6000 0000 	// yyyy data length of the main text is 0x60, which is 96 bytes in length
// 0100 0000 	// zzzz is 1, indicating it is a complex command followed by main text
/*
 * bbbbbbbbbb...................
 * The following is the main text, with a total of 96 bytes; the data is an array of double type with a length of 12
 * 2efd 3ccf 47e4 e1bf f130 4992 f49e e7bf 
 * c3d7 eefe 8567 f13f 9ae4 b66d 6583 f1bf 
 * 24d8 f33c 3d00 f13f 8c16 222a 0b1f 0140 
 * e8ba aaaa 3803 ef3f d009 0000 60bd d2bf 
 * db04 3353 1150 e73f 7d0b 0000 72f0 e53f 
 * 0a3d 0cc3 f071 e73f c3d7 eefe 7b52 f93f
*/
// Corresponding to robot joint angles
double joint_angle[12] = {-0.5591162726995316, -0.7381537301203293, 1.0877742727584383,
-1.0945791516990426, 1.0625584011991203, 2.1401580135014395,
0.9691432317102597, -0.2928085327149832, 0.7285238862024487,
0.6856012344363617, 0.7326587495353192, 1.5826377828902742};
```


### 2.2 Lookup Table for Robot Status

The values of (robot_basic_state, robot_gait_state/robot_policy_state, robot_motion_state) correspond to the following states in the table:


<table><tr><td><p><strong>Value</strong></p>
</td><td><p><strong>State</strong></p>
</td></tr>
<tr><td><p>(1,0,0)</p>
</td><td><p>Sitting State</p>
</td></tr>
<tr><td><p>(1,0,11)</p>
</td><td><p>The robot is performing Long Jump</p>
</td></tr>
<tr><td><p>(4,0,0)</p>
</td><td><p>Prepare State</p>
</td></tr>
<tr><td><p>(5,0,0)</p>
</td><td><p>Sit-to-Stand State</p>
</td></tr>
<tr><td><p>(6,0,0)</p>
</td><td><p>In Torque-control state (standing) with Flat gait in Slow gear</p>
</td></tr>
<tr><td><p>(6,0,1)</p>
</td><td><p>Stepping with Flat gait in Slow gear or twisting the body according to axis command</p>
</td></tr>
<tr><td><p>(6,0,2)</p>
</td><td><p>The robot is performing Twist</p>
</td></tr>
<tr><td><p>(6,0,4)</p>
</td><td><p>The robot is performing Twist Jump</p>
</td></tr>
<tr><td><p>(6,2,0)</p>
</td><td><p>In Torque-control state (standing) with RUG gait in General gear</p>
</td></tr>
<tr><td><p>(6,2,1)</p>
</td><td><p>Stepping with RUG gait in General gear</p>
</td></tr>
<tr><td><p>(6,4,0)</p>
</td><td><p>In Torque-control state (standing) with Flat gait in Medium gear</p>
</td></tr>
<tr><td><p>(6,4,1)</p>
</td><td><p>Stepping with Flat gait in Medium gear</p>
</td></tr>
<tr><td><p>(6,5,0)</p>
</td><td><p>In Torque-control state (standing) with Flat gait in Fast gear</p>
</td></tr>
<tr><td><p>(6,5,1)</p>
</td><td><p>Stepping with Flat gait in Fast gear</p>
</td></tr>
<tr><td><p>(6,6,0)</p>
</td><td><p>In Torque-control state (standing) with RUG gait in Grip gear</p>
</td></tr>
<tr><td><p>(6,6,1)</p>
</td><td><p>Stepping with RUG gait in Grip gear</p>
</td></tr>
<tr><td><p>(6,12,1)</p>
</td><td><p>The robot is performing Moonwalk</p>
</td></tr>
<tr><td><p>(6,13,0)</p>
</td><td><p>In Torque-control state (standing) with RUG gait in H-Step gear</p>
</td></tr>
<tr><td><p>(6,13,1)</p>
</td><td><p>Stepping with RUG gait in H-Step gear</p>
</td></tr>
<tr><td><p>(7,0,0)</p>
</td><td><p>Sitting State</p>
</td></tr>
<tr><td><p>(8,0,0)</p>
</td><td><p>Lose Control Protection State</p>
</td></tr>
<tr><td><p>(9,0,0)</p>
</td><td><p>Posture Adjustment State</p>
</td></tr>
<tr><td><p>(11,0,0)</p>
</td><td><p>Flipping over state</p>
</td></tr>
<tr><td><p>(16,0,1)</p>
</td><td><p>In AI state and in the AI base gait</p>
</td></tr>
<tr><td><p>(16,16,1)</p>
</td><td><p>In AI state and in the AI jump gait</p>
</td></tr>
<tr><td><p>(16,18,1)</p>
</td><td><p>In AI state and in the AI stand gait</p>
</td></tr>
<tr><td><p>(16,20,1)</p>
</td><td><p>In AI state and in the AI fast gait</p>
</td></tr>
<tr><td><p>(17,0,0)</p>
</td><td><p>Reset-to-Zero State</p>
</td></tr>
<tr><td><p>(18,0,0)</p>
</td><td><p>Backflip State(be performing Backflip action)</p>
</td></tr>
<tr><td><p>(20,0,0)</p>
</td><td><p>Hello State(be performing Hello action)</p>
</td></tr>
</table>

