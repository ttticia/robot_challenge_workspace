# Lite3 Venture Edition Motion Host SDK Mode

Source: [Lite3 Venture Edition Motion Host SDK Mode](https://alidocs.dingtalk.com/i/p/OlnXRxOLAljEbGLp/docs/P7QG4Yx2Jp7mXxnmiQ2bpp5nV9dEq3XD)

Last Updated：2026-07-06

Document Version: V1.0.0

Document Software Requirements:

robot_server V2.4.23 or later;

System software package V3.8.3 or later.

To check the robot_server version, access the navigation host (default address: 192.168.1.106) and run the following command:


```plaintext
dpkg -l robot_server
```


---


---


## 1 Preparation


### 1.1 SSH Connection

All configuration must be performed on the motion host. First, connect to the motion host via SSH.

[Note] Before establishing an SSH connection, make sure both your computer and controller are connected to the Wi-Fi network named `lite3********` with the password `12345678` (if the password is incorrect, please contact technical support).


```shell
# ssh <username>@<motion_host_IP>
ssh XXXXXX@192.168.X.XXX   

# Default motion host username and motion host address: ysc@192.168.2.1
# Default password: ' (single quote)
# Enter the password to connect remotely to the Lite3 motion host
```


### 1.2 External Network Connections


```shell
# Detect available Wi-Fi networks and connect to them
sudo nmcli dev wifi list
sudo nmcli dev wifi connect "name" password "password" ifname wlan0
```

Once connected, you can use `ping` command to check whether you can connect to the external network.

If successful, proceed to the installation steps for “1.3 SDK Service Feature Pack”;

If unsuccessful, enter the following command in the terminal to open the configuration file:


```shell
cd ~/etc/netplan
vim config.yaml
```

Delete the gateway configuration:


```shell
# The following is an example; please delete it as appropriate.
network:
    version: 2
    renderer: NetworkManager
    ethernets:
        eth1:
            addresser:
                - 192.168.1.120/24
                - 192.168.137.120/24
            gateway4: 192.168.137.120   # Delete this gateway.
            nameservers:
                addresser: [223.5.5.5]
```

Save and apply the changes:


```shell
sudo netplan apply
```

After applying the changes, use the `ping`  command again to check if you can connect to the external network.


### 1.3 Installing the SDK Service Feature Pack

The SDK Service feature package is a supporting service program for the Lite3 SDK deployment. It converts UDP messages into ROS2 messages used by the SDK and transmits them; users can configure it to start automatically at boot if needed.

The SDK Service package offers two installation options: via image or manually:

Installation via image: First, verify that the SDK Service package is already installed in your Lite3 system image. If it is already installed, proceed to the “2. Enabling SDK Mode” section; if the image has not been upgraded or the corresponding package is not installed, please contact after-sales support.

 Manual installation: Please follow the steps below to install it.


#### 1.3.1 Installing ROS2

The current Lite3 motion control system runs Ubuntu 20.04. Please install ROS2 Foxy, unless your environment already uses a different standardized configuration.

Please follow the installation steps on the official website:

ROS2 foxy: https://docs.ros.org/en/foxy/Installation.html


![ROS2-1.png](images/5c88e62a010b26f4f0f8.png)

![ROS2-2.png](images/91f321406f0a94c5b798.png)


#### 1.3.2 `network.toml`Configuration

After logging in to the motion host via SSH, enter the following command in the terminal to open the network configuration file:


```shell
vim ~/jy_exe/conf/network.toml
```

Modify the `network.toml` configuration file to the following:


```shell
ip = '192.168.2.1' 
target_port = 43897
local_port = 43893

ips = ['192.168.1.103']
ports = [43897]
```

Save your changes and exit.


```shell
:wq
```


#### 1.3.3 Service Installation

Download the package using `git clone` or transfer it using `scp`


```shell
# cd <Project Storage Path>
cd xxxxxxxxxx

# Method 1: Download using `git clone`
git clone https://github.com/DeepRoboticsLab/sdk_deploy.git

# Method 2. Use scp to transfer the file to Lite3; the password is ' (single quote)
ssh ysc@192.168.2.1 "mkdir -p ~/sdk_service/src"
scp -r ~/sdk_deploy/src/drdds ~/sdk_deploy/src/lite3_transfer ~/sdk_deploy/src/lite3_sdk_service ysc@192.168.2.1:~/sdk_service/src
```

Installation

One-click configuration: Run the `setup_service.sh` one-click deployment script located in `lite3_sdk_service/src/lite3_sdk_service` to automatically compile the ROS 2 feature packages and configure them to start automatically at boot. If the one-click configuration fails, proceed to b. Manually compile the software packages.


```shell
cd ~/sdk_service/src/lite3_sdk_service
chmod +x setup_service.sh
./setup_service.sh
```

Compile the software package manually (if the one-click configuration fails, you can choose to compile it manually)


```shell
cd ~/sdk_service
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to lite3_sdk_service
```

Run the program to check if the package was installed successfully:


```shell
source install/setup.bash
ros2 run lite3_sdk_service sdk_service
```

Installation success message:

![sdk service 运行成功.png](images/32480ec4ec3ddaf8636c.png)

(Optional) If you need to configure the service to start automatically at boot, please refer to

the "Auto-start on Boot (Systemd)" section in [sdk_deploy/src/lite3_sdk_service/README.md at e3440a925e96fc7c65efd589ffdb31e5eb3bd180 · DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy/blob/e3440a925e96fc7c65efd589ffdb31e5eb3bd180/src/lite3_sdk_service/README.md)


---


## 2 SDK Mode Enabled

The current default controller mode is set to gamepad mode. If you want to switch to keyboard controls, change `RemoteCommandType::kRetroidGamepad` to `RemoteCommandType::kKeyBoard` in `main.cpp`.

Please use the DEEP Robotics app to complete the remaining steps.

![lite3_app.png](images/254f4e4b5cfdea720f1c.png)


### 2.1 SDK Mode Activation

[Note]Ensure Lite3 switches modes while in a safe state such as idle; failure to do so may result in machine damage or personal injury.

[Note]You must first activate SDK mode on the gamepad, then launch the SDK Deploy terminal.

Click the slider to the right of `SDK Mode` When the slider turns blue, it indicates that SDK Mode is enabled. On the contrary，SDK mode will become inactive, and the system will switch to MPC mode.

![手柄细节.png](images/6d039c9ce4e3029019e1.png)


### 2.2 Lite3 Posture Adjustment

Once SDK mode is activated, Lite3 will automatically perform a zero-reset and enter the ready state. This process takes some time, so please wait approximately 10 seconds.

![机器狗姿势.png](images/9383aadbd1a3eb3e46f8.png)![机器狗姿势.png](images/9383aadbd1a3eb3e46f8.png)

Next, manually rotate each joint to ensure that it moves freely and without resistance, and that it is not in damping mode. Only after doing so should you attempt to stand or walk. Otherwise, the Lite3 may exhibit dangerous behavior, such as suddenly jumping or sprinting.

![关节旋转(1).png](images/e4f5e93ba981a9b11f5e.png)


### 2.3 (Optional) Change topic frequency

You can use the app on the controller to adjust the publication rates for /JOINTS_DATA and /IMU_DATA.

![手柄话题频率.png](images/f296153cbe15102ba315.png)


---


## 3 SDK Deployment

Make sure you are logged in to the sports host via SSH (see 1.1 SSH Connection), and then follow the steps below.


### 3.1 Download the SDK Deploy Feature Package


```shell
# cd <Project Storage Path>
cd xxxxxxxxxx

# Method 1: Download using `git clone`
git clone https://github.com/DeepRoboticsLab/sdk_deploy.git

## Method 2. Use scp to transfer the file to Lite3; the password is ' (single quote)
ssh ysc@192.168.2.1 'mkdir -p ~/Lite3_sdk_deploy/src'
scp -r ~/sdk_deploy/src/drdds ~/sdk_deploy/src/Lite3_sdk_deploy ysc@192.168.2.1:~/Lite3_sdk_deploy/src
```


### 3.2 Compiling the SDK Deploy Feature Package


```shell
cd Lite3_sdk_deploy
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to lite3_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm 
```


### 3.3 Running the SDK Deploy Feature Package


```shell
cd Lite3_sdk_deploy
source install/setup.bash
ros2 run lite3_sdk_deploy rl_deploy
```


---


## 4 Motion Control

Y: Enter standing mode

A: Enter RL control mode

Left joystick: Forward/Left/Backward/Right

Right joystick: Rotate clockwise/counterclockwise

Press both joysticks simultaneously: Enter damping mode (prone position); after 3 seconds, enter idle mode


---


## 5 SDK Mode Disabled

[Note]This operation must be performed while the robot is in a prone position.

![机器狗姿势.png](images/9383aadbd1a3eb3e46f8.png)

[Note]You must first disable SDK mode on the gamepad, then exit the terminal used for SDK deployment.

Disable SDK mode in the gamepad settings

![SDK关闭.jpeg](images/c8d4c73ad8120455c7ba.png)

After turning off SDK mode, press Ctrl+C on the SDK deploy terminal message output page to exit SDK mode.


