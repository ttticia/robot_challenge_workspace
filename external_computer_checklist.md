# External computer setup checklist

The robot runs only the onboard half of perception (`src/team_perception`: `qr_scanner`,
`poster_detector`, `image_throttle`). YOLO person detection (`human_detector`) has to run on a
separate computer. **Without it, QR codes still work but no poster is ever reported.**

Package to install there: `team_perception_external`, from the
[Robot-Dog-Grp4](https://github.com/ttticia/Robot-Dog-Grp4) repo
(`challenge_files/external/team_perception_external`). Full reference: that repo's
[`challenge_files/onboard-external_communication.md`](https://github.com/ttticia/Robot-Dog-Grp4/blob/main/challenge_files/onboard-external_communication.md)
(§7 network, §8.2 setup, §10 link check, §11 troubleshooting).

Tick each box in order. Each step depends on the one before.

## 1. Pick the machine

- [ ] Ubuntu 22.04 with ROS 2 Humble installed (`ros2 --help` works after
      `source /opt/ros/humble/setup.bash`).
- [ ] An NVIDIA GPU is optional. A CPU works at a lower frame rate. Write down which you have,
      because step 3 and step 6 differ.
- [ ] It can reach the lab network: it can be on `192.168.10.x`, the same subnet as the robot
      computer (`192.168.10.74`). Use wired, or 5 GHz Wi-Fi near the access point (needs about
      6 Mbit/s from the robot).
- [ ] It is a different machine from the robot computer. The robot computer has no GPU and its
      CPU is already busy with Fast-LIO, SLAM and Nav2.

## 2. Get the package

- [ ] Clone Robot-Dog-Grp4 on this machine (or copy the folder over with `scp`).
- [ ] Copy **only** the package folder into a new workspace. Don't copy
      `challenge_files/COLCON_IGNORE`, or colcon will skip the package:
      ```bash
      mkdir -p ~/ext_ws/src
      cp -r Robot-Dog-Grp4/challenge_files/external/team_perception_external ~/ext_ws/src/
      ```

## 3. Install dependencies and YOLO weights

- [ ] Run the installer (asks for sudo for apt; downloads about 200 MB for CPU torch):
      ```bash
      bash ~/ext_ws/src/team_perception_external/install_deps.sh         # CPU
      bash ~/ext_ws/src/team_perception_external/install_deps.sh --gpu   # NVIDIA GPU
      ```
- [ ] It ends with `OK: torch ..., CUDA available: ...`. With `--gpu`, check that this says `True`.
- [ ] `~/.cache/team_perception/yolo11n.pt` exists. **No internet on this machine?** Copy that
      file (about 5 MB) from a machine that ran the installer, and install the pip packages from
      wheels (see §8.2 of the reference doc).

## 4. Build

- [ ] Build from the workspace root, never from `src/`:
      ```bash
      cd ~/ext_ws
      source /opt/ros/humble/setup.bash
      colcon build --symlink-install --packages-select team_perception_external
      source install/setup.bash
      ```
- [ ] If the build fails with `_distribution_fullname`, run `pip3 uninstall -y setuptools` and
      build again.

## 5. Join the robot's ROS graph

- [ ] Use the same domain as the robot: `export ROS_DOMAIN_ID=4`. Leave `ROS_LOCALHOST_ONLY`
      unset, and use the default DDS (Fast DDS).
- [ ] Optional: add these three lines to `~/.bashrc` so every new terminal is ready:
      ```bash
      source /opt/ros/humble/setup.bash
      source ~/ext_ws/install/setup.bash
      export ROS_DOMAIN_ID=4
      ```
- [ ] `ros2 topic list | grep camera` shows the robot's `/camera/...` topics. If they don't
      appear, check the domain ID on both machines and the subnet, then whether the network
      blocks multicast (§11 of the reference doc).
- [ ] Only use domain 4 here for real runs. For offline tests, use another ID and
      `ROS_LOCALHOST_ONLY=1`.

## 6. Run

- [ ] **Robot computer:** `ros2 launch team_perception robot.launch.py` (leave `posters`
      at its default of `true`). On slow Wi-Fi, add `offboard_rate_hz:=5.0`.
- [ ] **External computer:**
      ```bash
      ros2 launch team_perception_external external.launch.py              # CPU
      ros2 launch team_perception_external external.launch.py device:=0    # NVIDIA GPU
      ```
- [ ] The external side's `rate_hz` (default 10) must not be higher than the robot's
      `offboard_rate_hz`.

## 7. Check the link

| # | Where | Command | Expect |
|---|---|---|---|
| 1 | external | `ros2 topic hz /camera/color/image_offboard/compressed` | about 10 Hz (= `offboard_rate_hz`) |
| 2 | robot | `ros2 topic hz /humans/detections` (someone in front of the camera) | a steady rate |
| 3 | robot | `ros2 topic echo /humans/detections --once` | JSON with `stamp`, `image_size`, `people` |
| 4 | robot | `ros2 topic echo /found` (show a person poster) | `Human found` after a few seconds |

- [ ] All four pass. Note the frame rate from step 2 and whether a GPU was used, so the team
      knows what to expect.

## If something fails

| Symptom | Fix |
|---|---|
| No robot topics on the external computer | Same `ROS_DOMAIN_ID` on both? Same `192.168.10.x` subnet? If both are yes, multicast is probably blocked: set unicast peers in a Fast DDS profile |
| Check 1 shows nothing | The robot's `robot.launch.py` isn't running, or was started with `posters:=false` |
| Check 1 well below 10 Hz | Weak Wi-Fi: go wired or move closer, or lower `offboard_rate_hz` |
| Robot logs `no depth image within max_dt_s of the detection` | The round trip takes over 2 s: lower `offboard_rate_hz`, use the GPU, or set a smaller `imgsz` |
| People detected but never `Human found` | No TF to `map` (start localisation), the poster is more than 4 m away, or there are too few votes yet (needs 5 of 8 frames) |
| `ros2 topic echo` fails with `!rclpy.ok()` | `ros2 daemon stop; ros2 daemon start` |
