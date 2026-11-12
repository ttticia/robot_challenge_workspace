# Recording Camera Video on the Lite3 (rosbag)

How to record the robot's camera while teleoperating, then turn the recording into a viewable `.mp4`.

**Machine:** robot computer `192.168.10.74`, ROS 2 Humble.

---

## 1. Before you start

Open a terminal on the robot computer (VS Code terminal or `ssh`) and load ROS:

```bash
source /opt/ros/humble/setup.bash
```

Do this in **every new terminal**.

Check that the camera is publishing:

```bash
ros2 topic list | grep image
```

You should see `/camera/color/image_raw/compressed`. If it's missing, the camera driver isn't running, so start the robot first.

### Camera topics

| Topic | Use |
|---|---|
| `/camera/color/image_raw/compressed` | **Use this one.** Color camera, compressed (small files) |
| `/camera/color/image_raw` | Same camera, uncompressed (very large) |
| `/image_raw` | Second camera, uncompressed only |

---

## 2. Record while teleoping

You need **two terminals**.

**Terminal 1: teleop.** Use your usual setup, for example:

```bash
ros2 launch transfer transfer_launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keyboard teleop only responds while its terminal is focused.

**Terminal 2: start recording.**

All recordings go in `~/challenge_ws/ROS_bags/`. Create it once if it's missing:

```bash
mkdir -p ~/challenge_ws/ROS_bags
```

```bash
ros2 bag record -o ~/challenge_ws/ROS_bags/<name> /camera/color/image_raw/compressed
```

- Start recording **before** you start driving.
- Press **Ctrl+C** in Terminal 2 to stop.
- Use a **new name** each time (`teleop_run2`, `teleop_run3`, …). The command fails if the folder already exists.

The recording is saved as a folder (`~/challenge_ws/ROS_bags/teleop_run1/`) containing a `.db3` file and `metadata.yaml`.

---

## 3. Check the recording

```bash
ros2 bag info ~/challenge_ws/ROS_bags/<name>
```

`Messages:` should be **more than 0**. If it's 0, the topic name was wrong or the camera wasn't publishing.

Forgot the folder name? List recordings, newest first:

```bash
ls -dt ~/challenge_ws/ROS_bags/*/metadata.yaml | xargs -n1 dirname
```

---

## 4. Convert to video

```bash
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/<name>
```

This creates `~/challenge_ws/ROS_bags/teleop_run1.mp4` (H.264, real-time speed).

Optional: choose the output name and/or a different camera topic:

```bash
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/teleop_run1 ~/challenge_ws/ROS_bags/front_cam.mp4 /image_raw
```

---

## 5. Watch the video

The robot computer has no screen, so watch the video on your own computer:

- **VS Code:** click the `.mp4` in the Explorer sidebar to play it. Right-click it and choose **Download…** to save a copy.
- **Browser:** open **http://192.168.10.74:8080** (FileBrowser), log in, and click the file to play or download it.

---

## Quick reference

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/challenge_ws/ROS_bags
ros2 bag record -o ~/challenge_ws/ROS_bags/teleop_run1 /camera/color/image_raw/compressed   # Ctrl+C to stop
ros2 bag info ~/challenge_ws/ROS_bags/teleop_run1                                           # check Messages > 0
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/teleop_run1                               # -> ~/challenge_ws/ROS_bags/teleop_run1.mp4
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `invalid choice: 'mcap'` | Remove `-s mcap` from the command, or install it with `sudo apt install ros-humble-rosbag2-storage-mcap`. |
| `Messages: 0` in `ros2 bag info` | Wrong topic name. Re-check with `ros2 topic list \| grep image`. |
| `could not connect to display` / Qt platform plugin error | GUI tools (e.g. `rqt_image_view`) don't work over SSH/VS Code. Convert to `.mp4` instead (step 4). |
| `Package 'image_view' not found` | Not needed. Use `bag_to_video.py`. |
| `parameter 'fps' has invalid type` (image_view) | Write `fps:=30.0`, not `fps:=30`. |
| `Topic ... not in bag` (bag_to_video.py) | The script lists the topics that are in the bag. Pass one of them as the 3rd argument. |
| `ros2: command not found` | Run `source /opt/ros/humble/setup.bash`. |
| Disk filling up | Check with `df -h`. Use the `compressed` topic. Delete old bag folders after converting. |
