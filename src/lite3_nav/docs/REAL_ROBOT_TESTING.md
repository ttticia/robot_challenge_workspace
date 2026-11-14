# Testing lite3_nav on the real Lite3: step-by-step guide (Group 4)

This guide takes Group 4 from "laptop in the lab" to a supervised autonomous exploration run with
`lite3_nav`, then to recording depth-camera data for the curtain work. Each stage has a **pass check**.
Don't start a stage until the previous one has passed.

Sources: Lab Exercise 1 (remote access), Lab Exercises 4 and 5 (robot sessions, lite3_launcher,
System Computer), the Legged Robot Challenge 2026 Quickstart Guide (session checklist), the team's
`robot_info.md` (robot interface, measured 2026-11-13) and `src/lite3_mission/README.md`
(supervised execution).

> **Marked TBC** = an assumption that still needs confirming. Fix it in this file once it's confirmed.

---

## 0. Quick reference (Group 4)

| What | Address / value | Notes |
|---|---|---|
| Lab Wi-Fi | SSID `RI_network` | Password: Lab Exercise 1, page 2 |
| Robot hotspot (outdoor / no RI_network) | SSID `ysc-…` (written on the robot) | Lab Exercise 5, page 2 |
| **Group Computer on the robot** (runs our code, RViz via Remote Desktop) | **`192.168.10.74`** on RI_network, **`192.168.1.74`** on the robot hotspot | 70 + group number. Username `user`; the password is your group's (default in Lab Exercise 5, page 2) |
| System Computer (drivers, domain bridge; SSH only, shared with the other group) | `192.168.10.62` / `192.168.1.62` (**robot 2, TBC**) | 60 + robot number. **Never change its password or configuration** |
| Class virtual computers (Labs 1–3; not on the robot) | `192.168.10.104` (mini-group A), `192.168.10.124` (B) | 100 + group / 120 + group |
| ROS domain for the robot | `ROS_DOMAIN_ID=4` | The sim uses 89, our tests use 88: never 4 |
| DDS | Fast DDS, **UDP-only profile** (set by `robot_env.sh`) | Nav2 lifecycle startup stalls with shared memory |
| Mode switching | `lite3_auto` / `lite3_manual` (aliases on the Group Computer) | Only the operator switches modes |
| Emergency | Handheld controller **STOP**; software stop = `/simple_cmd` code `0x21020C0E` | Software zero velocity is **not** an e-stop |

---

## 1. Before the session (at home)

1. Check the branch builds and its tests pass on your laptop (WSL):
   ```bash
   cd ~/challenge_ws && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   colcon build --symlink-install --packages-select lite3_mission lite3_nav
   bash /mnt/c/GitHub/robot_challenge_workspace/src/lite3_nav/test/run_tests.sh
   ```
   Expected: `lite3_nav` 16 passed.
2. Make sure the code can reach the Group Computer (see stage 3). Our branch
   `feat/no-ref/navigation-rey` is **not pushed yet (TBC)**. Either push it, or bring a tarball:
   ```bash
   cd /mnt/c/GitHub/robot_challenge_workspace/src && tar czf ~/lite3_nav.tgz lite3_nav
   ```
3. Book the robot with the other group sharing it. Only one group may control it at a time.
4. Bring: a charged laptop, a tape measure (for the curtain distances in stage 10), and a phone to
   film each run.
5. Roles for every robot stage: one **operator** (holds the handheld controller with STOP, stays next
   to the robot), one **driver** (types the commands), one **observer** (watches RViz and takes notes).

---

## 2. Connect to the lab network and the virtual desktop (from Lab Exercise 1)

### 2.1 Network
1. Connect the laptop to `RI_network` (or the robot hotspot `ysc-…` if instructed; then use the
   `192.168.1.x` addresses everywhere below).
2. Check: `ping 192.168.10.74` from PowerShell answers.

### 2.2 Remote Desktop (for RViz and GUI tools)
- **Windows:** open *Remote Desktop Connection* → computer `192.168.10.74` → Connect → username `user`
  and your group's password → OK. If a second (Ubuntu) login screen appears, enter the password again
  (if the screen looks different, click it and press Enter).
- **macOS:** install *Windows App* → **+** → *Add PC* → PC name `192.168.10.74` → Add → double-click
  it → Connect → username/password → then the password again on the Ubuntu screen. If the PC "cannot
  be found", allow *Local Network* access for Windows App in macOS settings.
- **Linux:** *Remmina* → **+** → protocol RDP, server `192.168.10.74`, username `user`, password →
  Save → connect; enter the password again on the Ubuntu screen. Keep the resolution modest
  ("Use client resolution" or a custom small size), because high resolutions lag.

Pass check: the Ubuntu desktop of the Group Computer is visible, and a terminal (`Ctrl+Alt+T`) opens.

### 2.3 SSH (faster for commands)
```bash
ssh user@192.168.10.74
```
Type `yes` the first time to accept the host fingerprint. `Ctrl+Alt+T` / `Ctrl+Shift+T` only work in
Remote Desktop, not over SSH. Several SSH windows (or `tmux`) are the easiest way to run the many
terminals below.

### 2.4 VS Code Remote SSH (optional, for editing)
VS Code → bottom-left *Open a Remote Window* → *Connect to Host* → *Add New SSH Host* →
`ssh user@192.168.10.74` → default config file → connect → platform *Linux* → password.

### 2.5 Passwords
If your group never changed the default password on the **Group Computer**, change it once over SSH
with `passwd` and share it only within the group. **Do not** change the System Computer's password.

---

## 3. Put lite3_nav on the Group Computer and build it

Run this in an SSH terminal on `192.168.10.74`.

1. Check what's there and what ROS it runs:
   ```bash
   echo $ROS_DISTRO          # expected: humble (the Quickstart warns the robot may run Jazzy: TBC)
   ls ~/challenge_ws/src     # expected: lite3_mission and team_perception from the team repo
   ```
2. Get the code (pick one):
   - **git (if the Group Computer can reach GitHub, TBC):**
     ```bash
     cd ~/challenge_ws/src/<repo> && git fetch origin && git checkout feat/no-ref/navigation-rey
     ```
   - **Copy from your laptop** (PowerShell or WSL, on the lab network):
     ```bash
     scp ~/lite3_nav.tgz user@192.168.10.74:~/
     ```
     then on the Group Computer:
     ```bash
     mkdir -p ~/challenge_ws/src && tar xzf ~/lite3_nav.tgz -C ~/challenge_ws/src
     ```
   `lite3_nav` needs the team's **unchanged** `lite3_mission` next to it in the same workspace.
3. Build only our two packages (don't rebuild the course packages):
   ```bash
   cd ~/challenge_ws && source /opt/ros/humble/setup.bash
   source ~/lite3_ws/install/local_setup.bash && source ~/ros2_ws/install/local_setup.bash
   colcon build --symlink-install --packages-select lite3_mission lite3_nav
   source install/setup.bash
   ros2 pkg executables lite3_nav     # expected: lite3_nav mission, lite3_nav velocity_gate
   ```

Pass check: both executables are listed.

---

## 4. Power-up and handset (operator, every power cycle)

From the Quickstart Guide session checklist (Lab Session 3):
1. Fit the battery, switch the robot on.
2. Connect the handheld controller to the robot's network, press **Control**, rotate the legs when
   prompted, press **Stand/Sit** to stand and calibrate.
3. The robot stays in **Manual** mode. Nobody sends `lite3_auto` until stage 8.
4. Check the battery is high enough for the planned runs (1.5–2 h per battery, Quickstart).

Pass check: the robot stands steadily and the operator can walk it with the handset.

---

## 5. Read-only connection checks (publishes nothing)

Open a **new** SSH terminal on the Group Computer for each numbered terminal from now on.

**Terminal A (robot terminal):**
```bash
source ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_env.sh
```
This sets domain 4, `ROS_LOCALHOST_ONLY=0`, Fast DDS UDP-only, marks the prompt `[ROBOT d4]`, then
checks the 192.168.10.x address, pings `192.168.10.74` and `.62`, and lists topics.

Then:
```bash
bash ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_check.sh
```
This measures rates for `/imu/data`, `/livox/points`, `/odom`, `/scan` and prints who is on `/cmd_vel`.

Expected before the lab stack runs: `/imu/data` ≈ 200 Hz, `/livox/points` ≈ 10 Hz; `/odom` and `/scan`
**missing** (they come from the lab stack in stage 6). `/cmd_vel` has the subscriber
`lite3_domain_bridge_4` and **no publisher**.

Also note down:
```bash
ros2 topic info -v /cmd_vel          # bridge subscription QoS: RELIABLE + VOLATILE or TRANSIENT_LOCAL
ros2 node list                        # anything already running? (another group's stack?)
date; ssh user@192.168.10.62 date     # clocks within ~0.1 s? (TF age checks depend on it)
```

Pass check: `/imu/data` and `/livox/points` arrive, nobody publishes `/cmd_vel`, and no other
SLAM/Nav2 stack is running (`ros2 node list` shows no `slam_toolbox`, `bt_navigator` or `controller_server`).
If drivers are dead: tell the other group, then
`ssh user@192.168.10.62` → `sudo systemctl restart lite3.service` (Lab Exercise 5, page 11).

---

## 6. Start the lab SLAM + Nav2 stack (team wrapper, unchanged)

**Terminal B:** source `robot_env.sh` as in stage 5, then:
```bash
ros2 launch lite3_mission lab_navigation.launch.py slam:=True
```
The wrapper starts the course `lite3_launcher` with its `/cmd_vel` output remapped to
`/mission/nav_cmd_vel`, so our velocity gate becomes the only `/cmd_vel` publisher.

**Critical check:** the Quickstart Guide says the lab launcher **does not accept Nav2 goals while
mapping** (`slam:=True`). Our exploration needs mapping and goals at the same time. Check:
```bash
ros2 node list | grep -E "bt_navigator|controller_server|slam"
ros2 lifecycle get /bt_navigator                 # want: active [3]
ros2 action list | grep navigate_to_pose
ros2 topic info -v /mission/nav_cmd_vel          # want: exactly 1 publisher (twist_mux), TBC
ros2 topic hz /odom ; ros2 topic hz /scan        # Ctrl+C each after a few seconds
```

Pass check: `/odom` and `/scan` arrive, the map grows in RViz (stage 7), `bt_navigator` is **active**,
and `/mission/nav_cmd_vel` has exactly one publisher.
**If `bt_navigator` is missing or inactive while mapping, stop here.** Exploration can't run on the lab
launcher as it is. Note what you saw and report it; the fallback is our own slam_toolbox + Nav2 launch,
like the one used in simulation (not built yet).

---

## 7. RViz on the Group Computer (Remote Desktop)

In the Remote Desktop session, open a terminal, source `robot_env.sh`, then `rviz2`:
- Fixed Frame `map`; add **Map** `/map` (Durability **Transient Local**).
- Add **Pose** `/mission/candidate_goal`, **LaserScan** `/scan`, **TF**.
- Optional **Image** `/camera/color/image_raw/compressed`. Never view raw images over Wi-Fi.

---

## 8. Mission in shadow mode (cannot drive)

**Terminal C:** source `robot_env.sh`, then:
```bash
ros2 launch lite3_nav mission.launch.py mode:=explore
```
Shadow mode creates no Nav2 client, no drive lease and no `/cmd_vel` publisher.

**Terminal D:** source `robot_env.sh`, then:
```bash
ros2 topic echo /mission/status
ros2 topic echo /mission/gate_status
```

Pass check: `/mission/status` state moves past `WAITING_FOR_MAP` / `WAITING_FOR_POSE_TF` /
`IMU_STALE_OR_INVALID` to `SHADOW_FRONTIER`, and RViz shows sensible candidate goals on the map
frontier. Gate status shows `PREVIEW_ONLY`.
If it stays at `POSE_TIMESTAMP_STALE_OR_FUTURE`, check the clocks (stage 5).

Stop it with `Ctrl+C` before stage 9.

---

## 9. Supervised driving: first a short waypoint, then exploration

Rules for every run: area clear (≥ 3 m), operator next to the robot with STOP, the other group agrees,
and the robot returns to Manual at the end.

### 9.1 Gate with drive enabled, robot still in Manual
**Terminal C:**
```bash
ros2 launch lite3_nav mission.launch.py execution:=true drive:=true
```
(Default mode `waypoints`: one goal 0.5 m straight ahead of the starting heading.)

Check in Terminal D:
- `/mission/gate_status` reason `READY` (otherwise it names the problem, e.g.
  `EXPECTED_1_NAV2_INPUT_PUBLISHERS_GOT_0`, or `COMPETING_COMMAND_SOURCE: …`).
- `ros2 topic info -v /cmd_vel`: publisher `velocity_gate` (RELIABLE, TRANSIENT_LOCAL) **matched** to
  the bridge's subscription.

### 9.2 First motion, 0.5 m forward
1. Operator: robot standing, path clear, STOP in hand.
2. **Terminal D:** `lite3_auto`
3. **Terminal D:** arm:
   ```bash
   ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: true}'
   ```
   If it answers `success=False`, read the message (e.g. `NAV2_BT_NAVIGATOR_NOT_ACTIVE`,
   `VELOCITY_GATE_NOT_READY: …`) and fix that first.
4. Watch: the robot walks ~0.5 m at ≤ 0.1 m/s; the status reaches `GOAL_SUCCEEDED`, then
   `MISSION_COMPLETE` (it disarms itself).
5. **Terminal D:** `lite3_manual`

Stop at any time: operator STOP, or
`ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: false}'`, then `lite3_manual`.

### 9.3 Does the robot stop when commands stop? (open question in robot_info.md §10)
With the robot in Auto, 0.5 m running start, operator ready: `Ctrl+C` the mission launch (Terminal C)
**while the robot is walking**. The gate sends a zero and then goes quiet. Note whether the robot stops
and how far it travels. If it keeps walking, STOP and **do not run exploration** until we've discussed it.

### 9.4 Exploration
**Terminal C:**
```bash
ros2 launch lite3_nav mission.launch.py mode:=explore execution:=true drive:=true
```
Then `lite3_auto`, arm, and watch. Expected: frontier goals one after another; the map grows; when no
reachable frontier is left (or at a curtain), the mission disarms with `NO_REACHABLE_FRONTIER`.
Afterwards: `lite3_manual`.

Record for each run: start pose, goals reached, any disarm reason from `/mission/status`, time, and a
video. Save the map:
```bash
ros2 run nav2_map_server map_saver_cli --free 0.196 -f ~/maps/run_$(date +%H%M)
```

Tuning (one change at a time, robot disarmed):
```bash
ros2 param set /velocity_gate max_linear_m_s 0.05
ros2 param set /mission frontier_clearance_m 0.25
```

---

## 10. Curtain data for the depth-camera plan (no autonomy)

Goal: find out whether the depth camera sees floor under a real curtain.
1. Operator walks the robot (Manual) to face a curtain squarely, camera height noted.
2. Measure the curtain's bottom gap above the floor.
3. At 0.7 m, 1.1 m and 1.6 m from the curtain (tape measure from the camera, at the front of the head),
   record 10 s each:
   ```bash
   ros2 bag record -o ~/bags/curtain_$(date +%H%M)_1p1m /camera/depth/image_raw /camera/depth/camera_info \
     /camera/color/image_raw/compressed /tf_static /imu/data
   ```
4. Do the same once facing a solid wall (negative example).
5. Copy the bags to the laptop:
   ```bash
   scp -r user@192.168.10.74:~/bags ./
   ```

Estimate for these distances: see "Curtains (planned)" in `src/lite3_nav/README.md`.

---

## 11. End of session

1. `ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: false}'`, then `lite3_manual`.
2. `Ctrl+C` the mission, then the lab stack. Check `ros2 node list` is clean.
3. Operator sits the robot down; battery off and on charge.
4. Tell the other group the robot is free.
5. Write down results and any TBC answers in this file.

---

## 12. Troubleshooting

| Problem | What to check |
|---|---|
| Can't reach 192.168.10.74 | On RI_network? On the hotspot the address is `192.168.1.74` |
| No topics in `ros2 topic list` | `ROS_DOMAIN_ID=4` and `ROS_LOCALHOST_ONLY=0` (source `robot_env.sh`); drivers alive (stage 5) |
| Nav2 never becomes active | Started from a terminal **with** the UDP-only profile (`robot_env.sh`)? Another stack already running? |
| Gate `EXPECTED_1_NAV2_INPUT_PUBLISHERS_GOT_n` | `ros2 topic info -v /mission/nav_cmd_vel`. 0: lab stack not remapped or not running. 2+: an extra producer; the expected count is `expected_input_publishers` |
| Gate `COMPETING_COMMAND_SOURCE` | Someone else publishes `/cmd_vel` (teleop, `ros2 topic pub`, a second stack): stop it |
| Arms but the robot doesn't move | Robot in Auto (`lite3_auto`)? `/cmd_vel` publisher matched to the bridge (`ros2 topic info -v /cmd_vel`)? |
| `POSE_TIMESTAMP_STALE_OR_FUTURE` | Clock offset between computers; Fast-LIO running? |
| Disarms with `NO_REACHABLE_FRONTIER` early | Look at the map: closed by curtains/doors? (expected); or `frontier_clearance_m` too large for the passages |
| Robot unsteady / falls | STOP, operator recovers; don't request H-Step gait (robot_info §6.3) |

---

## Open items for this guide (TBC)

- Robot number / System Computer address (`192.168.10.62` assumed from robot_info.md).
- Does the Group Computer reach GitHub, or do we copy code with `scp`?
- `echo $ROS_DISTRO` on the Group Computer: Humble (robot_info.md) or Jazzy (Quickstart Guide)?
- Does `lite3_launcher` with `slam:=True` start an **active** Nav2 (stage 6)?
- Number of publishers on `/mission/nav_cmd_vel` with the lab stack (expected 1, twist_mux).
- Does the robot stop when `/cmd_vel` goes quiet (stage 9.3)?
