# Aligning the camera's depth to colour

`poster_detector` and the map positions from `qr_scanner` and `human_detector` need depth
**aligned to the colour image**, meaning the same pixel grid and intrinsics. By default the
robot's camera driver doesn't align it, so `/camera/depth/image_raw` has fx ≈ 373 and the
colour image has fx ≈ 462. Do this once per robot.

## Facts

| | |
|---|---|
| Camera | Orbbec **Gemini 335**, launched with `orbbec_camera gemini_330_series.launch.py` |
| Where the driver runs | The robot's *system computer*, `192.168.10.<60 + robot number>`. Use the **robot** number, not the group number (robot 2 → `.62`) |
| Login | User `user`. Get the password from the lab operator or the team chat |
| Autostart file | `~/.config/tmuxinator/lite3.yml` on that computer |
| Setting to change | `depth_registration:=true` (default `false` for this model) |

## Steps

1. **Log in** to the system computer:
   ```bash
   ssh user@192.168.10.<60 + robot number>
   ```
2. **Back up the autostart file, then find the camera launch in it:**
   ```bash
   cp ~/.config/tmuxinator/lite3.yml ~/.config/tmuxinator/lite3.yml.bak
   grep -n -i orbbec ~/.config/tmuxinator/lite3.yml
   ```
3. **Edit that line** to add the alignment arguments, keeping the arguments already there:
   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py <existing args> \
     depth_registration:=true align_target_stream:=COLOR
   ```
   Keep `align_mode` at its default (`SW`, done on the host). `HW` uses less CPU, but not
   every resolution supports it.
4. **Restart only the camera driver**, not the whole tmuxinator session. Restarting the
   session would also restart the robot bridge.
   ```bash
   tmux ls
   tmux attach -t <session>   # go to the camera window, Ctrl-C, run the new command
                              # detach with Ctrl-b d
   ```
   The edit to `lite3.yml` keeps the setting after a reboot. Running the command by hand only
   lasts until the next restart.
5. **Check from your own machine** (`ROS_DOMAIN_ID=4`):
   ```bash
   ros2 topic echo --once /camera/depth/camera_info --field k   # fx should be ~462, not ~373
   ros2 topic echo --once /camera/depth/image_raw --field header.frame_id
   ```
   If fx matches `/camera/color/camera_info`, depth is aligned.

## Afterwards

- If the aligned depth is on `/camera/depth/image_raw`, `robot.launch.py` works with its
  defaults. If it's on another topic, pass that topic as `depth_topic:=...`.
- **Undo:** `cp ~/.config/tmuxinator/lite3.yml.bak ~/.config/tmuxinator/lite3.yml`, then
  restart the camera driver as in step 4.
