# lite3_nav: Rey's navigation package

`lite3_nav` is **self-contained**: it does not depend on, import or launch the team's
`lite3_mission` package (which `origin/navigation` removes). It **never modifies**
`src/lite3_mission`, `src/team_perception` or any other team file. It contains:

- forked nodes `lite3_nav/mission.py` and `lite3_nav/velocity_gate.py` (each file starts with a
  header listing what changed against `lite3_mission` at main `e725625`);
- unchanged copies of the team's helpers and helper nodes from `lite3_mission` at `e725625`:
  `core.py` (frontiers, `Grid`, `VelocityLimiter`, ...), `runtime.py`, `vision.py`,
  `observations`, `qr`, `scene_client`, `team_qr_adapter`, and `config/mujoco.yaml`
  (each copy has a one-line provenance header; relative imports now resolve to lite3_nav);
- `lite3_nav/exit_detect.py` (exit / final cell) and `lite3_nav/wall_scan.py` +
  `lite3_nav/wall_band.py` (wall band scan), borrowed from Tricia's `maze_nav`
  (`origin/navigation` `419f205`) and adapted.

The nodes keep the node names (`mission`, `velocity_gate`, `observations`, ...) and the same topics
and services (`/mission/set_armed`, `/mission/gate_status`, `/mission/drive_lease`,
`/mission/nav_cmd_vel` -> `/cmd_vel`), so the team's tools and commands still work. Only the
package name in `ros2 launch` / `ros2 run` changes. Never run `lite3_nav`'s and
`lite3_mission`'s mission launches at the same time.

## Differences from lite3_mission

| Item | lite3_mission (main) | lite3_nav |
|---|---|---|
| `velocity_gate` `/cmd_vel` QoS | depth 1, RELIABLE, VOLATILE | `output_transient_local: true` (default): RELIABLE + TRANSIENT_LOCAL, lifespan = `command_timeout_sec`; needed by the Lite3 domain bridge's `/cmd_vel` subscription; still matches VOLATILE subscribers |
| `velocity_gate` input publishers | exactly 1 endpoint | `expected_input_publishers` (default 1, sim 2), counted per node |
| Gate status reason | `EXPECTED_ONE_NAV2_INPUT_PUBLISHER_GOT_<n>` | `EXPECTED_<k>_NAV2_INPUT_PUBLISHERS_GOT_<n>` |
| `mission` Nav2 liveness | lifecycle `get_state` only, 3 s hard-coded | `nav2_health_check`: `lifecycle` (code default) / `action_server` / `either` (robot.yaml and sim.yaml); `nav2_active_timeout_sec` (default 3.0, sim 10.0); fresh goal feedback always counts as active |
| Lost `get_state` response | blocks all later queries | dropped after 2 s and retried |
| Freshness clocks | wall clock | sim time under `use_sim_time`, wall clock otherwise (unchanged on the robot) |
| Mission tick / drive lease | node clock | steady clock (a slow simulator cannot starve the lease) |
| Launch profiles | `robot`, `mujoco` | `robot`, `mujoco` (team's mujoco.yaml), `sim` (config/sim.yaml) |
| Robot SLAM + Nav2 | lab launcher only | also `launch/robot_navigation.launch.py`: own slam_toolbox + Nav2 on the live map (below) |
| `mission` start not clear | `START_NOT_CLEAR` disarms | frontier search retried from the nearest clear cell within 0.6 m |
| End of run | none (explores until no frontier) | `EXITED` at the arena exit or the known final cell (below) |
| SLAM input on the robot | `/scan` | `/wall_scan` (wall band only, below); `wall_scan:=false` for `/scan` |

`config/robot.yaml` is the team's `robot.yaml` plus the new lite3_nav parameters (the four above and
the `exit_*` / `final_cell_*` ones); `test/test_package.py` checks every team value against the frozen
copy `test/team_robot_e725625.yaml`. `config/sim.yaml` holds the Gazebo overrides
(`use_sim_time`, relaxed timeouts, 0.25 m/s / 0.5 rad/s caps, 2 input publishers).
`config/fastdds_udp.xml` is a UDP-only Fast DDS profile used by the sim and robot terminals.

## Build

```bash
cd ~/challenge_ws/src
ln -s /mnt/c/GitHub/robot_challenge_workspace/src/lite3_nav .
cd ~/challenge_ws && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
colcon build --symlink-install --packages-select lite3_nav   # lite3_mission is not needed
source install/setup.bash
```

## Run in the simulator (domain 89, localhost only)

`sim_navigation.launch.py` needs **Rey's local `lite3_sim`** (in `~/ros2_ws`) with the
`nav_cmd_vel_topic` and `sensors` launch arguments and `lite3_sim_nav.urdf`. Those changes are
not in any git repository. Domain 4 is refused by the launch file.

```bash
source /usr/share/gazebo/setup.sh
ros2 launch lite3_nav sim_navigation.launch.py sensors:=nav rviz:=false
# another terminal, same overlays, ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1:
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: true}'
```

Options: `world:=...`, `gui:=true`, `sensors:=full` (camera on, needed for `team_qr:=true`),
`mission_delay:=25.0`, `config_file:=extra.yaml`.

## Run in the lab (real robot, domain 4)

Either the team's lab wrapper (needs the team's `lite3_mission` package built as well; it only remaps
the course launcher's `/cmd_vel` to `/mission/nav_cmd_vel`) with our mission launch in place of the
team's, or, without `lite3_mission`, `robot_navigation.launch.py` below:

```bash
source ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_env.sh   # domain 4, read-only check
bash ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_check.sh   # read-only rates and /cmd_vel endpoints
ros2 launch lite3_mission lab_navigation.launch.py slam:=True                  # only if the lab stack is not already running
ros2 launch lite3_nav mission.launch.py                                        # shadow/preview first
ros2 launch lite3_nav mission.launch.py mode:=explore execution:=true drive:=true
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: true}'
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: false}'
```

Follow the team's supervised execution procedure (`src/lite3_mission/README.md` on main).

### Live mapping + Nav2 on the robot: robot_navigation.launch.py

The Quickstart Guide reports that the lab launcher **does not accept Nav2 goals while mapping**
(`slam:=True`), and exploration needs both (its stage N1). `launch/robot_navigation.launch.py`
starts our own Nav2 on the live map instead. It follows the team's earlier hardware work on
`origin/navigation` (`src/archive`: conservative Nav2 profile, UDP-only Fast DDS, Fast-LIO's
`config_file` scope isolated, no voxel layer):

- `slam:=toolbox` (default): lite3_launcher's sensor bringup (`launch/include/lite3_bringup.launch.py`:
  Fast-LIO → `/odom` + `odom → base_link`, `/livox/points` → `/scan`; skip with `bringup:=false`
  if it already runs) + slam_toolbox online async (`config/slam_robot.yaml`) → `/map` + `map → odom`.
- `wall_scan:=true` (default, with `slam:=toolbox`): `wall_scan` turns `/livox/points` into
  `/wall_scan` and slam_toolbox maps from it instead of `/scan` (see "Wall band scan").
  `costmap_scan_topic:=/scan` (default) is what the Nav2 obstacle layers read; set `/wall_scan` if
  the arena walls are too low for `/scan`.
- `slam:=launcher`: something else already provides `/map` and `map → odom`; only Nav2 is added.
- Nav2 (`config/nav2_robot.yaml`, tuned for 0.9 m corridors with maze_nav's settings): no AMCL or
  map server; 20 m × 20 m rolling global costmap fed by the live `/map` (Humble's Smac crashed when
  planning from just off the map edge), 4 m rolling local costmap on `/scan`; frames
  `map`/`odom`/`base_link`; footprint 0.64 × 0.46 m including the moving feet, 5 cm behind
  base_link; inflation 0.3 m / cost scaling 6 (lethal within 0.23 m of a wall, zero cost from 0.3 m:
  a 0.44 m plannable lane with a 0.3 m zero-cost middle in a 0.9 m corridor; 0.45–0.55 m inflation
  blocked it); Smac 2D planner (`cost_travel_multiplier` 2 keeps to the corridor centre, plans
  through unknown space); Regulated Pure Pursuit with a 0.35 m lookahead, turning on the spot above
  0.6 rad; progress checker 0.15 m in 25 s; 0.1 m/s and 0.25 rad/s caps. Goals more than 10 m (x or
  y) from the robot lie outside the global costmap and fail to plan. Its output goes to
  `/mission/nav_cmd_vel` through node-scoped remaps (velocity_smoother + behavior_server), never `/cmd_vel`.
- Safety guards: `robot:=true` (default) refuses to start unless `ROS_DOMAIN_ID=4`; `robot:=false`
  refuses domain 4; domain 89 (the simulator) is always refused. It also refuses to start while
  another SLAM/Nav2 stack is running on the domain (`check_graph:=false` skips that check).
- The mission must expect **two** Nav2 input publishers: pass `config/robot_own_nav2.yaml`.

In the lab (instead of `lab_navigation.launch.py`; stop any running lab stack first):

```bash
source ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_env.sh   # every terminal: domain 4, UDP-only
ros2 launch lite3_nav robot_navigation.launch.py                               # terminal 1: Fast-LIO + /scan + SLAM + Nav2
ros2 launch lite3_nav mission.launch.py mode:=explore \
  config_file:=$(ros2 pkg prefix lite3_nav)/share/lite3_nav/config/robot_own_nav2.yaml      # terminal 2: shadow first
ros2 launch lite3_nav mission.launch.py mode:=explore execution:=true drive:=true \
  config_file:=$(ros2 pkg prefix lite3_nav)/share/lite3_nav/config/robot_own_nav2.yaml      # then supervised
```

Verified offline only (2026-11-14, WSL, loopback domain 77, `robot:=false bringup:=false` with
`test/fake_robot_world.py` standing in for the sensors): Nav2 lifecycle active after ~10 s,
slam_toolbox publishes `/map`, a NavigateToPose goal is accepted and Nav2 streams commands capped at
0.1 m/s on `/mission/nav_cmd_vel` only, nothing publishes `/cmd_vel` until the gate runs, and the
lite3_nav gate with `robot_own_nav2.yaml` reports `READY` with the gate as the only
(TRANSIENT_LOCAL) `/cmd_vel` publisher. **Not verified:** lite3_launcher's bringup include path and
arguments on the robot, Fast-LIO + slam_toolbox map quality, Nav2 timing over the robot network,
and real driving.

2026-11-14, after the maze_nav changes (loopback domain 88, `robot:=false bringup:=false`, fake
world): with `wall_scan:=false` Nav2 becomes active with Smac 2D and the rolling global costmap and a
`ComputePathToPose` request succeeds; with `wall_scan:=true` (default) the `wall_scan` node starts and
slam_toolbox reads `/wall_scan`, but the fake world has no `/livox/points`, so no map is built there.

Offline dry run (any machine, never on domains 4 or 89):

```bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
python3 src/lite3_nav/test/fake_robot_world.py &
ros2 launch lite3_nav robot_navigation.launch.py robot:=false bringup:=false
```

## Exit / final cell (end of the run)

The organisers end the run when the robot reaches the final grid cell, which has an opening out of
the arena. `lite3_nav/exit_detect.py` (maze_nav's `ExitDetector`, adapted to rotated maps) checks at
2 Hz in `explore` mode: a disk of radius `exit_open_radius` (1.1 m) made only of SLAM-seen free floor
(walls and unknown both count as blocked) lies within `exit_near_radius` (0.8 m) of the robot and at
least `exit_min_displacement` (1.5 m) from where the robot was first armed, the robot has been in a
corridor once (clearance ≤ `exit_enclosed_radius`, 0.7 m), and this holds for `exit_dwell_sec`
(2 s). 0.9 m corridors allow at most ~0.45 m, so such a disk only exists outside the arena.
In practice this fires once the robot is about 0.3 m out through the opening.

If the final cell's map coordinates are known, set `final_cell_xy: [x, y]` (map frame; empty by
default) and `final_cell_radius` (0.4 m): being that close also ends the run, at once and in any mode.

On exit: the goal is cancelled, the drive lease drops (the gate stops the robot, which holds
position), the mission disarms with state `EXITED` (`SHADOW_EXITED` in shadow mode), and no further
goals are chosen. Arming is refused until relaunch or a `mode`/`waypoints` change. `/mission/status`
carries `exited` and `exit_reason` (`OPEN_FLOOR` or `FINAL_CELL`).

Frontiers on the open floor beyond the exit: with `exit_frontiers_last: true` (default) frontier
selection runs on a copy of the map where those open-floor disks are marked occupied, so the arena is
explored first; only when nothing else is left does it choose a frontier out there (state
`EXIT_FRONTIER`), which leads the robot out and ends the run. `exit_detection_enabled: false` turns
the open-floor test and the mask off (`final_cell_xy` still applies).

## Wall band scan

`lite3_nav/wall_scan.py` (maze_nav's `wall_scan`): each `/livox/points` cloud is transformed into
`odom` (gravity aligned, Fast-LIO), only points `band_low`..`band_high` (0.30..0.43 m) above the
floor are kept and binned by bearing into a 360° `LaserScan` on `/wall_scan` (frame `base_link`).
Terrain (ramps, mats ≤ 0.15 m), stair treads (≤ 0.26 m) and the floor stay out; walls and hand rails
stay in. lite3_launcher's `/scan` keeps 0.1..0.5 m above base_link, about 0.45..0.85 m above the
floor, and its band moves with the body on ramps. The floor height is measured once at startup from
the cloud around the robot (lowest dense layer near 0.35 m below base_link), so **the robot must be
standing when `robot_navigation.launch.py` starts**, or pass `floor_z:=<odom z>`. Launch arguments:
`wall_scan`, `band_low`, `band_high`, `floor_z`. `band_high` must stay below the arena's wall top
(unknown yet: check on the robot). `sim_navigation.launch.py` is unchanged (still `/scan`).

## Curtains (planned)

Exploration currently stops at curtains: the lidar and SLAM see a curtain as a wall, so rooms behind
one never become frontiers (in arena_1 the mission explored 4 of 12 cells, then reported
`NO_REACHABLE_FRONTIER`). The occupancy map alone cannot tell a curtain from a wall.

Plan, to be developed during real-robot testing: use the robot's depth camera to look under the
curtain's bottom gap (real curtains most likely do not reach the floor) and confirm open floor behind
it. A fan that makes curtains move is an alternative. Then: Nav2 to a staging pose in front of the
curtain, a slow straight push through, and resume exploration.

Viewing distance (**estimate**, to check on the robot): camera at base_link → camera_link ≈
(0.252, 0, 0.074) m, pitched 20° down; base_link ≈ 0.35 m above the floor when standing (manufacturer
value in robot_info.md), so camera height h ≈ 0.42 m. A ray that just clears a gap of height g at
distance d lands on the floor d·g/(h−g) behind the curtain, so to see floor at least s behind it the
camera must be at least d ≥ s·(h−g)/g in front. For the cases below the needed rays are 11–45° below
horizontal, inside the view of a camera pitched 20° down (sim camera: ±27° vertical, i.e. 7° above
to 47° below horizontal; the real camera's vertical field of view is not yet checked).

| Gap g | See 0.2 m behind | See 0.5 m behind |
|---|---|---|
| 0.10 m | d ≥ 0.65 m | d ≥ 1.62 m |
| 0.15 m | d ≥ 0.37 m | d ≥ 0.91 m |
| 0.20 m | d ≥ 0.22 m (camera minimum range applies) | d ≥ 0.56 m |

So a staging pose about 0.7–1.6 m in front of the curtain should show floor under a 10 cm gap.
Depth noise at the floor edge, the real standing height and the real camera's vertical field of view
are not yet verified. `docs/REAL_ROBOT_TESTING.md` stage 10 records the data to check this.

Groundwork already in place: the velocity gate's push input (`push_input_enabled`,
`/mission/push_cmd_vel`, `/mission/push_lease`, straight only, at most `max_push_linear_m_s` = 0.1,
never mixed with Nav2 commands; see the header of `lite3_nav/velocity_gate.py`). It stays
**`push_input_enabled: false`** (the default; set in neither robot.yaml nor sim.yaml) until the
detection and push behaviour exist and have been tested.

## Tests

```bash
bash src/lite3_nav/test/run_tests.sh   # static checks + ROS contract tests on loopback domain 88 (safe while a sim runs on 89)
```
