# lite3_nav: Rey's navigation workstation package

`lite3_nav` holds our navigation changes as a separate package on top of the team's
`lite3_mission`. It **never modifies** `src/lite3_mission`, `src/team_perception` or any other
team file. Instead it:

- imports the team's helpers unchanged (`from lite3_mission.core import ...`);
- ships forked copies of two nodes, `lite3_nav/mission.py` and `lite3_nav/velocity_gate.py`
  (each file starts with a header listing what changed against `lite3_mission` at main `e725625`);
- launches the team's `observations`, `qr`, `scene_client` and `team_qr_adapter` executables
  from `lite3_mission` as they are.

The forked nodes keep the node names `mission` and `velocity_gate` and the same topics and
services (`/mission/set_armed`, `/mission/gate_status`, `/mission/drive_lease`,
`/mission/nav_cmd_vel` -> `/cmd_vel`), so the team's tools and commands still work. Only the
package name in `ros2 launch` / `ros2 run` changes. Run either `lite3_nav` or `lite3_mission`'s
mission launch at one time, never both.

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

`config/robot.yaml` is the team's `robot.yaml` with only the four new parameters added
(`test/test_package.py` checks this). `config/sim.yaml` holds the Gazebo overrides
(`use_sim_time`, relaxed timeouts, 0.25 m/s / 0.5 rad/s caps, 2 input publishers).
`config/fastdds_udp.xml` is a UDP-only Fast DDS profile used by the sim and robot terminals.

## Build

```bash
cd ~/challenge_ws/src
ln -s /mnt/c/GitHub/robot_challenge_workspace/src/lite3_mission .   # team package, unchanged
ln -s /mnt/c/GitHub/robot_challenge_workspace/src/lite3_nav .
cd ~/challenge_ws && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
colcon build --symlink-install --packages-select lite3_mission lite3_nav
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

Use the team's lab wrapper by reference (unchanged; it only remaps the course launcher's
`/cmd_vel` to `/mission/nav_cmd_vel`), and our mission launch in place of the team's:

```bash
source ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_env.sh   # domain 4, read-only check
bash ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_check.sh   # read-only rates and /cmd_vel endpoints
ros2 launch lite3_mission lab_navigation.launch.py slam:=True                  # only if the lab stack is not already running
ros2 launch lite3_nav mission.launch.py                                        # shadow/preview first
ros2 launch lite3_nav mission.launch.py mode:=explore execution:=true drive:=true
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: true}'
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: false}'
```

Follow the team's supervised execution procedure in `src/lite3_mission/README.md`.

## Curtains (planned)

Exploration currently stops at curtains: the lidar and SLAM see a curtain as a wall, so rooms behind
one never become frontiers (in arena_1 the mission explored 4 of 12 cells, then reported
`NO_REACHABLE_FRONTIER`). The occupancy map alone cannot tell a curtain from a wall.

Plan, to be developed during real-robot testing: use the robot's depth camera to look under the
curtain's bottom gap (real curtains most likely do not reach the floor) and confirm open floor behind
it. A fan that makes curtains move is an alternative. Then: Nav2 to a staging pose in front of the
curtain, a slow straight push through, and resume exploration.

Groundwork already in place: the velocity gate's push input (`push_input_enabled`,
`/mission/push_cmd_vel`, `/mission/push_lease`, straight only, at most `max_push_linear_m_s` = 0.1,
never mixed with Nav2 commands; see the header of `lite3_nav/velocity_gate.py`). It stays
**`push_input_enabled: false`** (the default; set in neither robot.yaml nor sim.yaml) until the
detection and push behaviour exist and have been tested.

## Tests

```bash
bash src/lite3_nav/test/run_tests.sh   # static checks + ROS contract tests on loopback domain 88 (safe while a sim runs on 89)
```
