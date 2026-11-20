## stairs
reduce stair detection CPU usage. 

stairs were missed in 2nd last run. need to make stair check more robust. 

## curtains
some walls have a cavity that could be read as curtain gap. when checking for curtain look for curtain gap that extends >10cm behind the supposed curtain.

## turning
Costmap and path planning behaviour needs to be tuned. need to run with ros-bag and make turning avoid inner wall without inflating costmap too much.

## Navigation

fast-lio delay
Frequently running into issues where map is not updating fast enough. Look into
1. how to change launch stack to accommodate for buffer/loading time
2. make map loading independent of time (possibly leveraging laplace transform)
3. does running robot too long/overheating cause issues? add imu hz check to gauge if robot needs cooldown

robot not reliably sensing walls
check new map. identify why robot not detecting walls properly. is it linked to map delay?


## scanning
when scanning after launch and at a dead end/room, go into pose mode, yaw left and right.
Implemented (start scan + dead-end side walls; QR-room look-around still uses Nav2 spins). NEEDS TESTING ON ROBOT:
1. does the group bridge pass the yaw axis value (`size`) through?
2. does Pose mode work from auto mode (pose_manual:=false), or only via manual?
3. measure body yaw at full axis (log prints what /odom saw) and set pose_max_yaw_deg (currently 20)
4. check it returns to Move mode + gait and Nav2 velocity resumes after the look


## Perception
1. QR code reading always on, but QR detection and survivor detection only at dead ends/rooms. 
2. check feasibility of using yolo for stair and curtain detection. should be able to detect at 2-0.5m away


## obstacles
evaluate feasibility of adjusting gait based on obstacle height/terrain type. Some terrains, like slats or barriers, work better with rug mode h-step.
Obstacles up to 15 cm (checked 2026-11-20): nav already treats them as walkable (wall band starts 0.30 m above floor), stair detector needs >= 2 risers so a single 15 cm block/barrier is not stairs. Open: does RUG general gait clear 15 cm? (foot step height unmeasured, robot_info.md open item 7); H-Step made the robot unsteady 2026-11-12. Test on a 15 cm block at max_speed before relying on it.