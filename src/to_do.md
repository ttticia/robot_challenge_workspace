# stairs
first step height is only 10cm from the ground. Focus on identifying other steps to detect the stairs. suggest 5 alternative solutions to detect and align to the stairs

# maintenance
remove logs older than 1 week

# scene understanding
at junction (2 possible paths) robot should, align itself parallel to wall, turn multiple of 90 degrees to other path, wait 1s, then continue navigation

# curtains
if the wall infron has a gap at the bottom of <15cm and is wider than 0.7m, that walled area should be marked as a curtain. nav2 map should be updated so that wall should be overridden. 

approach the curtain at 70% the normal speed, walk straight through the curtain aiming for its middle for 20cm.
turn 20 degrees left, 40 degrees right, than 20 degrees left again. 

if the lidar or depth camera is blocked (curtain drapped over sensor)
1. creep forward 5cm and check if unblocked
2. if still blocked, do the 20 degree turn sequence again
3. alternate 1 and 2, checking after each step if still blocked untill not blocked anymore