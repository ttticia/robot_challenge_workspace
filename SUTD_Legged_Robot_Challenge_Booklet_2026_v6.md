## Legged Robot Challenge Post-Earthquake Search & Rescue

Competition Challenge Booklet SUTD | EPD

2026

## Contents


## Singapore University of Technology & Design Engineering Product Development Robotics Intelligence

## A. Challenge Introduction

A major earthquake has struck a building, leaving parts of the interior damaged and difficult to access. A rescue team must assess the environment, locate survivors, identify hazards, and deliver supplies to an isolated area. Your task is to develop an autonomous legged-robot system that can assist with this response.

The ground robot begins at a designated Start Area. It must explore the unfamiliar environment, interpret visual and spatial cues, navigate obstructed passages and three- dimensional terrain, complete mission tasks, and reach the Drone Launch Area. The overall challenge also envisions aerial delivery beyond terrain unsuitable for ground robots.

## B. Challenge Objectives

The challenges include to:

- Develop autonomous exploration and navigation without relying on a pre-built arena map.

- Navigate uneven terrain and stairs using a legged robot

- Apply perception and scene understanding to identify rooms, objects, hazards, and mission-relevant locations.

- Integrate sensing, decision-making, and robot actions through a robust ROS 2-based system, as well as to demonstrate coordinated ground and aerial robotic capabilities


## C. Mission and Competition Arena

The arena represents a damaged indoor environment (a shopping mall) with rooms, corridors, obstacles, survivors, terrain sections, stairs and an exit leading to a drone launch zone. Objects may be represented using miniature or toy props, printed graphics, or simulated features. Interpret them as the corresponding real-world objects or locations for the mission. The competition will be held in SUTD Campus Centre, including an allocated area (Preparation Area) for student groups to develop their robots in preparation for the competition will be available for use throughout the competition week (23-27 November 2026). A Pre-Competition Practice Setup will also be available in the Ground Robot Arena and an Aerial target for groups to test their algorithms for both the legged robot and drone respectively. Figure 1 shows the competition layout, as well as an example illustration for the Ground Robot Arena.

*Figure 1. Competition layout in SUTD Campus Centre and an illustrative ground robot arena*

*layout (expect changes for the final competition layout).*

The actual competition will be held on 27 November 2026. On the morning of the Competition Day, the Ground Robot Arena will be modified with a new layout to ensure that teams cannot rely on pre-defined sequences for their autonomous robot run.

All teams are to use preparation area for robot development. On competition day, teams will wait in the preparation area while waiting for their run, with two teams having their sessions in the two Arenas and the next two teams waiting in the Standby area.


## D. Challenge Components

The following describes key components of the challenge.

## 1. Autonomous Exploration and Navigation

The robot should build an understanding of its surroundings while moving through the environment and select appropriate paths towards mission objectives. Teams are encouraged to integrate localization, mapping, planning and obstacle avoidance, and to adapt standard navigation stacks where required.

## 2. Uneven Terrain

Traverse designated disaster or rubble-like cells. These sections test stability, terrain handling, and the ability to integrate locomotion with navigation. A successful traversal should be controlled and avoid substantial contact with barriers or props.


## 3. Stairs

Locate the staircase and negotiate it as part of the route. The challenge aims to extend beyond conventional flat-floor, two-dimensional navigation. Teams may need to modify navigation behaviour, change locomotion modes, or add custom logic suited to a legged platform.

## 4. Movable Obstacles

The environment may contain hanging, movable obstacles such as curtains. These are typically perceived as standard obstacles that cause standard 2D navigation to fail. Teams will have to develop intelligent perception and navigation strategies to differentiate between rigid and traversable obstacles, enabling the robot to push through passable barriers.

Overhead obstacles such as curtains will be mounted higher than the robot (about 1m) so that the robot will not collide with the mounting interface.


## 5. Scene Understanding

The robot is required to perform various scene understanding tasks, such as to infer the type of scene, detect the type of terrain, or look for object(s) of interest relevant to the given mission tasks.

One of the tasks may also involve hazard detection and identification. Here are examples of hazard symbols that may be included in the scenes:

They could be pasted on the wall, a cabinet-prop, or an object such as a container or a bottle. The objective is to detect and identify these hazard(s) and report them.

Refer to section F for more examples of mission tasks and scoring.


## 6. Survivor Search and Localization

Find survivors using the supplied cues and information observed during exploration. The concept may include a survivor interaction that yields a cue (QR code) and other survivors that do not provide cues. A cue may refer to the type of room containing other survivor(s) rather than its coordinates. All survivor tasks require reporting their location.

## QR Code Placement and Size

QR codes may be placed on the left or right walls of rooms that contain survivor(s), simulating clues given by survivors. They will not be placed along generic walls (e.g., normal corridors).

QR codes will be approximately 12 × 12 cm in size.

## 7. Drone Mounting

Teams are allowed to remove the drone during the ground arena run and remount it on the robot at the final cell before launching. However, teams that successfully keep their drone securely mounted on the robot throughout the entire ground arena run, until drone launch, will receive bonus points.

You may also attempt the ground arena with the drone mounted initially. If it becomes detached or falls off during the run, you may remove it and continue the ground task normally, but you will forgo the bonus points.


## 8. Exit and Drone Launch Area

The final cell of the ground robot arena also functions as the Drone Launch Area. Teams must launch a drone towards a distant target to supply materials across a zone inaccessible or unstable for ground robots.

There will be an opening where the robot has line-of-sight with the target that the drone should be launched towards (essentially the wall facing/nearest to the target will not be installed). Teams are awarded points if their robot autonomously stop (such as by using perceptive cues) within the final cell. Stopping within the final cell refers to all four legs of the quadruped contained within the cell and no part of the leg should be touching the frame or protruding from the frame along the cell border.

You can then teleoperate your robot to reposition or orientate it within the final cell to prepare for launching the drone towards the distant target. Drone launching must be manually triggered in case two teams reach the end point at a similar time and launching autonomously at the same time may result in a drone collision

## 9. Drone Launching

Once the robot has reached the final cell (the Drone Launch Area), the team may launch its drone towards the distant target. The drone must be guided to the target automatically by the team’s visual-servo control system, using the standardised camera and Raspberry Pi 5 + AI HAT mounted on the robot. No human flight commands are allowed after launch.

Each team may launch up to three times within its run time. Each flight is scored by where the drone first touches the ground on the Drone Landing Field. The team’s drone score is the average of its two highest-scoring flights, which reduces the effect of a single unlucky (or lucky) flight.

## Drone Landing Field

The Drone Landing Field is a fan-shaped area that extends from the two Launch Boxes towards the target and is enclosed by the field boundary. It is divided by distance from the launch line into four bands. Each band is further divided into the corridor leading toward the target and the outer (off-line) regions on either side. Points are roughly doubled for each band closer to the target, and landing in the corridor always scores more than landing outside it at the same distance.


| Distance band (from launch line) | Inside corridor | Outside corridor |
| --- | --- | --- |
| Band 1 (nearest the launch line) | 5 | 3 |
| Band 2 | 10 | 7 |
| Band 3 | 20 | 7 |
| Band 4 (target zone) | 50 | 14 |
| Direct hit on target | 50 + 30 hit bonus | – |
| First contact outside the field boundary | 0 |   |

Table: Points per flight, based on the first ground contact.

## Calculating the drone score

For each flight, the flight score is the landing points plus the hit bonus (30 for a direct hit, otherwise 0), so a flight scores between 0 and 80. The two highest flight scores are averaged.


## E. Task Reporting Interface and Scoring

Teams are required to submit their mission results to the competition scoring server using HTTP POST requests with a standardized JSON format.

The scoring server will receive and validate submissions, associate them with the corresponding group and task, and record the results for automated scoring.

Teams should submit a result when a mission task has been completed or when an existing result needs to be updated. Only the latest valid submission for each task will be considered for scoring.

- Each group must select a unique group identifier in the format group##_XXXX, where ## is the group number and XXXX is a four-character alphanumeric code (e.g., group01_A7K2).

- The same identifier must be used for all requests to the AI scene-understanding server and for submitting competition task results to the scoring server. Groups should register their chosen identifier with the organizers and keep it unchanged throughout the competition.

- The group identifier allows the servers to associate requests and task submissions with the correct team. As introduced in the scene-understanding lab, the AI server also uses this identifier to manage requests from different groups.

Where appropriate, we will provide a predefined list of possible categories or answers (e.g., task types and scene types) so that your results can be evaluated objectively.

## Competition task reporting interface

| Server address | http://192.168.10.141 |
| --- | --- |
| API endpoint | /task_report |
| HTTP method | POST |
| Request format | JSON (application/json) |
| Group identification | group##_XXXX (same as the scene- understanding server) |
| Submission | One HTTP request per task result |
| Server response | JSON acknowledgement indicating acceptance or rejection |

Check health (from terminal):

curl http://192.168.10.141:8000/health

URL for task reporting:

http://192.168.10.141:8001/task_report


## JSON Message Structure

```
{
"group_id": "group01_A7K2",
"task_id": "M01",
"task_type": "object",
"scene_type": "clinic",
"label": "medical_equipment",
"pos": [3.06, 2.04],
"message": ""
}
```

## Field Definitions

| Key | Data type | Description |
| --- | --- | --- |
| group_id | string | Team ID and pin (group##_XXXX) |
| task_id | string | Predefined mission identifier |
| task_type | string | object, hazard, survivor, qr |
| scene_type | string | Identified scene category |
| label | string | Object or hazard identifier, if applicable |
| pos | array [x, y] or null | Estimated centre position of the relevant room/arena cell |
| message | string | Decoded QR message, if applicable |


## Sample task report (from terminal)

Note to update group_id ("YOUR_FULL_ID") with the same one used for querying the perception server (including 4-char-pin):

```
curl -X POST http://192.168.10.141:8001/task_report \
-H 'Content-Type: application/json' \
-d
'{"group_id":"YOUR_FULL_ID","task_id":"M01","task_type":"object","scene_type":"cli
nic","label":"medical_equipment","pos":[3.06,2.04],"message":""}'
```

## HTTP response format

\* Note that it is intentional for group_id in the response to exclude the four-character alphanumeric PIN

## For a successful request

```
{
"status": "accepted",
"group_id": "group01",
"task_id": "M01",
"message": "Submission recorded"
}
```

## For an unsuccessful request

```
{
"status": "rejected",
"group_id": "group01",
"task_id": "M01",
"reason": "Invalid scene_type"
}
```


## Keys with fixed vocabulary

(TAKE NOTE OF THE EVALUATED FIELDS FOR EACH task_type)

| Key | Values | Evaluated fields |
| --- | --- | --- |
| group_id | group##_xxxx | - |
| task_type | object hazard survivor qr | label, scene_type, pos label, scene_type, pos scene_type, pos message |
| scene_type | clothes_shop fitness_centre kitchen market workshop clinic cleaning_room furniture_shop | - |
| label | medical_equipment hand_tools food water utensils high_voltage radioactive chemical biological flammable | - |


## Example JSON by mission (carefully note all relevant keys for each mission)

| task_id | Illustrative Mission Tasks | Result Message |
| --- | --- | --- |
| M01 | The rescue team needs to provide urgent care to injured survivors. Find a location where medical equipment may be available. | { "group_id": "group01_A7K2", "task_id": "M01", "task_type": "object", "scene_type": "clinic", "label": "medical_equipment", "pos": [3.06, 2.04], "message": "" } |
| M02 | A damaged electrical installation has been reported. Find the affected area and identify the warning sign displayed nearby | { "group_id": "group01_A7K2", "task_id": "M02", "task_type": "hazard", "scene_type": "kitchen", "label": "high_voltage", "pos": [4.08, 2.04], "message": "" } |
| M03 | A survivor is believed to be in the right side of the room for food preparation. Locate the room, find the survivor, and report the survivor’s location. Find all other survivors | { "group_id": "group01_A7K2", "task_id": "M03", "task_type": "survivor", "scene_type": "kitchen", "label": "", "pos": [2.04, 3.06], "message": "" } |
| M04 | The first survivor may be able to share useful information. Gather and report this information | { "group_id": "group01_A7K2", "task_id": "M04", "task_type": "qr", "scene_type": "kitchen", "label": "", "pos": null, "message": "My friend is in the market." } |
| M05 | Locate and report the location of another survivor | { "group_id": "group01_A7K2", "task_id": "M05", "task_type": "survivor", "scene_type": "market", "label": "", "pos": [3.06, 3.06], "message": "" } |


## Scene understanding scoring rules

- Each mission task_id corresponds to one specific task or target, as defined and communicated by the organizers before the competition. For example, if there are two survivors to identify, two separate mission IDs will be assigned, one for each survivor.

- Mission types and task_id are fixed to standardise scoring.

- For M01, M02, M03 and M05, up to 10 points may be awarded per task:

- o 5 points: Correct identification of the relevant object/hazard (label) and scene_type for M01/M02, or correct scene_type for M03/M05 (survivors).

- o 5 points: Correct localization (pos), provided the relevant object/hazard or scene type is correctly identified.

- M04 is evaluated independently based on the decoded QR message.

- All reported coordinates must follow the organizer-defined arena coordinate system. Localization will be evaluated at room/cell level using organizer-defined acceptance criteria.

- A location is considered correct if its reported coordinates fall within 0.5 m of the ground-truth cell centre along both the X and Y axes.

- Teams may submit multiple results for the same task (task_id) during their run. Only the latest valid submission for each task will be evaluated for scoring, replacing any previous submission for that task. Submissions received after the competition run ends will not be considered.

- A valid submission refers to a correctly formatted message with permitted field values, regardless of whether the reported result is correct.

## Localization Reference Frame

- For localization tasks, all reported positions must use a common arena-relative coordinate system.

- o The origin (0, 0) is defined at the centre of the robot's starting cell.

- o The positive X-axis points in the robot's designated initial forward direction.

- o The positive Y-axis points to the robot's left at the start.

- o Adjacent maze cell centres are separated by approximately 1.02 m, accounting for the 1.00 m wall sections and 0.02 m posts.

- o The reported pos: [x, y] should represent the estimated centre of the cell containing the relevant scene, object, hazard or survivor.

- Both competition arenas will follow the same starting-pose convention, Teams should initialize their robot's localization system using the marked starting position and orientation.

- Localization scores will be evaluated against the ground-truth cell positions for the respective arena, using the specified localization acceptance criteria.


## Example arena coordinate convention


## F. Challenge Guidelines

## Ground Robot Rules and Guidelines

## General

- 1. All robots are required to connect to the RI_network for scoring and querying the server for scene-understanding.

- 2. Each team must assign a team Captain who makes final decisions for the team and communicate with the marshal. Key decisions include whether to return to manual mode for manual teleoperation past an obstacle.

- 3. Each team must assign a robot operator who will be allowed to handle the robot’s controller and be ready to press the STOP button if the robot goes out of control.

- 4. If the robot fails to autonomously overcome an obstacle, the team is allowed to switch to manual teleoperation to move past the obstacle, but no points will be award for clearing that obstacle. Only the team Captain can make this decision and must involve the Marshal before switching, as well as when the robot is returning to autonomous mode.

- 5. Teams are encouraged to be familiar with quickly switching their robot between manual and autonomous modes.

- 6. AI gait mode is only allowed for overcoming the stairs obstacle and all teams should use this AI mode to overcome the stairs for the robot’s safety. Teams are allowed to ENTER AI mode 1 cell before the stairs, and EXIT AI mode 1 cell after the stairs.

- 7. No points will be awarded for using AI mode to overcome any other cells or terrains.

## Mapping, Autonomy and Mission Tasks

- 8. The robot starts from a defined Start Area and must autonomously explore to discover routes and targets.

- 9. Pre-mapping the competition environment is not allowed; autonomous exploration during the attempt is compulsory.

- 10. Some regions contain uneven terrain representing earthquake damage; stair obstacles require three-dimensional mobility.

- 11. Mission tasks (relevant to scene understanding) will be provided before the start of the competition. They may specify a room or describe objects rather than providing explicit coordinates.

## Navigation and Safety

- 12. Keep the robot within the designated arena and avoid striking walls, obstacles, props, spectators or other equipment.

- 13. Major collisions are intended to incur a penalty; judging thresholds will be defined in the final rules.

- 14. Use safe gait selection, obstacle handling and appropriate stop/recovery behaviour on difficult terrain.

- 15. Stop immediately and follow the marshals’ instructions if the robot behaves unsafely.

Perception Tasks


- 16. Only report a detected room, object, survivor or hazard when supported by sensor observations.

- 17. Design the robot to recover gracefully if the scene-understanding service is delayed or returns an unusable response.

## Definitions

- Autonomous Navigation

- o Robot navigates independently without human control.

- Manual Teleoperation

- o Operator remotely controls the robot's movement using a controller or software interface.

- Physical Repositioning

- o The team physically lifts or moves the robot to the nearest designated safe position when recovery is necessary.

## Terrain-based obstacles (5 points each)

- Fully autonomous traversal: 5 points.

- Traversal using manual intervention (manual teleoperation or physical repositioning): 0 points.

- If an obstacle was previously traversed autonomously, its 5 points are retained even if manual intervention is subsequently required to backtrack across it.

- Each obstacle is scored once per attempt, based on the highest level of successful traversal demonstrated.

## Scene-understanding and mission tasks (10 points each)

- Correct completion after autonomous discovery and approach: 10 points.

- Correct completion where manual intervention directly assists the robot in reaching a position from which the mission scene can be observed: 8 points.

- Incorrect mission response: 0 points, regardless of assistance level.

- Manual intervention does not automatically reduce the scores of subsequent missions that the robot independently discovers and approaches autonomously.

## Autonomy Bonus

- The autonomy bonus is awarded when the robot successfully reaches the final cell, based on the number of manual interventions used within that attempt.

## General Rules Regarding Restarts and Manual Intervention

- 1. Physical repositioning is strictly for recovery. All physical repositioning requires judge approval, limited to a specific obstacle or stuck condition. Slow exploration, entering a dead end, or needing to backtrack does not, by itself, justify physical repositioning.

- 2. Manual teleoperation and physical repositioning are treated equally as manual interventions. Each approved recovery action counts as one intervention toward the autonomy bonus.

- 3. Terrain and scene-understanding scores are evaluated independently. Manual intervention affects only the relevant obstacle or mission and does not automatically reduce the scores of subsequent independently completed tasks.

- 4. Scene-understanding assistance is determined by how the robot reaches the observation position. If manual intervention brings the robot into a position from


- which a mission scene can be directly observed, that mission is capped at 8 points, even if scene recognition is subsequently performed autonomously.

- 5. Subsequent autonomous scene discovery remains eligible for full points. For example, if the robot is manually assisted past an obstacle, then autonomously navigates through at least one cell and discovers another scene, that mission remains eligible for 10 points.

- 6. Manual repositioning is limited to the immediate recovery area, typically one cell away, such as the nearest designated safe cell beyond an obstacle. Repositioning directly to a mission target or final cell for the purpose of gaining points is prohibited.

- 7. Previously earned obstacle points are retained within the same attempt. For example, if the robot autonomously traverses an obstacle but later requires manual assistance to backtrack across it, the original 5 points remain valid.

- 8. Each restart begins a new independent attempt, with zero points, zero manual interventions, and cleared mission and environment-specific state. Scores, assistance classifications and discovered maps or routes do not carry over between attempts.

- 9. The team's final ground score is the highest total score achieved in a single valid attempt, including terrain points, mission points and any autonomy bonus earned.

- 10. Ground completion time is recorded when the robot validly reaches the final cell. Drone-launching time is excluded from the recorded ground completion time.

- 11. Completion time is used only as a tiebreaker, measured using the overall competition-session clock, which continues running across restarts. No completion- time bonus points are awarded.

- 12. Physical repositioning directly to the final cell does not qualify as valid ground completion. Valid completion requires the robot to navigate into the final cell rather than being physically placed there.

- 13. All interventions, scoring classifications and completion events are subject to judge verification and will be recorded in the competition scoring system.


## Drone Mission Rules and Guidelines

- 1. Launch condition. The drone may only be launched from the team’s Launch Box after the robot has reached the final cell of the Ground Robot Arena. Teams that used manual teleoperation during the ground run may still launch and score the drone component.

- 2. Launch trigger. Each launch is triggered manually by the team on the Marshal’s signal. The Marshal will stagger launches between the two arenas so that two drones are not in the air over the shared landing field at the same time.

- 3. Autonomous guidance. After launch, the drone must be guided only by the team’s automatic control system. Any human flight command or manual input after launch makes that flight score 0.

- 4. Number of attempts. Each team may make up to three launches within its run time. Every launch counts as an attempt, including a launch in which the drone fails to fly, which scores 0. The run timer continues during all launches.

- 5. Drone retrieval and spares. Team members must not enter the Drone Landing Field. Drones will be retrieved by the organisers between launches. Teams may prepare spare airframes in advance to save time.

- 6. Scoring point. Each flight is scored at the drone’s first contact with the ground; any part of the airframe touching the ground counts as contact. Where the drone lands after first contact does not affect the score. A first contact on a line between zones scores the higher zone. A first contact outside the field boundary scores 0.

- 7. Direct hit. A direct hit is a first ground contact inside the target circle. A direct hit scores 50 landing points plus the 30-point hit bonus for that flight.

- 8. Safety. A drone that strikes a person or a structure before touching the ground scores 0 for that flight. Teams must follow the Marshal’s instructions at all times, and the Marshal may stop launches if conditions are unsafe.

- 9. Interference. If drones from the two arenas collide, or one interferes with the other in flight, the Marshal may award a re-flight to the affected team.

- 10. Judging and video. Judge’s decision is final.

- 11. Launch-related bonuses. The bonuses for autonomous re-positioning and orientation, and for keeping the drone securely mounted, are assessed on the first launch only.

- 12. Tie-breakers. Between teams with equal total scores, the drone tie-breakers are, in order: the score of the third (unused) flight, then the distance from the target centre to the first ground contact of the team’s best flight.


## G. Evaluation and Scoring

## Task Completion

‘Auto’ and ‘Manual’ points refer to completing the task autonomously or with manual intervention, as defined in

| Task | Points (Auto) | Points (Manual) | Occur -rence | Subtotal (Auto) |
| --- | --- | --- | --- | --- |
| Autonomously navigate past each uneven-terrain cell 5 |   | 0 | 4 | 20 |
| Autonomously cross the stairs | 10 | 0 | 1 | 10 |
| Autonomously pass through movable obstacle | 15 | 0 | 1 | 15 |
| M01 — Identify the correct object and its location | 10 | 8 | 1 | 10 |
| M02 — Identify the correct hazard and location | 10 | 8 | 1 | 10 |
| M03 — Report first survivor's location | 10 | 8 | 1 | 10 |
| M04 — Decode and report QR code | 5 | 4 | 1 | 5 |
| M05 — Report second survivor's location | 10 | 8 | 1 | 10 |
| Autonomously stop within the final cell* | 10 | 0 | 1 | 10 |
| Drone landing score (average of the best 2 of 3 flights; see Drone Landing Field) | 50 | - | 1 | 50 |
|   |   |   | Total | 150 |

## Bonuses and Penalties

| Item | Points |
| --- | --- |
| Autonomy Bonus (up to 1 manual intervention**) | +20 |
| Autonomy Bonus (up to 2 manual intervention**) | +10 |
| Autonomy Bonus (up to 3 manual intervention**) | +5 |
| Drone kept securely mounted onto robot throughout ground arena run up to the first launch | +10 |
| Direct hit on drone target (+30 per flight) | +30 |
| Major collision | −10 |

** One manual intervention refers to one continuous, judge-declared recovery action limited to a specific obstacle or stuck condition, using either manual teleoperation or manual repositioning, typically to overcome a single cell. Autonomy Bonus is only awarded when the robot reaches the final cell and has completed at least one valid scene-understanding mission in that attempt.


## H. Competition Rounds

## Arena Configuration

- The ground arena will have a fixed layout size of 3 × 5 cells, with one additional cell designated as the final destination for the ground mission and the drone-launching area.

- The robot's starting position will always be located in the first row of the ground arena. However, its initial orientation may vary, and the robot may not necessarily be facing towards the final cell (drone-launching area).

- Two different arena layouts will be made available for practice prior to the competition.

- The Preliminary Rounds will use a new, previously undisclosed arena layout, different from those used during practice.

- The Final Round may feature further changes to the arena layout, including the arrangement of walls, terrain obstacles, and mission-related scenes or objects.

- All competition layouts will follow the published arena specifications, including the permitted terrain types, obstacles, scene categories, and mission requirements.

- The final arena layouts and mission-related placements will not be disclosed to teams before their respective competition rounds.

## Pre-Competition Preparation and Software Lockdown

- Software Freeze: Teams must finalize and deploy their robot software, configurations, and AI prompts before the competition begins. No modifications are permitted after the official software-freeze deadline, including between Preliminary and Final Rounds.

- Robot Handover: Teams must surrender their robots to the organizers by the designated deadline, with all robots and NUCs powered off. Teams or only allowed to power on on their robot and NUC when it is their turn or the next team on standby.

- After Software Freeze, teams are not allowed to have any extra networking connections other than RI_network.

- Teams must declare which laptop(s) or device(s) they will be using during the competition and only these declared devices are permitted during the team’s run. The network will be monitored to check if undeclared devices are connected.

- Permitted Operations: During competition, teams may execute predefined startup/restart procedures, monitor robot status, and perform judge-approved manual recovery. Autonomous adjustments made by the robot's own algorithms are permitted.

- Prohibited Actions: Teams must not manually enter or modify arena-specific information (e.g., maps, coordinates, routes, mission locations), change software or runtime parameters, or provide navigation instructions based on knowledge of the competition layout. This applies to all control methods, including laptops, terminals, web interfaces, and remote connections.

## Preliminary Rounds

- During Preliminary Rounds, each team will have one session of 12 mins, including ground tasks and drone launching. Teams are allowed to re-attempt the ground tasks (bring robot back to start and re-attempt) as many times as they want within their session duration. Only the best-scoring ground-task attempt will be kept. Drone launching must be performed last, with up to three drone launches. Top four teams with the highest total score (best ground-task attempt + drone launch score) will proceed to the Finals.


## Finals

- The finals will adopt the same the duration and rules as the Preliminary Rounds. Competition placing will be determined by highest total score, with the time taken for best-scoring ground-task attempt as tiebreaker if required.

## I. Competition Schedule

| Date / time | Activity |
| --- | --- |
| 23–26 Nov 2026 | Student testing, modification and preparation using practice arena |
| 27 Nov, 09:00–11:45 | Preliminary rounds for all teams |
| 27 Nov, 12:15–12:45 | Finals for the top four teams |
| 27 Nov, 13:30 onwards | TL@SUTD programme, including VIP segment (tentative) |

The event concept includes a preparation area, ground robot arena and drone flight path. Final allocation of spaces, reporting times and competition order will be communicated separately.

## J. Practical Design Considerations

- 1. Plan how the robot decides when to stop and interpret a scene, and when to continue exploring. Design a trigger (based on sensor data) to query scene understanding upon interesting observations, rather than continuously querying every camera frame.

- 2. Treat scene understanding query outputs as uncertain predictions and check them against other evidence where possible.

- 3. Be aware that some actions may be slow or unreliable: ROS 2 nodes can operate asynchronously, navigation can recover from local failures, and cloud-based scene analysis may require retries or timeouts.

- 4. Indoor air currents (e.g. air-conditioning) in Campus Centre can push a 25 g drone off course. Design your guidance to detect and correct drift during the flight rather than relying on an accurate launch alone.

- 5. Only the first ground contact counts, so controlling where and when the drone comes down matters as much as steering it towards the target.

- 6. Plan your three launches within the run time: allow time for the drone to be retrieved or a spare airframe to be prepared between launches.


## K. Autonomous Strategies and Fair Play

The competition is designed to evaluate each robot's ability to autonomously perceive, navigate and make decisions in an unfamiliar environment.

## 1. Permitted autonomous strategies

- Teams may implement any navigation, exploration, perception or decision-making algorithm, including AI-assisted algorithms, provided that the robot operates using information acquired through its own sensors or other explicitly permitted sources.

- Teams may also use general arena specifications and competition information provided by the organisers.

## 2. Human intervention and external information

- During an autonomous attempt, teams must not manually provide the robot with arena- specific information that is intended to be discovered autonomously. This includes, but is not limited to, undisclosed waypoint coordinates, obstacle locations, scene locations and exit positions.

- Teams must not remotely modify the robot's navigation targets, decision-making parameters or mission objectives during an autonomous attempt, except through explicitly permitted interventions.

- This restriction applies regardless of the interface used, including web GUIs, terminals, ROS 2 messages, configuration files and wireless controllers.

## 3. Monitoring and manual assistance

- Teams may monitor their robot's status, sensor data, camera feed and navigation progress without affecting their autonomy score.

- Any manual action that influences the robot's navigation or task execution must be declared to the judges and will be assessed according to the competition's manual- intervention and scoring rules.

- Safety-related interventions remain permitted, subject to the applicable competition procedures.

## 4. Verification and fair play

- Teams are expected to participate in good faith and respect the intended autonomous nature of the challenge.

- The organisers reserve the right to request clarification, inspect relevant system behaviour or logs, or require a supervised demonstration to verify claimed autonomous capabilities.

- Undeclared human assistance or deliberate attempts to circumvent competition restrictions may result in penalties or disqualification, subject to the organisers' review.

## 5. Interpretation of rules

- The rules describe the principles of permitted autonomous operation rather than an exhaustive list of possible implementations or prohibited workarounds.

- Novel or unconventional algorithms are encouraged, provided they comply with the principles above. Where ambiguity arises, the organisers will evaluate the method based on the information used, the degree of human involvement and the intended learning objectives of the challenge.


## L. Frequently Asked Questions (FAQ)

## A. Competition Course and Navigation

## 1. Are stairs and curtains mandatory obstacles, or will alternative routes be available?

The course will require robots to traverse stairs to reach the exit. However, alternative routes may be available around curtains. Some routes behind curtains may lead to additional obstacles or mission tasks, but not necessarily towards the exit. Teams that bypass these routes may forgo the corresponding points.

## 2. Where can curtains be placed in the course?

Curtains may be placed along various sections of the course, including sections with uneven terrain. However, curtains will not be placed:

- At the entrance to a scene-understanding cell.

- At the entrance to a staircase cell.

- At the start or exit of the course.

## 3. Will scene-understanding cells be at the same height as neighbouring cells?

Scene-understanding cells will always be at ground level (the same height as standard flat- ground cells). However, neighbouring cells may contain elevated or uneven terrain. Therefore, the robot may approach a scene-understanding cell from a higher elevation.

## 4. Will the QR codes be positioned so that robots can detect them without entering scene- understanding cells?

QR codes will be placed on the nearer half of the relevant cell wall, making them more accessible from neighbouring cells. Teams may attempt to detect and decode QR codes without fully entering the scene-understanding cell. However, successful detection will depend on the robot's camera position, viewing angle, and perception capabilities.

## B. Scene Understanding and Mission Tasks

## 5. How should teams interpret and complete M01 and M02 mission tasks?

For M01 and M02, teams must interpret the mission description to identify the relevant object(s) or hazard(s) from the predefined list of labels, then detect and report them along with their corresponding scene_type. For example, given the mission:

“The rescue team needs to provide urgent care to injured survivors. Find a location where medical equipment may be available.”

- Teams should identify medical_equipment as the relevant label, locate it in the environment, and report both the detected label (medical_equipment) and scene type (e.g., clinic) using the appropriate task_id.

Note: Teams must detect the relevant object or hazard, rather than infer its presence solely from the scene type.

## 6. What will hazard labels look like, and where might they be placed?

Hazard identification may be included as a mission task. Hazards will be represented by visual symbols or labels, which may be attached to walls, cabinets, containers, bottles, or other props within a scene. You may refer to section D.5 for examples of hazard symbols. Teams may be required to detect, identify, and report the relevant hazards.


## 7. Can a scene contain a survivor without a QR code, or a QR code without a survivor?

Survivor-related scenes will have one of two configurations:

- A survivor with a QR code.

- A survivor without a QR code.

There will be no survivor-related scene containing only a QR code without a survivor. Both configurations may appear, and teams should not assume that exactly one of each will be present.

## 8. Can multiple survivors appear in the same cell?

No. Each scene-understanding cell will contain at most one survivor. If both a survivor and QR code are present, teams should submit separate reports for the survivor-identification task and the QR-decoding task, where applicable.

## 9. Does decoding a QR code take priority over identifying a survivor?

No. QR decoding and survivor identification are treated as separate mission tasks. A robot may need to report both pieces of information from the same scene, using the appropriate task IDs.

## 10. How many QR codes will appear in the competition course?

The number of QR codes will correspond to the QR-related mission tasks specified in the final mission list. For example, if only one of the mission tasks (M01–M05) requires QR-code decoding, there will be only one QR code associated with that task in the course.

## 11. Must mission tasks M01–M05 be completed in numerical order?

No. Mission IDs are not arranged in the order in which robots will encounter them. Robots may encounter mission-related objects or information in any order. Teams must identify the relevant mission and submit the appropriate information using its corresponding task_id.

## C. Robot Control, Obstacles and Scoring

## 12. When is the robot's built-in AI mode permitted?

Teams are expected to use the robot's built-in AI mode when traversing stairs to reduce the risk of damaging the robot. For scoring purposes, stairs are the only obstacle type for which AI mode is permitted. Using AI mode on other terrain or obstacles, whether intentionally or due to incorrect terrain identification, will not qualify as an autonomous traversal for scoring purposes.

## 13. If a robot crosses an obstacle autonomously but later uses manual teleoperation to backtrack across the same obstacle, will the original points still count?

Yes. Once an obstacle has been successfully traversed autonomously and the corresponding points have been awarded, subsequent manual backtracking across the same obstacle will not revoke those points. Obstacle points are awarded at most once per obstacle within each attempt. Repeated traversal of the same obstacle does not earn additional points.

## 14. Are completion times used to calculate scores?

There is no time-completion bonus. However, completion time will still be recorded and used for tie-breaking according to the competition's tie-breaking rules.

## D. Robot Hardware and Modifications


## 15. What should teams do if the mounting standoffs above the robot's onboard computer become damaged? Can teams modify the mounting system?

The provided standoffs are intended to facilitate mounting additional components above the robot's onboard computer (NUC). Teams that do not currently require them are encouraged to remove and safely store them, particularly during testing involving legged locomotion. Teams may also design alternative mounting arrangements, such as:

- Adapters that reposition the standoffs using the existing mounting holes.

- Redesigned mounting plates.

- Alternative mounting interfaces attached to suitable existing structures.

Physical modifications are permitted provided that both teams sharing the same robot agree. Modifications should not compromise the robot's safety or normal operation.


## Sample layout
