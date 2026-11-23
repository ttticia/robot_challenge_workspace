# maze_nav behaviour

Flow of `ros2 launch maze_nav maze_nav.launch.py` with default arguments. It is the current
code in `src/maze_nav` (`explorer.py`, `cmd_vel_bridge.py`, `stair_detector.py`,
`curtain_detector.py`, `scene_trigger.py`) **plus the design decisions of 2026-11-21** (D1–D11
below). States in `CAPS` are the values on `/maze/status`.

Legend: ✅ = decided 2026-11-21 and implemented 2026-11-22 (unit tests + simulator; not yet
run on the robot). Unmarked = was already in the code.

```mermaid
flowchart TD
    %% ---------- startup ----------
    launch["launch maze_nav<br/>(Fast-LIO, wall_scan, SLAM, Nav2, detectors, explorer)"] --> posture{"robot standing?<br/>(lidar height)"}
    posture -- "not standing" --> notStand["✅ D1: log ERROR 'robot not standing: stand it up'<br/>no stand command sent"]
    notStand --> waitStand["✅ D1: wait, re-check posture"]
    waitStand --> posture
    posture -- standing --> move["cmd_vel_bridge: Move mode"]
    move --> gait["send gait: H-Step (gait:=hstep, default for now)"]
    gait --> opAuto[/"operator: lite3_auto after 'gait sent'"/]
    opAuto --> paused["PAUSED: SLAM + TF + Nav2 up, holding still"]
    paused --> startQ{"/maze/start received?"}
    startQ -- no --> paused
    startQ -- yes --> steady{"/wall_scan steady 3 s?"}
    steady -- "no: 'start held'" --> steady
    steady -- yes --> creep["CREEPING: 0.10 m straight ahead, no steering<br/>(no start scan, D9)"]
    creep --> explore["EXPLORING: nearest reachable frontier → Nav2 NavigateToPose<br/>(replan 1 Hz, finished goals blacklisted)"]

    %% ---------- end of run ----------
    explore --> exitQ{"open floor ≥ 1.1 m radius,<br/>≥ 1.5 m from start, 2 s?"}
    exitQ -- yes --> exited(["EXITED: save map"])
    explore -- "no frontiers left / 3 recovery rounds failed" --> noexit(["NO_EXIT: save map"])
    explore -- "15 min / Nav2 down / Ctrl+C" --> stopped(["STOPPED: save map"])

    %% ---------- junction ----------
    explore -- "/scene/junction: new cell, 2+ ways on" --> jStop["JUNCTION: cancel Nav2 goal, stay in this cell (D5)"]
    jStop --> jNext{"adjacent unexplored cell<br/>not yet scanned?<br/>(left-most first)"}
    jNext -- yes --> jFace["✅ D2: turn on the spot to face that cell<br/>(turn cut short > 10° by walls: no sideways move)"]
    jFace --> jCentre["✅ D2: move sideways to the centre line of the junction cell<br/>(stops if it stops closing in: unmapped wall)"]
    jCentre --> jScan["scan that cell: hold 1 s facing it,<br/>perception ON (QR only), /scene/capture"]
    jScan --> jSeen{"what was seen?"}
    jSeen -- "possible stairs / curtain" --> jNote["✅ D3: note candidate (position, kind)"]
    jSeen -- "object (room cell)" --> jRoom["✅ D7: mark that cell EXPLORED once scene<br/>understanding has been triggered (no room look-in later)"]
    jSeen -- nothing --> jNext
    jNote --> jNext
    jRoom --> jNext
    jNext -- "✅ D5: all adjacent unexplored cells scanned" --> qNext{"✅ D3: candidate in queue?<br/>(this junction's + earlier dead ends')"}
    qNext -- "yes: nearest first" --> qType{"stairs or curtain?"}
    qNext -- "no (or no frontiers left: end of run)" --> jLeave["✅ D6: leave by the nearest unexplored cell"]
    jLeave --> explore

    %% ---------- dead end ----------
    explore -- "/scene/trigger: facing a dead end from the cell before" --> deHold["DEAD_END: blacklist cell (never entered), hold 0.5 s"]
    deHold --> deLook["perception ON (/maze/perceive): QR scanner<br/>Pose mode (manual → Pose → body yaw, feet stay):<br/>left side wall 1 s, right side wall 1 s"]
    deLook --> deOff["perception off; Move mode → gait → auto mode"]
    deOff --> deCand{"possible stairs / curtain seen?"}
    deCand -- yes --> deNote["✅ D10: add to candidate queue (not verified now)"]
    deCand -- no --> explore
    deNote --> explore
    explore -- "✅ D10: no frontiers left, queue not empty" --> qNext

    %% ---------- objects / rooms (seen outside a junction scan) ----------
    explore -- "object cell within 3 m" --> room["room look-in (status JUNCTION): neighbour cell, face in, hold 3 s,<br/>left + right walls (Pose mode), /scene/capture; never enter; keepout mask"]
    room --> explore

    %% ---------- curtains ----------
    qType -- curtain --> cInspect["CURTAIN inspection: drive 0.5 m before line, square up,<br/>LIE DOWN, measure gap 3 s, stand up<br/>(once the YOLO model is trained: curtain_inspect_mode:=yolo,<br/>stand 1 m in front, 3 s of YOLO boxes instead)"]
    cInspect --> cGap{"gap 12–20 cm over ≥ half width?"}
    cGap -- "no: drop candidate" --> qNext
    cGap -- "yes: confirmed" --> cThrough["CURTAIN: face middle, walk through at 70 % speed to 0.2 m past"]
    cThrough --> cWiggle["turn 20° L / 40° R / 20° L, hold 0.8 s, check lidar + depth"]
    cWiggle --> cClear{"sensors clear?"}
    cClear -- yes --> explore
    cClear -- "no (≤ 12 creep/wiggle steps)" --> cWiggle
    cClear -- "still covered" --> atCurtain["AT_CURTAIN: hold"]
    atCurtain --> opC[/"operator: /maze/start"/] --> explore

    %% ---------- stairs ----------
    qType -- stairs --> sCheck["go in front of first riser, face it, hold, count hits"]
    sCheck -- "not stairs: drop candidate" --> qNext
    sCheck -- confirmed --> sStop
    explore -- "stairs confirmed ≤ 1.5 m, robot in front" --> sStop["STAIRS: pause navigation (cancel Nav2)"]
    sStop --> sAlign["line up: turn to face stop point → drive onto centre line →<br/>square up ±2.5°, base_link 0.45 m from first riser"]
    sAlign --> lined{"lined up?"}
    lined -- "no (blocked / > 45 s)" --> notLined["AT_STAIRS: NOT LINED UP, hold for hand line-up"]
    notLined --> opS[/"operator: line up, /maze/start"/] --> ai
    lined -- yes --> ai["AI mode: manual → AI state → AI basic gait"]
    ai --> aiWalk["✅ D4: walk 1.3 m forward at 0.1 m/s on /odom<br/>(going down: 2.0 m)"]
    aiWalk --> aiOff["✅ D4: leave AI mode by itself: AI exit → manual →<br/>Move mode → gait → auto mode"]
    aiOff --> explore
```

Also active while `EXPLORING`, not drawn as branches:

- **TURNING**: before a goal whose turn on the spot would hit a mapped wall, the explorer turns the long way itself (blocked both ways: left to Nav2).
- **FACING_QR**: a QR code located but not read → turn to face it and hold while `qr_scanner` decodes (perception on).
- ~~LOOKING~~: removed (D11).
- **Nav2 recovery** (`config/maze_bt.xml`): replan 1 Hz → follow path; recoveries clear costmaps, spin 90°, wait 3 s (no BackUp; the bridge never reverses).
- **curtain_yolo** (off: `curtain_yolo:=false`, no trained model yet): YOLO curtain boxes in junction / dead-end looks and CURTAIN only, never while EXPLORING; meant to replace the lie-down verification once trained (to_do.md ## curtains).
- **cmd_vel_bridge watchdog**: clamps 0.25 m/s / 0.5 rad/s, never backwards; stops on stale lidar/odom or an odom jump.

## Decisions (2026-11-21) and the code changes made for them

| # | Decision | Code before | Change made | Where |
|---|---|---|---|---|
| D1 | Not standing at launch → error message, wait until standing, then carry on with startup | `auto_stand:=true` sends stand once (a stand/sit toggle) when the lidar shows the robot lying, waits ≤ 15 s; posture unknown → nothing sent, startup goes on. `wall_scan` retries the floor 20 times, then guesses | No stand command; log an ERROR and loop on the posture check; hold gait, floor measurement and `PAUSED` until standing | `cmd_vel_bridge.py` (`auto_stand`, `stand_timeout`), `wall_scan.py` floor retry, `maze_nav.launch.py:450` |
| D2 | At a junction, for each adjacent unexplored cell: turn to face it, move sideways to the centre of the junction cell, then scan | Turns to face each way (0.5 s hold), no centring. **No sideways motion is possible today**: the bridge forwards only `linear.x` and `angular.z` (`cmd_vel_bridge.py:1032`), Nav2 `max_velocity` y = 0 | Forward a clamped `linear.y` in the bridge (watchdog + wall check too); explorer step "strafe to cell centre line" checked against walls on `/map`; confirm the Lite3 follows `linear.y` in auto mode | `cmd_vel_bridge.py:1003-1033`, `explorer.py` junction look, `config/nav2_params.yaml:256` |
| D3 | Candidates noted during the junction scan are verified right after it (all adjacent cells scanned), nearest first | A curtain candidate is inspected right after the look; a stairs candidate waits until the junction's other ways are explored (`curtain_branch.Deferral`). Dead-end candidates are handled after that stop | Detectors report candidates instead of handing over; the explorer keeps the junction's candidate list and releases them nearest first once the scan is done. A rejected candidate goes back to the list; when none are left, D6 | `explorer.py`, `stair_detector.py` 2b, `curtain_detector.py` 2b, `curtain_branch.py` |
| D4 | Stairs confirmed → pause navigation → line up and square → AI mode → walk 1.3 m → carry on by itself | Same up to the walk, but walks 1.2 m and then holds `AT_STAIRS` until the operator's `/maze/start` (which leaves AI mode) and `lite3_auto` | `ai_walk_distance` 1.2 → 1.3; after the walk the bridge leaves AI mode itself (AI exit → manual → Move → gait → **auto mode**, which the code has never sent unprompted) and the explorer resumes `EXPLORING` without `/maze/start` | `cmd_vel_bridge.py:44-62, 202`, `explorer.py` AT_STAIRS hold, `stair_detector.py` 4 |
| D5 | Don't leave the junction cell until every adjacent unexplored cell has been scanned | After the look it drives to the cell before each dead-end way (`junction_follow`), so it leaves mid-scan | Drop the dead-end visits from the junction follow-up; all scanning happens from the junction cell | `explorer.py:2426` onwards |
| D6 | Then leave by the nearest unexplored cell | Drives one cell into the rightmost way that isn't a dead end | Replace "rightmost" with the nearest unexplored adjacent cell (by path length) | `explorer.py:2426-2503` |
| D7 | A room cell with an object seen during the junction scan is marked explored once scene understanding has been triggered | Object cells are blocked (keepout) and later get a room look-in from a neighbouring cell | After its `/scene/capture` in the junction scan, mark the cell explored (no frontier goal, no later room look-in) | `explorer.py` `room_looks`, `block_objects` |
| D8 | Gait RUG General gear (changed 2026-11-22: H-Step for now); poster detection off | `gait:=rug`, launch starts only `qr_scanner` | Default gait `hstep` in the launch and cmd_vel_bridge (`gait:=rug` for RUG General gear); LOOKING and the poster wording removed | `maze_nav.launch.py`, `cmd_vel_bridge.py` |

| D9 | No start scan | Launch passes `start_scan:=false`, but the explorer's own default is `True` | Set the explorer default to `False` so a run without the launch file doesn't scan | `explorer.py:357` |
| D10 | Dead ends outside a junction scan keep the current look (0.5 s hold, Pose-mode left + right wall, QR on); candidates seen there are queued, not verified on the spot | Candidates seen during a dead-end stop are inspected / checked once the explorer is back in `EXPLORING` | Add them to the explorer's candidate queue (D3). The queue is worked nearest first after the next junction scan, or when no frontiers are left, before ending `NO_EXIT` | `explorer.py`, `stair_detector.py` 2b, `curtain_detector.py` 2b |
| D11 | Remove LOOKING (look-around in a QR room until a survivor poster is confirmed): poster detection is off | `look_topic` from poster_detector; `LOOKING` state in the explorer and in the detectors' `trigger_states` | Remove the LOOKING state, `look_topic` subscription and its parameters; drop `LOOKING` from `stair_detector` / `curtain_detector` `trigger_states` and from `at_dead_end` | `explorer.py` (`look_*`, ~1767-1800), `stair_detector.py`, `curtain_detector.py` |

No open questions left.
