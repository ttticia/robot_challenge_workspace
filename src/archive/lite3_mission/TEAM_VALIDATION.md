# Team integration verification — 12 November 2026

Base: `d84078d202f920275f6dad1de7756fa60c9df7a7` from
`ttticia/robot_challenge_workspace`. Branch: `feat/autonomous-navigation`.

All 33 files tracked at the base commit were hashed before integration and checked
again afterwards. They remain byte-for-byte unchanged. Every added source file
is under `src/lite3_mission/`; no existing QR code, configuration, recordings or
logs were edited. No robot connection or remote GitHub write was made.

## Checks completed locally

- Both `team_perception` and `lite3_mission` built with ROS 2 Humble and colcon.
- `rosdep check` passed for both source packages.
- Combined test run: **34 passed, 1 skipped, 1 expected failure**. The skipped
  existing QR photo test requires optional zxing-cpp; the expected failure is the
  existing curved-paper photograph. Neither test was changed or disabled here.
- The unchanged QR helper tests separately returned 10 passed, 1 skipped and
  1 expected failure with the available PyZbar backend.
- A real instance of the team's unchanged `qr_scanner` decoded generated image
  pixels, wrote its original CSV and JPEG evidence, and published `/qr/detections`.
  The new adapter forwarded that detection to the mission reporter, preserving
  the capture timestamp and evidence without inventing a depth position.
  The pipeline published no `/cmd_vel` messages or publishers.
- Adapter tests cover the documented localized schema, camera-frame coordinates,
  absent depth/frame/time, non-finite geometry, invalid timestamps and decoded
  text containing apparent robot instructions. Text remains data.
- A real Nav2 planner/controller/BT waypoint test using this branch's installed
  mission launch passed. The synthetic planar base moved approximately 0.468 m
  towards a 0.5 m goal (0.05 m test goal tolerance), then stopped and disarmed.
  Maximum commanded translation was 0.10 m/s and yaw approximately 0.228 rad/s.
- Existing waypoint/frontier software validation is retained in `VALIDATION.json`;
  the new branch integration evidence is recorded there separately.

The combined test launcher is source-only and uses a loopback DDS profile on
domain 89, isolated from robot domain 4:

```bash
bash src/lite3_mission/test/run_team_tests.sh
```

The Nav2 fixture uses domain 90 and a synthetic kinematic base. Neither run
establishes physical robot performance, camera/depth registration, synchronized
RGB/depth capture, real stopping distance, narrow-maze traversal, stairs or
curtain pushing. Factory gait APIs and the actual course Nav2 velocity routing
still require verification on the robot. QR reporting does not yet trigger an
autonomous pause or a route change.

## Upload this branch

Changes are staged for review. The repository's Git commit name/email were unset
when checked. Configure your own Git identity before committing; a GitHub no-reply
email can be used for privacy. Then:

```bash
git diff --cached --name-status
git commit -m "Add autonomous navigation alongside team QR perception"
git push -u origin feat/autonomous-navigation
```

Open a pull request and review the additions before merging into the group branch.
The new ROS package should be uploaded from this branch; the older standalone
source archive predates the team QR adapter.
