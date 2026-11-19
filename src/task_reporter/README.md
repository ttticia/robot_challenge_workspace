# Task reporter

This standalone ROS 2 package posts confirmed competition results to the scoring
server. Build and run it from the workspace root after sourcing ROS Humble.

```bash
colcon build --packages-select task_reporter
source install/setup.bash
ros2 run task_reporter task_reporter
```

Publish a `std_msgs/msg/String` containing the completed seven-field report as
JSON on `/mission/task_completed`. The producer supplies the registered group ID,
task ID, and verified result fields. A producer can publish an M04 QR result as
soon as it confirms the decoded text belongs to that task. This node does not
subscribe to raw QR detections or VLM observations.

The default scoring endpoint is `http://192.168.10.141:8001/task_report`.
Override `server_url` with a mock endpoint for tests. The reporter publishes
outcomes on `/mission/submission_result` and logs them with the shortened group ID.
Statuses are `accepted`, `duplicate`, `rejected`, `invalid`, and `error`.

Each HTTP submission attempt appends a line to
`~/.ros/task_reporter/results.jsonl`. The line contains the exact report,
the submission outcome, and the latest TF robot position sampled before the
request. The default transform is `map` to `base_link`. A missing transform
leaves `robot_position` null and records the lookup error. The report's
`pos` remains the task location supplied by the producer. Configure
`results_file`, `map_frame`, or `base_frame` as ROS parameters if needed.
The results file contains the registered group PIN as part of the exact report.

Reports are posted in arrival order. The reporter skips repeats of the latest
accepted result for each group and task while it is running. A changed result
is sent as a correction. Following an HTTP error or timeout, check the outcome
before republishing the confirmed report, since the server may have received it.
