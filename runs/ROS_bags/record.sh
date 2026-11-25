#!/usr/bin/env bash
# Record a rosbag with one of the presets from ROSBAG_GUIDE.md (section 2).
#
#   record.sh <preset> [name] [-- extra topics...]
#   record.sh --rename [old] new      rename a bag (old defaults to the newest bag)
#   record.sh --list
#
# Presets:
#   manual      hand-driven demonstration run with maze_explore explore.launch.py
#               explore:=false: lidar, odometry, /scan, /map, tf, colour, controller.
#               No depth: one raw depth subscriber slows every bridge stream (guide 2.6)
#   explore     autonomous maze_explore run: cloud, odometry, /scan, /map, tf, colour,
#               commands sent, explorer status / events
#   curtain     curtain detection test, hand-driven, with curtain_node running
#               (`ros2 run maze_explore curtain_node --ros-args -p always:=true`): colour,
#               the node's /curtain/seen + /curtain/status, tf, and /odom /scan /map if the
#               mapping launch runs too (guide 2.3). No depth
#   img         camera only: /camera/color/image_raw/compressed, nothing else
#
# Options:
#   -n, --dry-run   print the command, don't record
#   -l, --list      list presets and their topics
#   -h, --help      this help
#
# The bag goes to ~/challenge_ws/runs/ROS_bags/<name>; the default name is <preset>_<date>_<time>.
# Names: letters, digits, '.', '_', '-' only. Topics after "--" are appended (e.g. an aligned
# depth topic). After Ctrl+C you are asked for a new name (Enter keeps it); REC_ASK_NAME=0
# skips the question.
#
# manual, explore and curtain bags are zstd-compressed message by message (about half the size; plain
# .db3 file, read with ros2 bag / rosbag2_py as usual). REC_COMPRESS=0 turns it off.
# They also run the sensor-rate check
# (src/maze_explore/preflight.sh) first and ask before recording on a NO-GO; REC_CHECK=0 skips it.

set -euo pipefail

BAG_DIR="${BAG_DIR:-$HOME/challenge_ws/runs/ROS_bags}"
QOS_FILE="$BAG_DIR/qos_overrides.yaml"
MIN_FREE_GB=5

# ---- topic groups -------------------------------------------------------------------------
A="/tf /tf_static /rosout"
B="/camera/color/image_raw/compressed /camera/color/camera_info"
E="/odom /scan /map"

preset_topics() {
  case "$1" in
    # Demonstration drive by hand, to design the exploration stack from. Needs
    # `ros2 launch maze_explore explore.launch.py explore:=false` (Fast-LIO, wall_scan,
    # slam_toolbox) running.
    #   /livox/lidar + /livox/imu   raw lidar: Fast-LIO can be re-run with other settings
    #   /cloud_registered_body      Fast-LIO's motion-corrected cloud (local topic, no bridge
    #                               copy): /scan can be rebuilt with another height band
    #   /odom /scan /map + tf       where the robot was and what it had mapped
    #   colour (compressed)         what the robot was looking at
    #   /handle_state /simple_cmd /cmd_vel   what the operator did (sticks, mode changes)
    #   /commands/joy_raw           handheld controller buttons (mode / gait changes made on
    #                               the controller never reach /simple_cmd)
    #   /maze/body_height           wall_scan's height above the floor
    #   /leg_odom2 /imu/data        robot's own odometry and body tilt (terrain, stairs)
    #   /explore/*                  the explorer's state, if it runs (drive:=false)
    manual)     echo "$A $B /livox/lidar /livox/imu /cloud_registered_body /imu/data /leg_odom2 $E /handle_state /commands/joy_raw /simple_cmd /cmd_vel /maze/body_height /explore/status /explore/event /explore/cells" ;;
    # Autonomous maze_explore run (explore.launch.py). Every topic taken from the robot is one
    # more copy across the bridge, so the only one here is the compressed colour stream; the
    # rest is published on this computer: Fast-LIO's cloud, odometry, the map, what the
    # explorer decided and sent.
    # Left out on purpose: raw lidar, /imu/data, /leg_odom2, /handle_state, camera_info.
    explore)    echo "$A /camera/color/image_raw/compressed /cloud_registered_body $E /simple_cmd /cmd_vel /explore/status /explore/event /explore/cells /maze/body_height" ;;
    # Curtain detection test by hand: what the camera saw and what curtain_node said about
    # each frame (boxes, confidence, direction), so misses and false boxes can be found
    # afterwards. /curtain/image is NOT recorded: it is raw (2.7 MB/s) and makes the node
    # decode at full size; the boxes can be drawn again from /curtain/seen.
    # $E and /handle_state only have messages with the mapping launch / the controller on.
    curtain)    echo "$A $B /curtain/seen /curtain/status $E /maze/body_height /handle_state" ;;
    # Camera only: the compressed colour stream and nothing else (small, no mapping needed).
    img)        echo "/camera/color/image_raw/compressed" ;;
    *)          return 1 ;;
  esac
}
PRESETS="manual explore curtain img"
checked() { [[ "$PRESET" == manual || "$PRESET" == explore || "$PRESET" == curtain ]]; }

usage() { sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; }

list_presets() {
  for p in $PRESETS; do
    printf '%-11s %s\n' "$p" "$(preset_topics "$p")"
  done
}

# ---- names -------------------------------------------------------------------------------
valid_name() {   # usable as a folder and file name, can't be mistaken for an option
  [[ "$1" =~ ^[A-Za-z0-9_][A-Za-z0-9._-]*$ ]]
}

newest_bag() {
  local m
  m=$(ls -dt "$BAG_DIR"/*/metadata.yaml 2>/dev/null | head -1) || true
  [[ -n "$m" ]] && basename "$(dirname "$m")"
}

# rename_bag OLD NEW: folder, its <OLD>_N.db3/.mcap files, metadata.yaml and <OLD>.mp4.
# (metadata.yaml refers to the storage files by name, so they are renamed together.)
rename_bag() {
  local old="$1" new="$2" src="$BAG_DIR/$1" dst="$BAG_DIR/$2" f base
  if [[ ! -f "$src/metadata.yaml" ]]; then
    echo "No bag '$old' in $BAG_DIR." >&2; return 1
  fi
  if ! valid_name "$new"; then
    echo "Bad name '$new': use letters, digits, '.', '_' or '-' (no spaces or '/')." >&2
    return 1
  fi
  if [[ -e "$dst" || -e "$BAG_DIR/$new.mp4" ]]; then
    echo "$dst (or $new.mp4) already exists. Pick another name." >&2; return 1
  fi
  if pgrep -f "bag record .*-o $src( |\$)" >/dev/null; then
    echo "'$old' is still being recorded. Stop the recording first." >&2; return 1
  fi
  mv "$src" "$dst"
  for f in "$dst/$old"_*; do
    [[ -e "$f" ]] || continue
    base=$(basename "$f")
    mv "$f" "$dst/$new${base#"$old"}"
  done
  python3 - "$dst/metadata.yaml" "$old" "$new" <<'PY'
import re, sys
path, old, new = sys.argv[1:]
text = open(path).read()
# only storage file entries: "    - old_0.db3" and "path: old_0.db3"
text = re.sub(r'^(\s*(?:-|- path:|path:)\s*)' + re.escape(old) + r'(_\d+\.(?:db3|mcap))\s*$',
              lambda m: m.group(1) + new + m.group(2), text, flags=re.M)
open(path, 'w').write(text)
PY
  [[ -f "$BAG_DIR/$old.mp4" ]] && mv "$BAG_DIR/$old.mp4" "$BAG_DIR/$new.mp4"
  echo "Renamed: $src -> $dst"
}

ask_new_name() {   # after a recording: offer to rename, Enter keeps the name
  local new
  [[ -t 0 && "${REC_ASK_NAME:-1}" == 1 ]] || return 0
  while true; do
    read -r -p "Name for this bag (Enter keeps '$NAME'): " new || return 0
    [[ -z "$new" || "$new" == "$NAME" ]] && return 0
    rename_bag "$NAME" "$new" && return 0
  done
}

# ---- arguments ---------------------------------------------------------------------------
DRY_RUN=0
POSITIONAL=()
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--dry-run) DRY_RUN=1 ;;
    -l|--list)    list_presets; exit 0 ;;
    -r|--rename)
      shift
      case $# in
        1) OLD=$(newest_bag) || { echo "No bags in $BAG_DIR." >&2; exit 1; }
           echo "Newest bag: $OLD"; rename_bag "$OLD" "$1"; exit ;;
        2) rename_bag "$1" "$2"; exit ;;
        *) echo "Usage: record.sh --rename [old] new" >&2; exit 2 ;;
      esac ;;
    -h|--help)    usage; exit 0 ;;
    --)           shift; EXTRA=("$@"); break ;;
    -*)           echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    *)            POSITIONAL+=("$1") ;;
  esac
  shift
done

if [[ ${#POSITIONAL[@]} -lt 1 || ${#POSITIONAL[@]} -gt 2 ]]; then
  usage >&2; exit 2
fi
PRESET="${POSITIONAL[0]}"
if ! TOPICS=$(preset_topics "$PRESET"); then
  echo "Unknown preset '$PRESET'. Choose one of: $PRESETS" >&2; exit 2
fi
NAME="${POSITIONAL[1]:-${PRESET}_$(date +%Y%m%d_%H%M%S)}"
if ! valid_name "$NAME"; then
  echo "Bad name '$NAME': use letters, digits, '.', '_' or '-' (no spaces or '/')." >&2; exit 2
fi
OUT="$BAG_DIR/$NAME"

# ---- checks ------------------------------------------------------------------------------
if ! command -v ros2 >/dev/null; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [[ -e "$OUT" ]]; then
  echo "$OUT already exists. Pick another name." >&2; exit 1
fi
mkdir -p "$BAG_DIR"

if [[ ! -f "$QOS_FILE" ]]; then
  cat > "$QOS_FILE" <<'EOF'
/map:
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
EOF
  echo "Created $QOS_FILE"
fi

NODES=$(timeout 5 ros2 node list 2>/dev/null || true)
node_running() { grep -qx "$1" <<<"$NODES"; }

free_gb=$(df -Pk "$BAG_DIR" | awk 'NR==2 {print int($4 / 1048576)}')
if (( free_gb < MIN_FREE_GB )); then
  echo "Warning: only ${free_gb} GB free in $BAG_DIR (a bag takes about 0.5 GB/min)." >&2
fi

# maze_explore presets: Fast-LIO needs /livox/imu at ~200 Hz. One subscriber to raw depth
# (even BEST_EFFORT, measured 2026-11-23: /imu/data 195 -> 11 Hz, /livox/imu -> 8 Hz) or to
# /livox/points slows EVERY stream from the bridge, so those are refused here. The camera has
# no compressed depth topic; one would have to be published on the robot side of the bridge.
if checked; then
  for t in "${EXTRA[@]}"; do
    if [[ "$t" == /camera/depth/* || "$t" == /livox/points ]]; then
      echo "$t slows every bridge stream (lidar IMU, odometry) while it is recorded." >&2
      echo "Set ALLOW_HEAVY=1 to record it anyway." >&2
      [[ "${ALLOW_HEAVY:-0}" == 1 ]] || exit 1
    fi
  done
  PREFLIGHT="$HOME/challenge_ws/src/maze_explore/preflight.sh"
  if [[ "${REC_CHECK:-1}" == 1 && $DRY_RUN == 0 && -x "$PREFLIGHT" ]]; then
    echo "Checking sensor rates (about 30 s; REC_CHECK=0 skips this)..."
    # curtain_node is part of a curtain test, not a leftover stack
    [[ "$PRESET" == curtain ]] && export EXPLORE_CHECK_ALLOW=curtain_node
    if ! "$PREFLIGHT"; then
      echo "NO-GO: with a slow /livox/imu the bag's /odom, /map and tf will be wrong, and" \
           "Fast-LIO cannot be re-run from it. Camera and raw lidar are still recorded." >&2
      if [[ -t 0 ]]; then
        read -r -p "Record anyway? [y/N] " ans || ans=n
        [[ "$ans" == [yY]* ]] || exit 1
      else
        exit 1
      fi
    fi
  fi
fi

# Topics a manual bag is useless without. Checked before (published?) and after (count).
REQUIRED=()
if [[ "$PRESET" == manual ]]; then
  REQUIRED=(/tf_static /livox/lidar /livox/imu /odom /scan /map
            /camera/color/image_raw/compressed /handle_state)
  for n in /slam_toolbox /odom_relay; do
    node_running "$n" || echo "Warning: $n isn't running, so no /odom, /scan or /map. Start" \
      "'ros2 launch maze_explore explore.launch.py explore:=false' first (robot standing still)." >&2
  done
fi
if [[ "$PRESET" == curtain ]]; then
  REQUIRED=(/camera/color/image_raw/compressed /curtain/seen)
  node_running /curtain_node || echo "Warning: /curtain_node isn't running, so no /curtain/seen." \
    "Start it first: ros2 run maze_explore curtain_node --ros-args -p always:=true" >&2
fi
if (( ${#REQUIRED[@]} )); then
  LISTED=$(timeout 5 ros2 topic list 2>/dev/null || true)
  for t in "${REQUIRED[@]}"; do
    grep -qx "$t" <<<"$LISTED" || echo "Warning: $t is not published (camera / lidar driver" \
      "/ mapping launch running? ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}). The bag will be missing it." >&2
  done
fi

# ---- record ------------------------------------------------------------------------------
# shellcheck disable=SC2206
CMD=(ros2 bag record -o "$OUT")
CMD+=(--qos-profile-overrides-path "$QOS_FILE")
COMPRESS="${REC_COMPRESS:-}"
[[ -z "$COMPRESS" ]] && checked && COMPRESS=1
if [[ "$COMPRESS" == 1 ]]; then
  # message mode: the file stays <name>_0.db3. A full queue drops messages, so it is large.
  CMD+=(--compression-mode message --compression-format zstd
        --compression-queue-size 2000 --compression-threads 2)
fi
CMD+=($TOPICS "${EXTRA[@]}")

echo "Preset: $PRESET  ->  $OUT"
echo "${CMD[*]}"
(( DRY_RUN )) && exit 0
echo "Recording. Ctrl+C to stop."
trap : INT              # let Ctrl+C stop the recorder, then print the summary
"${CMD[@]}" || true   # Ctrl+C makes ros2 bag exit non-zero
trap - INT

if [[ -f "$OUT/metadata.yaml" ]]; then
  echo
  INFO=$(ros2 bag info "$OUT" 2>/dev/null || true)
  grep -E "Duration|Bag size|Messages|Topic:.*Count: 0" <<<"$INFO" || true
  echo "Topics with 'Count: 0' above were not published during the recording."
  missing=()
  for t in "${REQUIRED[@]}"; do
    n=$(grep -E "Topic: $t \|" <<<"$INFO" | sed -nE 's/.*Count: ([0-9]+).*/\1/p')
    [[ "${n:-0}" -gt 0 ]] || missing+=("$t")
  done
  # Rates: a stream that arrived slowly is as bad as a missing one (bag "demo", 2026-11-23:
  # /livox/imu at 8 Hz, so /odom and /map were garbage).
  if checked; then
    # Each stream's rate over ITS OWN first-to-last message, not the bag's length: a stack
    # stopped before the recorder used to read as a slow /odom (run3, 2026-11-24).
    python3 - "$OUT" "$PRESET" <<'PY' || true
import glob, sqlite3, sys
out, preset = sys.argv[1:]
need = {'/odom': 9, '/scan': 8, '/cloud_registered_body': 9,
        '/camera/color/image_raw/compressed': 20}
if preset == 'curtain':
    need = {'/camera/color/image_raw/compressed': 20, '/curtain/seen': 2, '/odom': 9, '/scan': 8}
if preset == 'manual':
    need.update({'/livox/imu': 150, '/livox/lidar': 9, '/imu/data': 150, '/handle_state': 40})
db = sqlite3.connect('file:%s?mode=ro' % glob.glob(out + '/*.db3')[0], uri=True)
ids = {n: i for i, n in db.execute('select id, name from topics')}
for topic, hz_min in need.items():
    ts = [r[0] * 1e-9 for r in db.execute('select timestamp from messages where topic_id = ? '
                                          'order by timestamp', (ids.get(topic, -1),))]
    span = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    if span < 5:
        print('  %-38s %d messages' % (topic, len(ts)))
        continue
    hz, gap = (len(ts) - 1) / span, max(b - a for a, b in zip(ts, ts[1:]))
    print('  %-38s %6.1f Hz over %5.1f s, longest gap %.2f s%s' % (
        topic, hz, span, gap, '   SLOW (needs >= %d)' % hz_min if hz < hz_min else ''))
PY
    [[ "$PRESET" == curtain ]] && echo "/odom and /scan only count if the mapping launch ran; a SLOW /curtain/seen means the node fell behind."
    echo "SLOW /odom, /scan or (manual) /livox/imu: the map and pose in this bag cannot be trusted."
    echo "A longest gap near 1 s is a stall in the robot's feed: the explorer stands still through it."
  fi
  if (( ${#missing[@]} )); then
    echo "WARNING: this $PRESET bag has no ${missing[*]}. Fix the driver / topic and record again." >&2
  fi
  ask_new_name
fi
