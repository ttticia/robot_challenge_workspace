#!/usr/bin/env bash
# Record a rosbag with one of the presets from ROSBAG_GUIDE.md (section 2).
#
#   record.sh <preset> [name] [-- extra topics...]
#   record.sh --rename [old] new      rename a bag (old defaults to the newest bag)
#   record.sh --list
#
# Presets:
#   full        teleop, everything incl. depth          (guide 2.1, groups A-G)
#   nav         Nav2 driving, everything except depth   (guide 2.2, groups A, B, D-G)
#   perception  QR / poster replay, incl. depth         (guide 2.3, groups A, B, C, E, G)
#   detections  perception results only, small bag      (guide 2.3 small variant)
#   navdebug    why it moved/stopped/turned             (guide 2.4)
#   lidar       raw sensors to re-run Fast-LIO / SLAM   (guide 2.5, groups A, D)
#   video       compressed colour camera only           (guide 2.6)
#
# Options:
#   -n, --dry-run   print the command, don't record
#   -l, --list      list presets and their topics
#   -h, --help      this help
#
# The bag goes to ~/challenge_ws/ROS_bags/<name>; the default name is <preset>_<date>_<time>.
# Names: letters, digits, '.', '_', '-' only. Topics after "--" are appended (e.g. an aligned
# depth topic). After Ctrl+C you are asked for a new name (Enter keeps it); REC_ASK_NAME=0
# skips the question.

set -euo pipefail

BAG_DIR="${BAG_DIR:-$HOME/challenge_ws/ROS_bags}"
QOS_FILE="$BAG_DIR/qos_overrides.yaml"
MIN_FREE_GB=5

# ---- topic groups (ROSBAG_GUIDE.md section 1) ------------------------------------------
A="/tf /tf_static /rosout"
B="/camera/color/image_raw/compressed /camera/color/camera_info"
C="/camera/depth/image_raw /camera/depth/camera_info"
D="/livox/points /livox/lidar /livox/imu /imu/data /leg_odom2"
E="/odom /scan /map"
F="/plan /cmd_vel_nav /cmd_vel_smoothed /nav_cmd_vel /cmd_vel /simple_cmd /handle_state"
G="/found /qr/detections /posters/detections /humans/detections /camera/color/image_offboard/compressed"

preset_topics() {
  case "$1" in
    full)       echo "$A $B $C $D $E $F $G" ;;
    nav)        echo "$A $B $D $E $F $G" ;;
    perception) echo "$A /map $B $C $G" ;;
    detections) echo "$A /map /camera/color/camera_info $G" ;;
    navdebug)   echo "$A /livox/points /odom /scan /map /leg_odom2 $F" ;;
    lidar)      echo "$A /livox/lidar /livox/imu /livox/points /imu/data /leg_odom2" ;;
    video)      echo "/camera/color/image_raw/compressed" ;;
    *)          return 1 ;;
  esac
}
PRESETS="full nav perception detections navdebug lidar video"

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

free_gb=$(df -Pk "$BAG_DIR" | awk 'NR==2 {print int($4 / 1048576)}')
if (( free_gb < MIN_FREE_GB )); then
  echo "Warning: only ${free_gb} GB free in $BAG_DIR (depth + lidar use ~1 GB/min)." >&2
fi

# Recording raw depth while Nav2 drives delayed /livox/points ~1.2 s and collision_monitor
# held the robot at zero (2026-11-13). Refuse unless the user insists.
if [[ " $TOPICS " == *" /camera/depth/image_raw "* ]] \
   && timeout 5 ros2 node list 2>/dev/null | grep -qE "controller_server|collision_monitor"; then
  echo "Nav2 is running and preset '$PRESET' records raw depth, which stalls the lidar." >&2
  echo "Use 'nav' instead, or set ALLOW_DEPTH_WITH_NAV2=1 to record anyway." >&2
  [[ "${ALLOW_DEPTH_WITH_NAV2:-0}" == 1 ]] || exit 1
fi

# ---- record ------------------------------------------------------------------------------
# shellcheck disable=SC2206
CMD=(ros2 bag record -o "$OUT")
[[ "$PRESET" != video ]] && CMD+=(--qos-profile-overrides-path "$QOS_FILE")
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
  ros2 bag info "$OUT" | grep -E "Duration|Bag size|Messages|Topic:.*Count: 0" || true
  echo "Topics with 'Count: 0' above were not published during the recording."
  ask_new_name
fi
