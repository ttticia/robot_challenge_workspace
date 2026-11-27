#!/usr/bin/env bash
# Merge our stairs crossing into the team's maze_explore (explorer.py + explore.launch.py),
# so `ros2 launch maze_explore explore.launch.py` detects and crosses the stairs by default.
#
#   bash ~/challenge_ws/src/stairs_crossing/integration/apply_to_maze_explore.sh          apply
#   bash ~/challenge_ws/src/stairs_crossing/integration/apply_to_maze_explore.sh --undo   put the team files back
#
# Backup of the two team files: ~/maze_explore_backup/ (outside challenge_ws, so colcon
# never sees a second copy of the package).
set -euo pipefail
WS=~/challenge_ws
PKG=$WS/src/maze_explore
HERE=$(cd "$(dirname "$0")" && pwd)
PATCH=$HERE/maze_explore_stairs.patch
BK=~/maze_explore_backup
FILES="launch/explore.launch.py maze_explore/explorer.py"

rebuild() {
  cd $WS
  set +u; source /opt/ros/humble/setup.bash; set -u
  nice -n 19 colcon build --symlink-install --packages-select maze_explore --parallel-workers 1
  echo; echo "Now run in every terminal: source ~/challenge_ws/install/setup.bash"
}

if [ "${1:-}" = "--undo" ]; then
  [ -d $BK ] || { echo "no backup in $BK"; exit 1; }
  for f in $FILES; do cp -v $BK/$f $PKG/$f; done
  rebuild; echo "UNDONE: team maze_explore restored"; exit 0
fi

[ -f "$PATCH" ] || { echo "missing $PATCH"; exit 1; }
if grep -q "def do_ai_walk" $PKG/maze_explore/explorer.py; then
  echo "already applied (explorer.py has do_ai_walk). Nothing to do."; exit 0
fi

echo "== dry run"
cd $WS/src
if ! patch -p1 --dry-run < "$PATCH"; then
  echo; echo "NOT APPLIED: the team files changed since 25 Nov 18:35, so the patch doesn't fit."
  echo "Send Claude: $PKG/maze_explore/explorer.py and $PKG/launch/explore.launch.py"; exit 1
fi

echo "== backup to $BK"
mkdir -p $BK/launch $BK/maze_explore
for f in $FILES; do cp -v $PKG/$f $BK/$f; done

echo "== apply"
patch -p1 --no-backup-if-mismatch < "$PATCH"
python3 -c "import ast,sys; [ast.parse(open(f).read(), f) for f in sys.argv[1:]]" $PKG/maze_explore/explorer.py $PKG/launch/explore.launch.py
rebuild

set +u; source $WS/install/setup.bash; set -u
echo "== check"
ros2 launch maze_explore explore.launch.py --show-args | grep -E "^ +'(stairs|stair_check|ai_auto|ai_reanchor)'" -A1 || true
echo "APPLIED. Run: ros2 launch maze_explore explore.launch.py 2>&1 | tee ~/challenge_ws/logs/runN.log"
