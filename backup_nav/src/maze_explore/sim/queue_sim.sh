#!/bin/bash
# One simulator at a time, shared by every session, with no waiting loops and no repeats.
#
#   src/maze_explore/sim/queue_sim.sh <tag> [maze=challenge_a.json] [timeout_s=900] [launch args...]
#
# - Takes a lock (logs/sim/lock): the run starts the moment the one before it ends.
# - Before it runs it looks in logs/sim/index.tsv for a finished run of the SAME maze, the
#   SAME launch arguments and the SAME code (git HEAD + uncommitted changes under src/).
#   If there is one it prints where that run's logs are and does not run again
#   (FORCE=1 runs anyway).
# - Logs: logs/sim/<date-time>-<tag>/ (launch.log, sim.log, events.log, run.out). One line
#   per run in logs/sim/index.tsv: time, tag, maze, code, seconds, result, args, folder.
# - ROS domain 97 for every queued run (DOMAIN=n to change); it never reaches the robot.
TAG=${1:?usage: queue_sim.sh <tag> [maze] [timeout_s] [launch args...]}
MAZE=${2:-challenge_a.json}
TIMEOUT=${3:-900}
shift 3 2>/dev/null || shift $#
ARGS="$*"
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../.." && pwd)
OUT=$WS/logs/sim
mkdir -p "$OUT"
code() { (cd "$WS" && echo "$(git rev-parse --short HEAD)+$(git diff HEAD -- src | sha1sum | cut -c1-8)"); }
echo "queued: $TAG ($MAZE $ARGS); waiting for the simulator lock"
exec 9>"$OUT/lock"
flock 9
# a simulator started without this script holds no lock: wait for it as well
while pgrep -f '^python3 .*[f]ake_maze_lite3\.py' >/dev/null; do sleep 2; done
CODE=$(code)                       # the code as it is when the run starts
if [ -z "$FORCE" ] && [ -f "$OUT/index.tsv" ]; then
    SAME=$(awk -F'\t' -v m="$MAZE" -v c="$CODE" -v a="$ARGS" '$3==m && $4==c && $7==a' "$OUT/index.tsv" | tail -1)
    if [ -n "$SAME" ]; then
        echo "already run on this code: $(echo "$SAME" | cut -f6)"
        echo "logs: $(echo "$SAME" | cut -f8)"
        exit 0
    fi
fi
DIR=$OUT/$(date +%Y%m%d-%H%M%S)-$TAG
T0=$SECONDS
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 nice -n 19 "$HERE/run_sim.sh" "$MAZE" "$TIMEOUT" "$DIR" "${DOMAIN:-97}" $ARGS 9>&- | tee "$DIR.out"
RC=${PIPESTATUS[0]}
RESULT=$(grep -a "^result after" "$DIR.out" | tail -1 | sed 's/^result after [0-9]*s: //; s/^\]: //')
[ -n "$RESULT" ] || RESULT=$(tail -1 "$DIR.out")
mv "$DIR.out" "$DIR/run.out" 2>/dev/null
printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$(date '+%F %T')" "$TAG" "$MAZE" "$CODE" "$((SECONDS - T0))" "$RESULT" "$ARGS" "$DIR" >> "$OUT/index.tsv"
echo "logs: $DIR"
exit $RC
