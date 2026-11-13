#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -x $root/.venv/bin/python ]]; then python3 -m venv --system-site-packages "$root/.venv"; fi
"$root/.venv/bin/pip" install 'numpy==1.26.4' 'onnxruntime==1.23.2'
echo 'Vision ready. OpenCV and cv_bridge are supplied by the ROS installation.'
