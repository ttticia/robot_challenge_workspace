#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
python3 -m venv --system-site-packages "$root/.venv"
"$root/.venv/bin/pip" install -r "$root/requirements-sim.txt"
