#!/usr/bin/env bash
# Install everything team_perception (qr_scanner, human_detector, poster_detector) needs,
# on the robot computer (Ubuntu 22.04, ROS 2 Humble already installed).
#
#     bash install_deps.sh            # apt (asks for sudo) + CPU torch + pip + YOLO weights
#     bash install_deps.sh --no-apt   # skip apt (packages already installed)
#
# The torch and weights downloads need internet once; after that everything runs offline.
# Then build from the workspace root:  colcon build --packages-select team_perception
set -euo pipefail
cd "$(dirname "$0")"

APT=1
for arg in "$@"; do
    case "$arg" in
        --no-apt) APT=0 ;;
        *) echo "unknown option: $arg" >&2; exit 1 ;;
    esac
done

APT_PKGS=(
    ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
    ros-humble-launch-ros
    python3-opencv python3-numpy
    python3-pyzbar libzbar0       # QR fallback decoder (Humble's OpenCV has no QUIRC)
    python3-pip
)
TORCH=(torch==2.14.1 torchvision==0.29.2)

if [[ $APT == 1 ]]; then
    echo ">> apt: ${APT_PKGS[*]}"
    sudo apt-get update
    sudo apt-get install -y "${APT_PKGS[@]}"
fi

# The robot computer has no CUDA GPU (robot_info.md): CPU build, ~200 MB instead of GBs.
echo ">> pip: ${TORCH[*]} (CPU build)"
pip3 install --user "${TORCH[@]}" --index-url https://download.pytorch.org/whl/cpu

echo ">> pip: requirements.txt"
pip3 install --user -r requirements.txt

# torch pulls in a pip setuptools that breaks colcon on Humble (_distribution_fullname);
# removing it restores the apt version.
echo ">> removing pip setuptools (keeps colcon working)"
pip3 uninstall -y setuptools || true

# ultralytics pulls in pip opencv-python, which shadows apt's python3-opencv (the build
# cv_bridge and the rest of ROS Humble use). Our code only needs what apt's 4.5.4 has.
echo ">> removing pip opencv (keeps apt python3-opencv)"
pip3 uninstall -y opencv-python opencv-python-headless opencv-contrib-python || true

echo ">> checking imports, YOLO weights -> ~/.cache/team_perception/yolo11n.pt"
python3 - <<'PY'
import os
import cv2, numpy, zxingcpp
assert cv2.__file__.startswith('/usr/lib/'), f'pip opencv shadows apt: {cv2.__file__}'
assert numpy.__version__.startswith('1.24.'), f'numpy {numpy.__version__}: want 1.24.x (apt scipy)'
from pyzbar import pyzbar
assert int(numpy.__version__.split('.')[0]) < 2, 'numpy must be < 2 for Humble cv_bridge'
d = os.path.expanduser('~/.cache/team_perception')
os.makedirs(d, exist_ok=True)
os.chdir(d)                      # ultralytics downloads bare names into the current folder
import torch
from ultralytics import YOLO
YOLO('yolo11n.pt')
# OpenVINO copy for human_detector (default model): fixed 320x416 input = the 640x480
# camera's 4:3 frame letterboxed at 416, so no padding is computed.
if not os.path.isdir('yolo11n_openvino_model'):
    YOLO('yolo11n.pt').export(format='openvino', imgsz=(320, 416), half=False, dynamic=False)
print(f'OK: opencv {cv2.__version__}, numpy {numpy.__version__}, zxing-cpp, pyzbar, '
      f'torch {torch.__version__}, weights in {d}')
PY
echo ">> done. Build: cd <workspace> && colcon build --packages-select team_perception"
