#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/coco/follwer_ws}"
SAMPLE_DIR="${WORKSPACE_ROOT}/data/samples"
PEDESTRIAN_JPG="${SAMPLE_DIR}/pedestrian.jpg"
PEOPLE_JPG="${SAMPLE_DIR}/people.jpg"
OUTPUT_VIDEO="${SAMPLE_DIR}/public_people_smoke.avi"

mkdir -p "${SAMPLE_DIR}"

if [ ! -f "${PEDESTRIAN_JPG}" ]; then
  curl -L https://raw.githubusercontent.com/vinay0410/Pedestrian_Detection/master/sample_images/pedestrian.jpg -o "${PEDESTRIAN_JPG}"
fi

if [ ! -f "${PEOPLE_JPG}" ]; then
  curl -L https://raw.githubusercontent.com/vinay0410/Pedestrian_Detection/master/sample_images/people.jpg -o "${PEOPLE_JPG}"
fi

python3 - <<'PY'
import cv2
import numpy as np
import os

workspace_root = os.environ.get("WORKSPACE_ROOT", "/home/coco/follwer_ws")
sample_dir = os.path.join(workspace_root, "data", "samples")
pedestrian_jpg = os.path.join(sample_dir, "pedestrian.jpg")
people_jpg = os.path.join(sample_dir, "people.jpg")
output_video = os.path.join(sample_dir, "public_people_smoke.avi")

people = cv2.imread(people_jpg)
pedestrian = cv2.imread(pedestrian_jpg)
if people is None or pedestrian is None:
    raise RuntimeError("failed to load public sample images")

w, h = 960, 540
black = np.zeros((h, w, 3), dtype=np.uint8)
frames = []
for image in [people] * 24 + [black] * 12 + [pedestrian] * 24:
    if image.shape[:2] != (h, w):
        frame = cv2.resize(image, (w, h))
    else:
        frame = image.copy()
    frames.append(frame)

writer = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*"MJPG"), 12.0, (w, h))
if not writer.isOpened():
    raise RuntimeError("failed to create smoke video")
for frame in frames:
    writer.write(frame)
writer.release()

cap = cv2.VideoCapture(output_video)
if not cap.isOpened():
    raise RuntimeError("failed to reopen smoke video")
ok, frame = cap.read()
cap.release()
if not ok or frame is None:
    raise RuntimeError("smoke video has no readable frame")

print(output_video)
PY

echo "[PASS] public people smoke video ready: ${OUTPUT_VIDEO}"
