#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
LOCAL_OUTPUT_DIR="${LOCAL_OUTPUT_DIR:-/home/coco/follwer_ws/artifacts/camera_snapshots}"
SNAPSHOT_BASENAME="${SNAPSHOT_BASENAME:-stage1_remote_camera_snapshot}"
WARMUP_FRAMES="${WARMUP_FRAMES:-12}"
IMAGE_ROTATION_DEG="${IMAGE_ROTATION_DEG:-180}"
FLIP_HORIZONTAL="${FLIP_HORIZONTAL:-false}"
FLIP_VERTICAL="${FLIP_VERTICAL:-false}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

mkdir -p "${LOCAL_OUTPUT_DIR}"

timestamp="$(date +%Y%m%d_%H%M%S)"
remote_path="/tmp/${SNAPSHOT_BASENAME}_${timestamp}.jpg"
local_path="${LOCAL_OUTPUT_DIR}/${SNAPSHOT_BASENAME}_${timestamp}.jpg"

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" "python3 - <<'PY'
import os
import sys

import cv2

video_device = os.environ.get('VIDEO_DEVICE', '${VIDEO_DEVICE}')
output_path = os.environ.get('REMOTE_SNAPSHOT_PATH', '${remote_path}')
warmup_frames = int(os.environ.get('WARMUP_FRAMES', '${WARMUP_FRAMES}'))

cap = cv2.VideoCapture(video_device, cv2.CAP_V4L2)
if not cap.isOpened():
    cap.release()
    cap = cv2.VideoCapture(video_device)
if not cap.isOpened():
    print('[FAIL] cannot open %s' % video_device, file=sys.stderr)
    sys.exit(1)

frame = None
for _ in range(max(1, warmup_frames)):
    ok, frame = cap.read()
    if not ok:
        frame = None
        continue

cap.release()

if frame is None:
    print('[FAIL] no frame captured from %s' % video_device, file=sys.stderr)
    sys.exit(1)

ok = cv2.imwrite(output_path, frame)
if not ok:
    print('[FAIL] cannot write %s' % output_path, file=sys.stderr)
    sys.exit(1)

print('[snapshot_saved] %s shape=%s' % (output_path, tuple(frame.shape)))
PY"

scp "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}:${remote_path}" "${local_path}" >/dev/null
ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" "rm -f '${remote_path}'" >/dev/null

corrected_path="${local_path%.jpg}_corrected.jpg"
LOCAL_PATH="${local_path}" \
CORRECTED_PATH="${corrected_path}" \
IMAGE_ROTATION_DEG="${IMAGE_ROTATION_DEG}" \
FLIP_HORIZONTAL="${FLIP_HORIZONTAL}" \
FLIP_VERTICAL="${FLIP_VERTICAL}" \
python3 - <<'PY'
import os
import sys

import cv2

local_path = os.environ["LOCAL_PATH"]
corrected_path = os.environ["CORRECTED_PATH"]
rotation_deg = int(os.environ["IMAGE_ROTATION_DEG"])
flip_horizontal = os.environ["FLIP_HORIZONTAL"].strip().lower() == "true"
flip_vertical = os.environ["FLIP_VERTICAL"].strip().lower() == "true"

frame = cv2.imread(local_path)
if frame is None:
    print("[FAIL] cannot read %s" % local_path, file=sys.stderr)
    sys.exit(1)

if rotation_deg == 90:
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
elif rotation_deg == 180:
    frame = cv2.rotate(frame, cv2.ROTATE_180)
elif rotation_deg == 270:
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

if flip_horizontal:
    frame = cv2.flip(frame, 1)
if flip_vertical:
    frame = cv2.flip(frame, 0)

if not cv2.imwrite(corrected_path, frame):
    print("[FAIL] cannot write %s" % corrected_path, file=sys.stderr)
    sys.exit(1)
PY

echo "[snapshot_local_raw] ${local_path}"
echo "[snapshot_local_corrected] ${corrected_path}"
