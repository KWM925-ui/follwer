#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
FCU_PORT="${FCU_PORT:-/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail

python3 - <<'PY'
import time

import serial

port = "${FCU_PORT}"
for baud in (57600, 115200, 230400, 460800, 921600):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        ser.reset_input_buffer()
        ser.write(b"\n")
        ser.flush()
        time.sleep(0.5)
        data = ser.read(256)
        ser.close()
        print(f"[probe] baud={baud} read_len={len(data)} head_hex={data[:32].hex()}")
    except Exception as exc:
        print(f"[probe] baud={baud} error={exc}")

ser = serial.Serial(port, 921600, timeout=1)
try:
    ser.reset_input_buffer()
    start = time.time()
    chunks = []
    while time.time() - start < 10.0:
        data = ser.read(256)
        if data:
            chunks.append(data)
    payload = b"".join(chunks)
    print(f"[probe_long] baud=921600 read_len={len(payload)} head_hex={payload[:64].hex()}")
    print(repr(payload[:128]))
finally:
    ser.close()
PY
EOF
