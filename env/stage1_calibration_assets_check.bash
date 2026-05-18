#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INTRINSICS_YAML="${INTRINSICS_YAML:-${WORKSPACE_ROOT}/src/human_follow_fusion/config/camera_intrinsics.example.yaml}"
EXTRINSICS_YAML="${EXTRINSICS_YAML:-${WORKSPACE_ROOT}/src/human_follow_fusion/config/camera_mid360_extrinsics.example.yaml}"
CAMERA_BODY_YAML="${CAMERA_BODY_YAML:-${WORKSPACE_ROOT}/src/human_follow_fusion/config/camera_to_body_extrinsics.example.yaml}"

python3 "${WORKSPACE_ROOT}/src/human_follow_fusion/scripts/calibration_asset_validator.py" \
  --intrinsics "${INTRINSICS_YAML}" \
  --extrinsics "${EXTRINSICS_YAML}" \
  --camera-body "${CAMERA_BODY_YAML}"
