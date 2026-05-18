#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/coco/follwer_ws}"
TRIALS="${TRIALS:-5}"
DETECTOR_PROBE_SEC="${DETECTOR_PROBE_SEC:-12.0}"
FULL_CHAIN_PROBE_SEC="${FULL_CHAIN_PROBE_SEC:-12.0}"

detector_trials_with_valid=0
full_chain_trials_with_detector_valid=0
full_chain_trials_with_command_valid=0

for trial in $(seq 1 "${TRIALS}"); do
  echo "[detector_trial_begin] ${trial}/${TRIALS}"
  detector_output="$(
    PROBE_SEC="${DETECTOR_PROBE_SEC}" \
    "${WORKSPACE_ROOT}/env/stage1_remote_detector_only_probe.bash"
  )"
  printf '%s\n' "${detector_output}"

  detector_valid="$(printf '%s\n' "${detector_output}" | sed -n 's/.*valid=\([0-9][0-9]*\).*/\1/p' | head -n 1)"
  detector_valid="${detector_valid:-0}"
  if [ "${detector_valid}" -gt 0 ]; then
    detector_trials_with_valid=$((detector_trials_with_valid + 1))
  fi

  echo "[full_chain_trial_begin] ${trial}/${TRIALS}"
  full_output="$(
    PROBE_SEC="${FULL_CHAIN_PROBE_SEC}" \
    "${WORKSPACE_ROOT}/env/stage1_remote_live_target_presence_probe.bash"
  )"
  printf '%s\n' "${full_output}"

  full_detector_valid="$(printf '%s\n' "${full_output}" | sed -n 's/.*detector_valid=\([0-9][0-9]*\).*/\1/p' | head -n 1)"
  full_command_valid="$(printf '%s\n' "${full_output}" | sed -n 's/.*command_valid=\([0-9][0-9]*\).*/\1/p' | head -n 1)"
  full_detector_valid="${full_detector_valid:-0}"
  full_command_valid="${full_command_valid:-0}"

  if [ "${full_detector_valid}" -gt 0 ]; then
    full_chain_trials_with_detector_valid=$((full_chain_trials_with_detector_valid + 1))
  fi
  if [ "${full_command_valid}" -gt 0 ]; then
    full_chain_trials_with_command_valid=$((full_chain_trials_with_command_valid + 1))
  fi
done

echo "[absence_repeatability_summary] detector_trials_with_valid=${detector_trials_with_valid}/${TRIALS} full_chain_trials_with_detector_valid=${full_chain_trials_with_detector_valid}/${TRIALS} full_chain_trials_with_command_valid=${full_chain_trials_with_command_valid}/${TRIALS}"
