#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<'EOF'
printf "snapshot_time=%s\n" "$(date "+%Y-%m-%d %H:%M:%S %Z")"
printf "host=%s\n" "$(hostname)"
printf "user=%s\n" "$USER"
printf "home=%s\n" "$HOME"

printf "\n[workspaces]\n"
ls -d /home/nv/*ws 2>/dev/null | sort || true

printf "\n[serial]\n"
ls -l /dev/serial/by-id 2>/dev/null || true

printf "\n[video]\n"
ls -l /dev/video* 2>/dev/null || true

printf "\n[network]\n"
ip -brief addr | sed -n "1,12p"

printf "\n[processes]\n"
pgrep -af "ego|faster_lio|livox|mavros|roslaunch|roscore" || true
EOF
