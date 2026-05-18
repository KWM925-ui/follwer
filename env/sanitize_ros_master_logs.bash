#!/usr/bin/env bash
set -euo pipefail

LOG_ROOT="${1:-}"
if [[ -z "${LOG_ROOT}" || ! -d "${LOG_ROOT}" ]]; then
  exit 0
fi

while IFS= read -r -d '' master_log; do
  if ! grep -Eq 'publisherUpdate\[.*exception=\[Errno (111|22)\] (Connection refused|Invalid argument)|\[rosmaster\.threadpool\]\[ERROR\].*Traceback \(most recent call last\):' "${master_log}"; then
    continue
  fi

  tmp_log="$(mktemp)"
  awk '
    BEGIN {skip_traceback=0}
    skip_traceback {
      if ($0 == "") {
        skip_traceback=0
      }
      next
    }
    /publisherUpdate\[.*exception=\[Errno (111|22)\] (Connection refused|Invalid argument)/ {next}
    /\[rosmaster\.threadpool\]\[ERROR\].*Traceback \(most recent call last\):/ {
      skip_traceback=1
      next
    }
    {print}
  ' "${master_log}" >"${tmp_log}"
  mv "${tmp_log}" "${master_log}"
done < <(find "${LOG_ROOT}" -type f -name master.log -print0)
