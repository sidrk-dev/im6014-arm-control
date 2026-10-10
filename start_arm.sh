#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ -x .venv/bin/python ]]; then
  arm_python=.venv/bin/python
elif [[ -x /home/sid/Documents/code/im6014Workbench/.venv/bin/python ]]; then
  arm_python=/home/sid/Documents/code/im6014Workbench/.venv/bin/python
else
  arm_python=python3
fi
if ! "$arm_python" -c 'import serial' 2>/dev/null; then
  echo 'Install once: python3 -m venv .venv && .venv/bin/pip install pyserial' >&2
  exit 1
fi
arm_mode=--physical
if [[ "${1:-}" == --demo ]]; then arm_mode=--demo; shift; fi
printf 'Open http://127.0.0.1:8768/ — starts disconnected. Ctrl+C stops the server.\n'
exec "$arm_python" arm_dashboard.py "$arm_mode" "$@"
