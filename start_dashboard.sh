#!/usr/bin/env bash
set -eu
app_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
exec python3 "$app_dir/motor_dashboard.py" "$@"
