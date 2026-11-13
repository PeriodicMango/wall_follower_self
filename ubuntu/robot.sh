#!/usr/bin/env bash
set -eu
here="$(cd -- "$(dirname -- "$0")" && pwd)"
exec /usr/bin/python3 "$here/robot_session.py" "$@"
