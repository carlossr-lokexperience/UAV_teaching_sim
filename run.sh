#!/bin/sh
# No instala paquetes, no termina procesos ajenos y no modifica PX4/Gazebo.
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
if [ -x .venv/bin/python ]; then
    exec .venv/bin/python main.py "$@"
fi
exec python3 main.py "$@"
