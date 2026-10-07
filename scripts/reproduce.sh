#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
MAMBA=${MAMBA:-micromamba}
MAMBA_ROOT_PREFIX=${MAMBA_ROOT_PREFIX:-$HOME/.armbench-mamba}
export MAMBA_ROOT_PREFIX
if ! command -v "$MAMBA" >/dev/null; then
 mkdir -p .local
 curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C .local bin/micromamba
 MAMBA="$PWD/.local/bin/micromamba"
fi
if ! "$MAMBA" run -n arm python -c 'import rclpy' >/dev/null 2>&1; then
 "$MAMBA" create -n arm -f conda-linux-64.explicit -y
fi
"$MAMBA" run -n arm cmake -S . -B build
"$MAMBA" run -n arm cmake --build build -j1
if ! command -v uv >/dev/null; then
 python3 -m venv .report-venv
 .report-venv/bin/python -m pip install -r report-requirements.lock
else
 test -d .report-venv || uv venv --python 3.10 .report-venv
 uv pip sync --python .report-venv/bin/python report-requirements.lock
fi
"$MAMBA" run -n arm python scripts/run_suite.py
.report-venv/bin/python -m unittest discover -s tests -v
.report-venv/bin/python scripts/report.py
.report-venv/bin/python scripts/demo.py
"$MAMBA" run -n arm env GAZEBO_MODEL_DATABASE_URI='' ./build/armbench 0 RRTConnect 1099 results/integration
.report-venv/bin/python - <<'PY'
import json
from pathlib import Path
x=json.loads(Path('results/integration/00_RRTConnect_1099.json').read_text())
assert x['tracking_pass'] and x['execution_collision_valid'] and x['collision_valid']
print('PASS actual MoveIt/Gazebo positive-control integration')
PY
