#!/usr/bin/env bash
set -euo pipefail
venv_path="${1:-/tmp/rm75_casadi_venv}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv "${venv_path}"
"${venv_path}/bin/python" -m pip install --disable-pip-version-check \
  -r "${script_dir}/requirements-casadi.txt"
echo "CasADi environment ready: ${venv_path}"
