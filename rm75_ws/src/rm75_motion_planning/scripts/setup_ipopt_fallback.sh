#!/usr/bin/env bash
set -euo pipefail

venv_path="${1:-/tmp/rm75_motion_ipopt_venv}"
package_dir="$(rospack find rm75_motion_planning 2>/dev/null || true)"
if [[ -z "${package_dir}" ]]; then
  package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

python3 -m venv "${venv_path}"
"${venv_path}/bin/python" -m pip install --disable-pip-version-check \
  -r "${package_dir}/requirements-ipopt.txt"
echo "Persistent Ipopt fallback environment ready: ${venv_path}"
