#!/usr/bin/env bash
set -euo pipefail

count="${1:-1000}"
package_dir="$(rospack find rm75_motion_planning)"
output="${package_dir}/benchmark/results/rm75_6fb_motion_${count}x5"
rosrun rm75_motion_planning rm75_motion_benchmark \
  --count-per-scenario "${count}" --seed 7506 --output "${output}"
python3 "${package_dir}/scripts/render_report.py" \
  "${output}.json" "${output}.md"
echo "Wrote ${output}.{csv,json,md}"
