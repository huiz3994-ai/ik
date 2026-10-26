#!/usr/bin/env bash
set -euo pipefail

# Compatibility wrapper only. All behavior lives in start_sampling_web.sh.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
    "")
        exec "${SCRIPT_DIR}/start_sampling_web.sh"
        ;;
    --dry-run)
        [[ "$#" -eq 1 ]] || { echo "FAIL: unexpected extra arguments." >&2; exit 2; }
        exec "${SCRIPT_DIR}/start_sampling_web.sh" --dry-run
        ;;
    --help|-h)
        cat <<EOF
Compatibility wrapper. Prefer: ${SCRIPT_DIR}/start_sampling_web.sh
No confirmation argument is required.
EOF
        ;;
    *) echo "FAIL: unexpected argument: $1" >&2; exit 2 ;;
esac
