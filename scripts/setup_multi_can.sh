#!/bin/bash
# Forward to the SDK multi-CAN setup script from the repository root.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

exec bash "${REPO_ROOT}/el_a3_sdk/scripts/setup_multi_can.sh" "$@"
