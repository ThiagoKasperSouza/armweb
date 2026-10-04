#!/usr/bin/env bash
# Diagnose FR3 mesh loading: per-link extent, node transforms, and whether
# each visual lands near the joint origin it should hang off.
#
# Usage: ./scripts/diag_meshes.sh
set -eo pipefail
cd "$(dirname "$0")/.."

docker compose exec -T sim python3 /ws/tools/diag_fr3_links.py 2>&1 | tail -80