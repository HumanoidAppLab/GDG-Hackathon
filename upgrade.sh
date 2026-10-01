#!/usr/bin/env bash
# Upgrade Genesis, Python, and the compatible project dependencies together.
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -u
cd "$project_dir"
source "$project_dir/uv.sh"
mh3_ensure_uv
exec "$MH3_UV" run --no-project --python 3.13 --script upgrade.py "$@"
