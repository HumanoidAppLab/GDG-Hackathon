#!/usr/bin/env bash
# Run with the hardware-selected environment created by setup.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
set -u
source "$project_dir/uv.sh"
if ! mh3_find_uv; then
    echo "uv was not found. Run ./setup.sh first." >&2
    exit 1
fi
exec "$MH3_UV" run --no-sync "$@"
