#!/usr/bin/env bash
# Run the installed MH3 simulation with any forwarded command-line arguments.
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -u
cd "$project_dir"
source "$project_dir/uv.sh"
mh3_find_uv || { echo "Run ./setup.sh first." >&2; exit 1; }
exec "$MH3_UV" run --no-sync mh3-genesis "$@"
