#!/usr/bin/env bash
# Run the installed MH3 simulation with any forwarded command-line arguments.
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -u
cd "$project_dir"
exec uv run --no-sync mh3-genesis "$@"
