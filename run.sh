#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -u
cd "$project_dir"
exec uv run --no-sync mh3-genesis "$@"
