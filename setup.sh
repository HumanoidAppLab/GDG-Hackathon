#!/usr/bin/env bash
# Bootstrap uv/Python, install the frozen release, and verify the MH3.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
set -u
case "$(uname -s):$(uname -m)" in
    Linux:x86_64|Darwin:arm64) ;;
    *) echo "This release requires Linux x64, Windows x64 (setup.ps1), or an Apple Silicon Mac. See README.md." >&2; exit 1 ;;
esac
if [[ "$(uname -s)" == "Darwin" ]]; then
    macos_version="$(sw_vers -productVersion)"
    if (( ${macos_version%%.*} < 14 )); then
        echo "The pinned PyTorch wheel requires macOS 14+. Update macOS first." >&2
        exit 1
    fi
else
    libc_version="$(getconf GNU_LIBC_VERSION 2>/dev/null || true)"
    if [[ ! "$libc_version" =~ ^glibc\ ([0-9]+)\.([0-9]+) ]] ||
        (( BASH_REMATCH[1] < 2 || (BASH_REMATCH[1] == 2 && BASH_REMATCH[2] < 28) )); then
        echo "Linux requires glibc 2.28+. Alpine/musl and older distributions lack compatible wheels." >&2
        exit 1
    fi
fi
source "$project_dir/uv.sh"
mh3_ensure_uv
python_version="$(cat .python-version)"
"$MH3_UV" python install --no-bin --no-registry "$python_version"
exec "$MH3_UV" run --no-project --python "$python_version" python setup.py "$@"
