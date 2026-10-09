# Shared uv discovery for setup, run, and upgrade. Source from the repository root.
mh3_find_uv() {
    if [[ -x "$project_dir/.uv/uv" ]]; then
        MH3_UV="$project_dir/.uv/uv"
    elif command -v uv >/dev/null 2>&1; then
        MH3_UV="$(command -v uv)"
    else
        return 1
    fi
    export MH3_UV
    export PATH="$(dirname "$MH3_UV"):$PATH"
}

mh3_ensure_uv() {
    # Keep the minimum uv version compatible with the download in .python-version.
    if mh3_find_uv; then
        local version
        version="$("$MH3_UV" --version)"
        if [[ "$version" =~ ^uv\ ([0-9]+)\.([0-9]+)\.([0-9]+) ]] &&
            (( BASH_REMATCH[1] > 0 || BASH_REMATCH[2] > 12 || (BASH_REMATCH[2] == 12 && BASH_REMATCH[3] >= 23) )); then
            return
        fi
    fi
    echo "Installing uv 0.12.23 into .uv (no administrator access or shell changes)."
    local installer
    installer="$(mktemp)"
    if command -v curl >/dev/null 2>&1; then
        curl --fail --location --silent --show-error https://astral.sh/uv/0.12.23/install.sh -o "$installer" || { rm -f "$installer"; return 1; }
    elif command -v wget >/dev/null 2>&1; then
        wget -q https://astral.sh/uv/0.12.23/install.sh -O "$installer" || { rm -f "$installer"; return 1; }
    else
        echo "Install curl or wget with your OS package manager, then rerun setup." >&2
        rm -f "$installer"
        return 1
    fi
    if ! UV_UNMANAGED_INSTALL="$project_dir/.uv" sh "$installer"; then
        rm -f "$installer"
        echo "uv download failed. Check internet access, proxy settings, and trusted TLS certificates." >&2
        return 1
    fi
    rm -f "$installer"
    mh3_find_uv
}
