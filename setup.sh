#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"

# `.python-version` selects Python 3.13. Both Genesis and PyTorch are installed
# inside the project environment; no system Python packages are reused.
uv sync --no-install-package torch
if [[ "$(uname -s)" == "Darwin" ]]; then
    # The standard macOS wheel contains Apple's MPS support; CUDA-style wheel
    # indexes do not apply to Metal.
    uv pip install --reinstall-package torch "torch>=2.14"
elif ! uv pip install --reinstall-package torch "torch>=2.14" --torch-backend=auto; then
    echo "Automatic PyTorch accelerator install failed; falling back to the locked platform wheel." >&2
    uv sync
fi

set -u
uv run --no-sync python -c 'import genesis, torch; from pxr import Usd; print(f"Genesis {genesis.__version__}; PyTorch {torch.__version__}; CUDA={torch.cuda.is_available()}; HIP={torch.version.hip}; MPS={torch.backends.mps.is_available()}; USD={Usd.__name__}")'
