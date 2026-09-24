$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot

# `.python-version` selects Python 3.13. Both Genesis and PyTorch are installed
# inside the project environment; no system Python packages are reused.
& uv sync --no-install-package torch
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& uv pip install --reinstall-package torch "torch>=2.14" --torch-backend=auto
if ($LASTEXITCODE -ne 0) {
    Write-Warning "Automatic PyTorch accelerator install failed; falling back to the locked platform wheel."
    & uv sync
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

& uv run --no-sync python -c 'import genesis, torch; from pxr import Usd; print(f"Genesis {genesis.__version__}; PyTorch {torch.__version__}; CUDA={torch.cuda.is_available()}; HIP={torch.version.hip}; MPS={torch.backends.mps.is_available()}; USD={Usd.__name__}")'
exit $LASTEXITCODE
