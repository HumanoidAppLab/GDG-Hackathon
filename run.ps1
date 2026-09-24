$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot
& uv run --no-sync mh3-genesis @args
exit $LASTEXITCODE
