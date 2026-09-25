# Run the installed MH3 simulation with any forwarded command-line arguments.
$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot
& uv run --no-sync mh3-genesis @args
exit $LASTEXITCODE
