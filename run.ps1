# Run the installed MH3 simulation with any forwarded command-line arguments.
$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot
. "$PSScriptRoot\uv.ps1"
if (-not (Find-Mh3Uv)) { throw "Run .\setup.ps1 first." }
& $Mh3Uv run --no-sync mh3-genesis @args
exit $LASTEXITCODE
