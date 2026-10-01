# Upgrade Genesis, Python, and the compatible project dependencies together.
$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot
. "$PSScriptRoot\uv.ps1"
Initialize-Mh3Uv
& $Mh3Uv run --no-project --python 3.13 --script upgrade.py @args
exit $LASTEXITCODE
