# Run with the hardware-selected environment created by setup.
$ErrorActionPreference = "Stop"
if (Get-Variable PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue) {
    $PSNativeCommandUseErrorActionPreference = $false
}
Set-Location $PSScriptRoot
. "$PSScriptRoot\uv.ps1"
if (-not (Find-Mh3Uv)) { throw "uv was not found. Run .\setup.ps1 first." }
& $Mh3Uv run --no-sync @args
exit $LASTEXITCODE
