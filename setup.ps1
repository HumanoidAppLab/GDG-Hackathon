# Bootstrap uv/Python, install the frozen release, and verify the MH3.
$ErrorActionPreference = "Stop"
if (Get-Variable PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue) {
    $PSNativeCommandUseErrorActionPreference = $false
}
Set-Location $PSScriptRoot
if ([IntPtr]::Size -ne 8) { throw "Open 64-bit Windows PowerShell, then rerun setup." }
$architecture = [System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture.ToString()
if ($architecture -ne "X64") {
    throw "This release requires Windows x64. Native Windows ARM and 32-bit Windows lack the required wheels."
}
. "$PSScriptRoot\uv.ps1"
Initialize-Mh3Uv
$pythonVersion = (Get-Content "$PSScriptRoot\.python-version" -Raw).Trim()
& $Mh3Uv run --no-project --python $pythonVersion python setup.py @args
exit $LASTEXITCODE
