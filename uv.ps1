# Shared uv discovery for setup, run, and upgrade.
function Find-Mh3Uv {
    $localUv = Join-Path $PSScriptRoot ".uv\uv.exe"
    if (Test-Path $localUv) {
        $script:Mh3Uv = $localUv
    } else {
        $command = Get-Command uv -CommandType Application -ErrorAction SilentlyContinue
        if (-not $command) { return $false }
        $script:Mh3Uv = $command.Source
    }
    $env:MH3_UV = $script:Mh3Uv
    $env:PATH = (Split-Path $script:Mh3Uv) + [IO.Path]::PathSeparator + $env:PATH
    return $true
}

function Initialize-Mh3Uv {
    # Keep the minimum uv version compatible with the download in .python-version.
    if (Find-Mh3Uv) {
        $versionText = & $script:Mh3Uv --version
        if ($LASTEXITCODE -eq 0 -and $versionText -match '^uv (\d+\.\d+\.\d+)' -and
            [version]$Matches[1] -ge [version]'0.12.23') { return }
    }
    Write-Host "Installing uv 0.12.23 into .uv (no administrator access or shell changes)."
    $installer = Join-Path ([IO.Path]::GetTempPath()) ([IO.Path]::GetRandomFileName() + ".ps1")
    $previousInstall = $env:UV_UNMANAGED_INSTALL
    try {
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -UseBasicParsing -Uri "https://astral.sh/uv/0.12.23/install.ps1" -OutFile $installer
        $env:UV_UNMANAGED_INSTALL = Join-Path $PSScriptRoot ".uv"
        $shell = (Get-Process -Id $PID).Path
        & $shell -NoProfile -ExecutionPolicy Bypass -File $installer
        if ($LASTEXITCODE -ne 0) { throw "uv installer failed; check internet access and proxy/TLS settings." }
    } finally {
        $env:UV_UNMANAGED_INSTALL = $previousInstall
        Remove-Item $installer -ErrorAction SilentlyContinue
    }
    if (-not (Find-Mh3Uv)) { throw "uv installation did not produce an executable." }
}
