Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot

Write-Host "Building application, manifest, LDROM, and J-Link command files..."

Push-Location $projectRoot
try {
    & make build-jlink
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware/J-Link build failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

Write-Host ""
Write-Host "Artifacts ready:"
Write-Host "  build\m2003-motor.update.bin"
Write-Host "  build\m2003-motor.manifest.bin"
Write-Host "  build\m2003-ldrom.bin"
Write-Host "  build\m2003-motor.jlink"
Write-Host "  build\m2003-firmware-verify.jlink"
Write-Host "  build\m2003-config-read.jlink"
Write-Host "Generated only; nothing was flashed."
Write-Host "The standard flash command reads and preserves CONFIG before selecting LDROM-first boot."
