Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot

Write-Host "Building firmware and regenerating J-Link script..."

Push-Location $projectRoot
try {
    & make build-jlink
}
finally {
    Pop-Location
}

$binPath = Join-Path $projectRoot "build\m2003-motor.bin"
$elfPath = Join-Path $projectRoot "build\m2003-motor.elf"
$jlinkPath = Join-Path $projectRoot "build\m2003-motor.jlink"

Write-Host ""
Write-Host "Artifacts ready:"
Write-Host "  ELF   $elfPath"
Write-Host "  BIN   $binPath"
Write-Host "  JLink $jlinkPath"
