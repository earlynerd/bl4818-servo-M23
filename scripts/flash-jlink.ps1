param(
    [switch]$SkipBuild,
    [string]$JLinkExe,
    [int]$Speed = 1000
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Resolve-JLinkExe {
    param([string]$ExplicitPath)

    if ($ExplicitPath) {
        $resolved = Resolve-Path -LiteralPath $ExplicitPath
        return $resolved.Path
    }

    $candidates = @(
        "C:\Program Files\SEGGER\JLink\JLink.exe"
    )

    $versioned = Get-ChildItem "C:\Program Files\SEGGER" -Directory -Filter "JLink*" -ErrorAction SilentlyContinue |
        Sort-Object Name -Descending |
        ForEach-Object { Join-Path $_.FullName "JLink.exe" }

    foreach ($candidate in ($candidates + $versioned)) {
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }

    $fromPath = Get-Command JLink.exe -ErrorAction SilentlyContinue
    if ($fromPath -and $fromPath.Source -match "\\SEGGER\\JLink") {
        return $fromPath.Source
    }

    throw "Could not find JLink.exe. Pass -JLinkExe with the full path."
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildScript = Join-Path $PSScriptRoot "build-jlink.ps1"
$commandFile = Join-Path $projectRoot "build\m2003-motor.jlink"
$binPath     = Join-Path $projectRoot "build\m2003-motor.bin"
$readbackPath = Join-Path $projectRoot "build\m2003-motor_readback.bin"

if (-not $SkipBuild) {
    & $buildScript
}

if (-not (Test-Path -LiteralPath $commandFile)) {
    throw "Missing command file: $commandFile"
}

$resolvedJLinkExe = Resolve-JLinkExe -ExplicitPath $JLinkExe

Write-Host "Flashing with:"
Write-Host "  J-Link      $resolvedJLinkExe"
Write-Host "  CommandFile $commandFile"
Write-Host "  Speed       $Speed"

& $resolvedJLinkExe -device Cortex-M23 -if SWD -speed $Speed -CommandFile $commandFile

# Verify readback against original binary
if (Test-Path -LiteralPath $readbackPath) {
    Write-Host ""
    Write-Host "Verifying flash contents..."

    $expected = [System.IO.File]::ReadAllBytes($binPath)
    $actual   = [System.IO.File]::ReadAllBytes($readbackPath)

    # Pad expected to 4-byte alignment
    $pad = (4 - ($expected.Length % 4)) % 4
    if ($pad -gt 0) {
        $expected = $expected + @([byte]0xFF) * $pad
    }

    if ($actual.Length -lt $expected.Length) {
        Write-Host "VERIFY FAIL: readback is $($actual.Length) bytes, expected $($expected.Length)" -ForegroundColor Red
        exit 1
    }

    $errors = 0
    for ($i = 0; $i -lt $expected.Length; $i += 4) {
        $exp = [BitConverter]::ToUInt32($expected, $i)
        $act = [BitConverter]::ToUInt32($actual, $i)
        if ($exp -ne $act) {
            Write-Host ("  MISMATCH @ 0x{0:X8}: expected 0x{1:X8}, got 0x{2:X8}" -f $i, $exp, $act) -ForegroundColor Red
            $errors++
            if ($errors -ge 20) {
                Write-Host "  ... (stopping after 20)" -ForegroundColor Red
                break
            }
        }
    }

    if ($errors -eq 0) {
        Write-Host "VERIFY OK ($($expected.Length) bytes)" -ForegroundColor Green
    } else {
        Write-Host "VERIFY FAILED ($errors words differ)" -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "VERIFY SKIP: no readback file (add savebin to jlink script)" -ForegroundColor Yellow
}
