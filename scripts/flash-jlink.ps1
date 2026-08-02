param(
    [switch]$SkipBuild,
    [string]$JLinkExe,
    [int]$Speed = 4000
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

function Assert-BinaryEqual {
    param(
        [string]$Label,
        [string]$ExpectedPath,
        [string]$ActualPath
    )

    if (-not (Test-Path -LiteralPath $ActualPath)) {
        throw "$Label verify failed: no readback at $ActualPath"
    }
    $expected = [System.IO.File]::ReadAllBytes($ExpectedPath)
    $actual = [System.IO.File]::ReadAllBytes($ActualPath)
    if ($actual.Length -lt $expected.Length) {
        throw "$Label verify failed: $($actual.Length) readback bytes, expected $($expected.Length)"
    }
    for ($i = 0; $i -lt $expected.Length; $i++) {
        if ($expected[$i] -ne $actual[$i]) {
            throw ("{0} verify failed at +0x{1:X}: expected 0x{2:X2}, got 0x{3:X2}" -f `
                $Label, $i, $expected[$i], $actual[$i])
        }
    }
    Write-Host "$Label VERIFY OK ($($expected.Length) bytes)" -ForegroundColor Green
}

function Convert-FmcReadLogToBinary {
    param(
        [string]$Label,
        [string]$LogPath,
        [string]$OutputPath,
        [int]$ExpectedWordCount
    )

    $bytes = [System.Collections.Generic.List[byte]]::new()
    $wordCount = 0
    foreach ($line in Get-Content -LiteralPath $LogPath) {
        $match = [regex]::Match(
            $line,
            '(?i)(?:0x)?4000C008\s*=\s*([0-9A-F]{8})'
        )
        if (-not $match.Success) {
            continue
        }
        $word = [Convert]::ToUInt32($match.Groups[1].Value, 16)
        $bytes.Add([byte]($word -band 0xFF))
        $bytes.Add([byte](($word -shr 8) -band 0xFF))
        $bytes.Add([byte](($word -shr 16) -band 0xFF))
        $bytes.Add([byte](($word -shr 24) -band 0xFF))
        $wordCount++
    }
    if ($wordCount -ne $ExpectedWordCount) {
        throw "$Label verify failed: found $wordCount physical FMC words, expected $ExpectedWordCount"
    }
    [System.IO.File]::WriteAllBytes($OutputPath, $bytes.ToArray())
}

function Invoke-JLinkLogged {
    param(
        [string]$Label,
        [string]$CommandFile,
        [string]$LogPath
    )

    Write-Host $Label
    [string[]]$captured = @(& $resolvedJLinkExe `
        -device Cortex-M23 -if SWD -speed $Speed -CommandFile $CommandFile 2>&1)
    $exitCode = $LASTEXITCODE
    $captured | ForEach-Object { Write-Host $_ }
    [System.IO.File]::WriteAllLines($LogPath, $captured)
    if ($exitCode -ne 0) {
        throw "J-Link failed with exit code $exitCode while running $CommandFile"
    }
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildScript = Join-Path $PSScriptRoot "build-jlink.ps1"
$commandFile = Join-Path $projectRoot "build\m2003-motor.jlink"
$verifyCommandFile = Join-Path $projectRoot "build\m2003-firmware-verify.jlink"
$configReadFile = Join-Path $projectRoot "build\m2003-config-read.jlink"
$configTool = Join-Path $PSScriptRoot "m2003_configure_ldrom.py"
$configCaptureLog = Join-Path $projectRoot "build\m2003-config-capture.log"
$configPlanFile = Join-Path $projectRoot "build\m2003-config-plan.json"
$configEraseFile = Join-Path $projectRoot "build\m2003-config-erase.jlink"
$configProgramFile = Join-Path $projectRoot "build\m2003-config-program.jlink"
$configEraseLog = Join-Path $projectRoot "build\m2003-config-erase.log"
$configProgramLog = Join-Path $projectRoot "build\m2003-config-program.log"
$verifyLog = Join-Path $projectRoot "build\m2003-firmware-verify.log"
$appVectorReadback = Join-Path $projectRoot "build\m2003-firmware-verify-app-vector-readback.bin"
$appTailReadback = Join-Path $projectRoot "build\m2003-firmware-verify-app-tail-readback.bin"
$appReadback = Join-Path $projectRoot "build\m2003-motor-app-readback.bin"
$manifestReadback = Join-Path $projectRoot "build\m2003-firmware-verify-manifest-readback.bin"
$ldromReadback = Join-Path $projectRoot "build\m2003-firmware-verify-ldrom-readback.bin"

if (-not $SkipBuild) {
    & $buildScript
}
if (-not (Test-Path -LiteralPath $commandFile)) {
    throw "Missing command file: $commandFile"
}
if (-not (Test-Path -LiteralPath $verifyCommandFile)) {
    throw "Missing command file: $verifyCommandFile"
}
if (-not (Test-Path -LiteralPath $configReadFile)) {
    throw "Missing command file: $configReadFile"
}
$resolvedJLinkExe = Resolve-JLinkExe -ExplicitPath $JLinkExe
# Prevent stale files from making a failed J-Link run appear verified.
foreach ($readback in @(
    $appReadback,
    $appVectorReadback,
    $appTailReadback,
    $manifestReadback,
    $ldromReadback,
    $verifyLog,
    $configCaptureLog,
    $configPlanFile,
    $configEraseFile,
    $configProgramFile,
    $configEraseLog,
    $configProgramLog
)) {
    if (Test-Path -LiteralPath $readback) {
        Remove-Item -LiteralPath $readback -Force
    }
}

Invoke-JLinkLogged `
    "Reading CONFIG0..2 before any write..." `
    $configReadFile `
    $configCaptureLog

& py $configTool prepare `
    --capture $configCaptureLog `
    --plan $configPlanFile `
    --erase-script $configEraseFile `
    --program-script $configProgramFile
if ($LASTEXITCODE -ne 0) {
    throw "CONFIG preservation plan failed with exit code $LASTEXITCODE"
}
$configPlan = Get-Content -LiteralPath $configPlanFile -Raw | ConvertFrom-Json

Write-Host "Programming APROM + manifest + LDROM."
& $resolvedJLinkExe -device Cortex-M23 -if SWD -speed $Speed -CommandFile $commandFile
if ($LASTEXITCODE -ne 0) {
    throw "J-Link failed with exit code $LASTEXITCODE"
}

Invoke-JLinkLogged `
    "Reading APROM, manifest, and LDROM through physical addresses..." `
    $verifyCommandFile `
    $verifyLog

$expectedAppPath = Join-Path $projectRoot "build\m2003-motor.update.bin"
$expectedApp = [System.IO.File]::ReadAllBytes($expectedAppPath)
$vectorLength = [Math]::Min(0x200, $expectedApp.Length)
Convert-FmcReadLogToBinary `
    "APROM vector page" `
    $verifyLog `
    $appVectorReadback `
    ($vectorLength / 4)

[byte[]]$vectorBytes = [System.IO.File]::ReadAllBytes($appVectorReadback)
[byte[]]$tailBytes = [byte[]]::new(0)
if (Test-Path -LiteralPath $appTailReadback) {
    $tailBytes = [System.IO.File]::ReadAllBytes($appTailReadback)
}
[byte[]]$completeAppReadback = [byte[]]::new($vectorBytes.Length + $tailBytes.Length)
[Array]::Copy($vectorBytes, 0, $completeAppReadback, 0, $vectorBytes.Length)
[Array]::Copy($tailBytes, 0, $completeAppReadback, $vectorBytes.Length, $tailBytes.Length)
[System.IO.File]::WriteAllBytes($appReadback, $completeAppReadback)

Assert-BinaryEqual "APROM" `
    $expectedAppPath `
    $appReadback
Assert-BinaryEqual "Manifest" `
    (Join-Path $projectRoot "build\m2003-motor.manifest.bin") `
    $manifestReadback
Assert-BinaryEqual "LDROM" `
    (Join-Path $projectRoot "build\m2003-ldrom.bin") `
    $ldromReadback

if ([bool]$configPlan.needs_update) {
    Invoke-JLinkLogged `
        "Erasing CONFIG under the preserved read/modify plan..." `
        $configEraseFile `
        $configEraseLog
    & py $configTool verify `
        --capture $configEraseLog `
        --plan $configPlanFile `
        --phase erased
    if ($LASTEXITCODE -ne 0) {
        throw "CONFIG erased-state verification failed with exit code $LASTEXITCODE"
    }

    Invoke-JLinkLogged `
        "Restoring CONFIG and committing LDROM-first CBS last..." `
        $configProgramFile `
        $configProgramLog
    & py $configTool verify `
        --capture $configProgramLog `
        --plan $configPlanFile `
        --phase final
    if ($LASTEXITCODE -ne 0) {
        throw "CONFIG final verification failed with exit code $LASTEXITCODE"
    }
}
else {
    Write-Host "CONFIG0.CBS already selected LDROM; CONFIG erase/write skipped." -ForegroundColor Green
}

Write-Host "Firmware stack and LDROM-first boot configuration verified." -ForegroundColor Green
Write-Host "Power-cycle before moving the shared pins from J-Link back to UART." -ForegroundColor Yellow
