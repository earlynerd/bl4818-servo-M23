param([ValidateSet('Apply','Restore')][string]$Mode = 'Apply')
$ErrorActionPreference = 'Stop'
$targets = @(
    @{ Port = 'COM29'; Identity = 'VID_0403+PID_6001+A10JO345A' },
    @{ Port = 'COM48'; Identity = 'VID_0403+PID_6001+BG03STIBA' }
)
$backupPath = Join-Path $PSScriptRoot 'latency_backup.json'
$resultPath = Join-Path $PSScriptRoot ('setting_' + $Mode.ToLower() + '_result.json')
$openedKeys = @()
$writtenKeys = @()
try {
    foreach ($target in $targets) {
        $keyPath = 'SYSTEM\CurrentControlSet\Enum\FTDIBUS\' + $target.Identity + '\0000\Device Parameters'
        $key = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey($keyPath, $true)
        if ($null -eq $key) { throw "Cannot open exact adapter key for $($target.Port)" }
        $openedKeys += @{ Target = $target; Key = $key; Before = $key.GetValue('LatencyTimer') }
        if ($key.GetValue('PortName') -ne $target.Port) { throw "Port identity mismatch for $($target.Port)" }
        if ($key.GetValueKind('LatencyTimer') -ne [Microsoft.Win32.RegistryValueKind]::DWord) { throw 'Unexpected registry value type' }
    }
    if ($Mode -eq 'Apply') {
        if (Test-Path -LiteralPath $backupPath) { throw 'Backup already exists; refusing to overwrite experiment provenance' }
        if (@($openedKeys | Where-Object { $_.Before -ne 16 }).Count -ne 0) { throw 'Baseline value is no longer 16 ms' }
        $backup = @($openedKeys | ForEach-Object {
            [pscustomobject]@{ Port = $_.Target.Port; Identity = $_.Target.Identity; LatencyTimer = $_.Before }
        })
        ConvertTo-Json -InputObject $backup | Set-Content -LiteralPath $backupPath -Encoding UTF8
    } else {
        $backup = Get-Content -LiteralPath $backupPath -Raw | ConvertFrom-Json
    }
    foreach ($entry in $openedKeys) {
        $value = 2
        if ($Mode -eq 'Restore') {
            $match = @($backup | Where-Object { $_.Port -eq $entry.Target.Port -and $_.Identity -eq $entry.Target.Identity })
            if ($match.Count -ne 1 -or $match[0].LatencyTimer -ne 16) { throw 'Invalid restore record' }
            $value = [int]$match[0].LatencyTimer
        }
        $entry.Key.SetValue('LatencyTimer', $value, [Microsoft.Win32.RegistryValueKind]::DWord)
        $writtenKeys += $entry
        if ($entry.Key.GetValue('LatencyTimer') -ne $value) { throw 'Registry readback mismatch' }
    }
    $result = @{ Success = $true; Mode = $Mode; Utc = [DateTime]::UtcNow.ToString('o'); Values = @($openedKeys | ForEach-Object {
        @{ Port = $_.Target.Port; Identity = $_.Target.Identity; Before = $_.Before; After = $_.Key.GetValue('LatencyTimer') }
    }) }
    $result | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $resultPath -Encoding UTF8
} catch {
    $originalError = $_.ToString()
    $rollbackErrors = @()
    foreach ($entry in $writtenKeys) {
        try { $entry.Key.SetValue('LatencyTimer', [int]$entry.Before, [Microsoft.Win32.RegistryValueKind]::DWord) }
        catch { $rollbackErrors += $_.ToString() }
    }
    @{ Success = $false; Error = $originalError; RollbackErrors = $rollbackErrors; Utc = [DateTime]::UtcNow.ToString('o') } |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $resultPath -Encoding UTF8
    exit 1
} finally {
    foreach ($entry in $openedKeys) { $entry.Key.Close() }
}
