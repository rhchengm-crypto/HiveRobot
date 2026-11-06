$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$cli = Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
if (-not (Test-Path -LiteralPath $cli)) {
    $cli = (Get-Command arduino-cli -ErrorAction Stop).Source
}
$logRoot = Join-Path $root 'build'
New-Item -ItemType Directory -Path $logRoot -Force | Out-Null
foreach ($target in @('macro_main', 'macro_secondary')) {
    Copy-Item -LiteralPath (Join-Path $root 'common\KeyLogic.h') -Destination (Join-Path $root "$target\KeyLogic.h") -Force
    $log = Join-Path $logRoot "$target.log"
    $oldPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue' # Windows PowerShell treats native stderr as ErrorRecord.
    $targetFqbn = if ($target -eq 'macro_main') { 'esp32:esp32:esp32:PartitionScheme=min_spiffs' } else { 'esp32:esp32:esp32' }
    & $cli compile --fqbn $targetFqbn --build-path (Join-Path $logRoot $target) (Join-Path $root $target) 2>&1 | ForEach-Object { $_.ToString() } | Tee-Object -FilePath $log
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = $oldPreference
    if ($compileExit -ne 0) { throw "Build failed: $target (see $log)" }
}
Write-Host 'Both builds and constexpr logic tests passed. No boards were flashed.'
