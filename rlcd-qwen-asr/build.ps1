$ErrorActionPreference = 'Stop'
$cli = Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
$fqbn = 'esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc'
& $cli compile --fqbn $fqbn --libraries (Join-Path $PSScriptRoot 'deps') --build-path $buildDir (Join-Path $PSScriptRoot 'rlcd_qwen_asr')
if ($LASTEXITCODE -ne 0) { throw 'Compile failed' }
Write-Host 'Compiled. No board has been flashed.'
