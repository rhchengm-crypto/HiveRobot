param([switch]$ConfigureOnly,[switch]$CheckApp)
$ErrorActionPreference='Stop'
$env:PYTHONUTF8='1'
$env:PYTHONIOENCODING='utf-8'
$toolchain='C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-gpt\toolchain'
$env:IDF_PATH=Join-Path $toolchain 'esp-idf'
$env:IDF_TOOLS_PATH=Join-Path $toolchain 'tools'
$env:IDF_PYTHON_ENV_PATH=Join-Path $env:IDF_TOOLS_PATH 'python_env\idf5.5_py3.14_env'
$idfPython=Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts\python.exe'
$compiler=Join-Path $env:IDF_TOOLS_PATH 'tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin'
$env:PATH=(Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts')+';'+$compiler+';'+(Join-Path $env:IDF_PATH 'tools')+';'+$env:PATH
$env:IDF_COMPONENT_MANAGER='0'
$env:IDF_SKIP_CHECK_SUBMODULES='1'
$env:ESP_ROM_ELF_DIR=Join-Path $env:IDF_TOOLS_PATH 'tools\esp-rom-elfs\20241011'
Push-Location $PSScriptRoot
try {
 & $idfPython (Join-Path $PSScriptRoot 'configure.py')
 if($LASTEXITCODE -ne 0){throw 'Local configuration failed'}
 $operation=if($ConfigureOnly){'reconfigure'}else{'build'}
 if($CheckApp){
  & ninja -C build 'esp-idf/firmware/CMakeFiles/__idf_firmware.dir/GptGateway.cpp.obj' 'esp-idf/firmware/CMakeFiles/__idf_firmware.dir/main.cpp.obj' 'esp-idf/microlink/libmicrolink.a' 'esp-idf/wireguard_lwip/libwireguard_lwip.a'
 }else{
  & $idfPython (Join-Path $env:IDF_PATH 'tools\idf.py') '-DIDF_TARGET=esp32s3' $operation
 }
 if($LASTEXITCODE -ne 0){throw 'ESP-IDF build failed'}
}finally{Pop-Location}
