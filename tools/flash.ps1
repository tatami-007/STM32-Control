param([string]$OpenOcdRoot = 'D:\Develop\DevEnv\openocd-v0.12.0-i686-w64-mingw32',
      [ValidateSet('Debug','Release')][string]$Preset = 'Debug')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$elf = Join-Path $projectRoot "build/$Preset/STM32_Control.elf"
if (!(Test-Path -LiteralPath $elf)) { throw 'Build the project first' }
Push-Location $projectRoot
try {
    $elf = $elf.Replace('\','/')
    & "$OpenOcdRoot/bin/openocd.exe" -s "$OpenOcdRoot/share/openocd/scripts" -f openocd.cfg -c 'tcl_port disabled' -c 'gdb_port disabled' -c "program `"$elf`" verify reset exit"
    if ($LASTEXITCODE) { throw 'Programming/verification failed' }
} finally { Pop-Location }
