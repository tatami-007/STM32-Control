param([string]$ToolchainBin = 'D:\Develop\DevEnv\GNU-tools-for-STM32\bin',
      [ValidateSet('Debug','Release')][string]$Preset = 'Debug')
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    $env:Path = "$ToolchainBin;" + $env:Path
    cmake --preset $Preset
    if ($LASTEXITCODE) { throw 'CMake configuration failed' }
    cmake --build --preset $Preset
    if ($LASTEXITCODE) { throw 'Build failed' }
} finally { Pop-Location }
