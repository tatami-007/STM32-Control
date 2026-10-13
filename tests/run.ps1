param([string]$Gcc='D:\Program Files\JetBrains\CLion\bin\mingw\bin\gcc.exe')
$ErrorActionPreference='Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    New-Item -ItemType Directory -Force build/host-tests | Out-Null
    & $Gcc -std=c11 -Wall -Wextra -Werror -Itests/control_host -ICore/Inc -Impu6050 tests/control_host/app_test.c Core/Src/app.c Core/Src/serial.c Core/Src/motor.c Core/Src/pid.c mpu6050/mpu6050.c mpu6050/ins.c -o build/host-tests/app_test.exe
    if($LASTEXITCODE){throw 'host build failed'}
    foreach($scenario in @('pwm','speed','angle','fault','serial')) {
        & build/host-tests/app_test.exe $scenario
        if($LASTEXITCODE){throw "test failed: $scenario"}
    }
} finally { Pop-Location }
