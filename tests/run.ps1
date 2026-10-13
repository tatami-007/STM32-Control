param([string]$Gcc = 'D:\Program Files\JetBrains\CLion\bin\mingw\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    New-Item -ItemType Directory -Force build/host-tests | Out-Null
    & $Gcc -std=c11 -Wall -Wextra -Werror -Itests/control_host -ICore/Inc tests/control_host/assessment_test.c Core/Src/control.c Core/Src/axis.c Core/Src/motor.c Core/Src/pid.c Core/Src/vofa.c -o build/host-tests/assessment_test.exe
    if ($LASTEXITCODE) { throw 'host build failed' }
    foreach ($scenario in @('pwm','speed','angle','fault','stall','serial')) {
        & build/host-tests/assessment_test.exe $scenario
        if ($LASTEXITCODE) { throw "failed: $scenario" }
    }
    & $Gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc tests/control_host/ins_test.c Core/Src/ins.c -o build/host-tests/ins_test.exe
    if ($LASTEXITCODE) { throw 'INS host build failed' }
    & build/host-tests/ins_test.exe
    if ($LASTEXITCODE) { throw 'INS test failed' }
    & $Gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc tests/control_host/imu_calibration_test.c -o build/host-tests/imu_calibration_test.exe
    if ($LASTEXITCODE) { throw 'calibration host build failed' }
    & build/host-tests/imu_calibration_test.exe
    if ($LASTEXITCODE) { throw 'calibration test failed' }
} finally { Pop-Location }
