param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/acquisition -ITests -ICore/Inc Tests/host/hils/test_hils.c Tests/hils/HilsInput.c Tests/hils/HilsBMP.c Core/Src/apps/StateMachine.c Core/Src/apps/BarometricAltitude.c Core/Src/apps/CameraControl.c Core/Src/drivers/MissionStore.c Core/Src/drivers/CalibrationStore.c -lm -o build/driver-tools/test_hils.exe
    if ($LASTEXITCODE) { throw 'HILS compilation failed' }
    & ./build/driver-tools/test_hils.exe
    if ($LASTEXITCODE) { throw 'HILS tests failed' }
} finally { Pop-Location }
