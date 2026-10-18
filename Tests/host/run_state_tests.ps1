param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/acquisition -ICore/Inc Tests/host/state/test_state.c Core/Src/apps/StateMachine.c Core/Src/apps/BarometricAltitude.c Core/Src/apps/CameraControl.c Core/Src/drivers/MissionStore.c -lm -o build/driver-tools/test_state.exe
    if ($LASTEXITCODE) { throw 'State test compilation failed' }
    & ./build/driver-tools/test_state.exe
    if ($LASTEXITCODE) { throw 'State tests failed' }
} finally { Pop-Location }
