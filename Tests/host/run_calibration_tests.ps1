param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Tests/host/calibration/test_store.c Core/Src/drivers/CalibrationStore.c Core/Src/drivers/MissionStore.c -lm -o build/driver-tools/test_calibration.exe
    if ($LASTEXITCODE) { throw 'Calibration test compilation failed' }
    & ./build/driver-tools/test_calibration.exe
    if ($LASTEXITCODE) { throw 'Calibration tests failed' }
} finally { Pop-Location }
