param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    $sources = @('Tests/host/acquisition/test_acquisition.c',
        'Core/Src/apps/Cansat.c', 'Core/Src/apps/SensorAcquisition.c',
        'Core/Src/apps/FlightApp.c')
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/acquisition -ICore/Inc @sources -o build/driver-tools/test_acquisition.exe
    if ($LASTEXITCODE) { throw 'Acquisition test compilation failed' }
    & ./build/driver-tools/test_acquisition.exe
    if ($LASTEXITCODE) { throw 'Acquisition tests failed' }
} finally { Pop-Location }
