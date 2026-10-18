param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    $sources = @('Tests/host/communication/test_communication.c',
        'Core/Src/apps/Cansat.c', 'Core/Src/apps/Communication.c', 'Core/Src/apps/Telemetry.c')
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/acquisition -ICore/Inc @sources -lm -o build/driver-tools/test_communication.exe
    if ($LASTEXITCODE) { throw 'Communication test compilation failed' }
    & ./build/driver-tools/test_communication.exe
    if ($LASTEXITCODE) { throw 'Communication tests failed' }
} finally { Pop-Location }
