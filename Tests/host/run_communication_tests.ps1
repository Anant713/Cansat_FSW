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
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/acquisition -ITests -ICore/Inc Tests/host/communication/test_telecommands.c Core/Src/apps/Cansat.c Core/Src/apps/Communication.c Core/Src/apps/Telemetry.c -lm -o build/driver-tools/test_telecommands.exe
    if ($LASTEXITCODE) { throw 'Telecommand bench test compilation failed' }
    & ./build/driver-tools/test_telecommands.exe
    if ($LASTEXITCODE) { throw 'Telecommand bench tests failed' }
    & $Compiler -DTEST_HDRM_DRONE -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/acquisition -ITests -ICore/Inc Tests/host/communication/test_telecommands.c Core/Src/apps/Cansat.c Core/Src/apps/Communication.c Core/Src/apps/Telemetry.c -lm -o build/driver-tools/test_hdrm_drone.exe
    if ($LASTEXITCODE) { throw 'HDRM drone test compilation failed' }
    & ./build/driver-tools/test_hdrm_drone.exe
    if ($LASTEXITCODE) { throw 'HDRM drone tests failed' }
} finally { Pop-Location }
