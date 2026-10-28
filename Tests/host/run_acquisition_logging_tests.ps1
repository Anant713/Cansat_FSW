param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -U_WIN32 -ITests/host/acquisition_logging -ITests/host/acquisition -ITests -ICore/Inc -IFATFS/Target -IMiddlewares/Third_Party/FatFs/src Tests/host/acquisition_logging/test_combined.c Core/Src/apps/Cansat.c -o build/driver-tools/test_combined.exe
    if ($LASTEXITCODE) { throw 'Combined test compilation failed' }
    & ./build/driver-tools/test_combined.exe
    if ($LASTEXITCODE) { throw 'Combined tests failed' }
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -DTEST_WITH_TELEMETRY=1 -U_WIN32 -ITests/host/acquisition_logging -ITests/host/acquisition -ITests -ICore/Inc -IFATFS/Target -IMiddlewares/Third_Party/FatFs/src Tests/host/acquisition_logging/test_combined.c Core/Src/apps/Cansat.c -o build/driver-tools/test_combined_telemetry.exe
    if ($LASTEXITCODE) { throw 'Telemetry combined test compilation failed' }
    & ./build/driver-tools/test_combined_telemetry.exe
    if ($LASTEXITCODE) { throw 'Telemetry combined tests failed' }
} finally { Pop-Location }
