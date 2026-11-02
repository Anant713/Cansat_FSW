param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    $sources = @('Tests/host/logging/test_logging.c', 'Core/Src/apps/Cansat.c',
        'Core/Src/apps/Telemetry.c', 'Core/Src/apps/Logging.c')
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/acquisition -ICore/Inc -IFATFS/Target -IMiddlewares/Third_Party/FatFs/src @sources -lm -o build/driver-tools/test_logging.exe
    if ($LASTEXITCODE) { throw 'Logging test compilation failed' }
    & ./build/driver-tools/test_logging.exe
    if ($LASTEXITCODE) { throw 'Logging tests failed' }
} finally { Pop-Location }
