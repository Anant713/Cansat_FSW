param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/xbee -ICore/Inc `
        Tests/host/xbee/test_xbee.c Core/Src/drivers/XBee.c Core/Src/drivers/GNSS_HAL.c `
        -o build/driver-tools/test_xbee.exe
    if ($LASTEXITCODE) { throw 'XBee test compilation failed' }
    & ./build/driver-tools/test_xbee.exe
    if ($LASTEXITCODE) { throw 'XBee tests failed' }
} finally { Pop-Location }
