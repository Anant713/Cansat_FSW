param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/gnss -ICore/Inc `
        Tests/host/gnss/test_gnss.c Tests/host/gnss/xbee_stubs.c Core/Src/drivers/GNSS.c Core/Src/drivers/GNSS_HAL.c `
        -lm -o build/driver-tools/test_gnss.exe
    if ($LASTEXITCODE) { throw 'GNSS test compilation failed' }
    & ./build/driver-tools/test_gnss.exe
    if ($LASTEXITCODE) { throw 'GNSS tests failed' }
} finally { Pop-Location }
