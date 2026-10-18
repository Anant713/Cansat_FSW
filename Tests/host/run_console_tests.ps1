param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/console -ITests `
        Tests/host/console/test_console.c Tests/Console.c -o build/driver-tools/test_console.exe
    if ($LASTEXITCODE) { throw 'Console test compilation failed' }
    & ./build/driver-tools/test_console.exe
    if ($LASTEXITCODE) { throw 'Console tests failed' }
} finally { Pop-Location }
