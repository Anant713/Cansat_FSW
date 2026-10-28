param([string]$Compiler = 'gcc', [switch]$Gsv, [switch]$Nav)
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    $extra = @()
    if ($Gsv) { $extra += '-DGNSS_ENABLE_GSV=1' }
    if ($Nav) { $extra += '-DGNSS_ENABLE_NAV=1' }
    & $Compiler @extra -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/gnss -ICore/Inc `
        Tests/host/gnss/test_gnss.c Tests/host/gnss/xbee_stubs.c Core/Src/drivers/GNSS.c Core/Src/drivers/GNSS_HAL.c `
        -lm -o build/driver-tools/test_gnss.exe
    if ($LASTEXITCODE) { throw 'GNSS test compilation failed' }
    & ./build/driver-tools/test_gnss.exe
    if ($LASTEXITCODE) { throw 'GNSS tests failed' }
} finally { Pop-Location }
