param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    $sources = @(
        'Tests/host/estimation/test_estimation.c',
        'Core/Src/apps/Cansat.c', 'Core/Src/apps/Estimation.c',
        'third_party/DCS_Firmware/dcs_attitude.c',
        'third_party/DCS_Firmware/dcs_nav.c',
        'third_party/DCS_Firmware/dcs_flight_config.c',
        'third_party/DCS_Firmware/dcs_sensor_transform.c',
        'third_party/DCS_Firmware/third_party/Fusion/Fusion/FusionAhrs.c',
        'third_party/DCS_Firmware/third_party/Fusion/Fusion/FusionConvention.c'
    )
    & $Compiler -DFUSION_USE_NORMAL_SQRT -std=c11 -Wall -Wextra -Werror -pedantic `
        -ITests/host/acquisition -ICore/Inc -Ithird_party/DCS_Firmware `
        -Ithird_party/DCS_Firmware/third_party/Fusion/Fusion @sources -lm `
        -o build/driver-tools/test_estimation_integration.exe
    if ($LASTEXITCODE) { throw 'Estimation integration test compilation failed' }
    & ./build/driver-tools/test_estimation_integration.exe
    if ($LASTEXITCODE) { throw 'Estimation integration tests failed' }
} finally { Pop-Location }
