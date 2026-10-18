param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    $sources = @('Tests/host/sensors/test_sensors.c',
        'Core/Src/drivers/SensorI2C.c', 'Core/Src/drivers/BMP.c',
        'Core/Src/drivers/AHT20.c', 'Core/Src/drivers/bmi270.c',
        'Core/Src/drivers/bmi270_config.c', 'Core/Src/drivers/IIS2MDC.c')
    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic -ITests/host/sensors -ICore/Inc @sources -lm -o build/driver-tools/test_sensors.exe
    if ($LASTEXITCODE) { throw 'Sensor test compilation failed' }
    & ./build/driver-tools/test_sensors.exe
    if ($LASTEXITCODE) { throw 'Sensor tests failed' }
} finally { Pop-Location }
