$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build/driver-tools | Out-Null
    $common = @('-std=c11', '-Wall', '-Wextra', '-include', 'Tests/host/sd_hal_mock.h',
        '-ITests/host', '-ICore/Inc', '-IFATFS/App', '-IFATFS/Target',
        '-IMiddlewares/Third_Party/FatFs/src')
    & gcc @common -Werror Tests/host/test_sd_spi.c Core/Src/sd_spi.c Core/Src/sd_diskio_spi.c -o build/driver-tools/test_sd_spi.exe
    if ($LASTEXITCODE) { throw 'SPI test compilation failed' }
    & ./build/driver-tools/test_sd_spi.exe
    if ($LASTEXITCODE) { throw 'SPI tests failed' }
    $sources = @('Tests/host/test_sd_filesystem.c', 'Core/Src/drivers/SDCard.c',
        'Core/Src/sd_functions.c', 'Core/Src/sd_diskio_spi.c',
        'FATFS/App/fatfs.c', 'FATFS/Target/user_diskio.c',
        'Middlewares/Third_Party/FatFs/src/diskio.c',
        'Middlewares/Third_Party/FatFs/src/ff_gen_drv.c',
        'Middlewares/Third_Party/FatFs/src/ff.c',
        'Middlewares/Third_Party/FatFs/src/option/syscall.c',
        'Middlewares/Third_Party/FatFs/src/option/ccsbcs.c')
    & gcc @common @sources -o build/driver-tools/test_sd_filesystem.exe
    if ($LASTEXITCODE) { throw 'Filesystem test compilation failed' }
    & ./build/driver-tools/test_sd_filesystem.exe
    if ($LASTEXITCODE) { throw 'Filesystem tests failed' }
} finally { Pop-Location }
