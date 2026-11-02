param(
    [ValidateSet('Host', 'Arm')]
    [string]$Target = 'Host'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$targetName = $Target.ToLowerInvariant()
$buildDirectory = Join-Path $repoRoot ".build/$targetName/all"
$library = Join-Path $buildDirectory 'libdcs.a'

$sources = @(
    'dcs_attitude.c',
    'dcs_circle_fit.c',
    'dcs_control.c',
    'dcs_dubins.c',
    'dcs_flight_config.c',
    'dcs_guidance.c',
    'dcs_nav.c',
    'dcs_planner.c',
    'dcs_runtime.c',
    'dcs_sensor_transform.c',
    'dcs_wind.c',
    'third_party/Fusion/Fusion/FusionAhrs.c',
    'third_party/Fusion/Fusion/FusionConvention.c'
)

if ($Target -eq 'Arm') {
    $compiler = 'arm-none-eabi-gcc'
    $archiver = 'arm-none-eabi-ar'
    $targetFlags = @(
        '-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16',
        '-mfloat-abi=hard', '-ffunction-sections', '-fdata-sections'
    )
} else {
    $compiler = 'gcc'
    $archiver = 'ar'
    $targetFlags = @()
}

if (-not (Get-Command $compiler -ErrorAction SilentlyContinue)) {
    throw "Compiler not found: $compiler"
}
if (-not (Get-Command $archiver -ErrorAction SilentlyContinue)) {
    throw "Archiver not found: $archiver"
}

$commonFlags = @(
    '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
    '-DFUSION_USE_NORMAL_SQRT', '-I.'
)

New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$objects = @()

Push-Location $repoRoot
try {
    foreach ($source in $sources) {
        $objectName = (($source -replace '[\\/]', '_') -replace '\.c$', '.o')
        $object = Join-Path $buildDirectory $objectName
        & $compiler @commonFlags @targetFlags -c $source -o $object
        if ($LASTEXITCODE -ne 0) {
            throw "Compilation failed: $source"
        }
        $objects += $object
    }

    if (Test-Path -LiteralPath $library) {
        Remove-Item -LiteralPath $library
    }
    & $archiver rcs $library @objects
    if ($LASTEXITCODE -ne 0) {
        throw 'Library archive failed'
    }
} finally {
    Pop-Location
}

Write-Host "Built $library"
