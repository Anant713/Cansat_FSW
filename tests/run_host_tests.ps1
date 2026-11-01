$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $repoRoot '.build/host'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$testCases = @(
    @{ Name = 'attitude'; Sources = @('dcs_attitude.c', 'dcs_sensor_transform.c',
        'third_party/Fusion/Fusion/FusionAhrs.c',
        'third_party/Fusion/Fusion/FusionConvention.c', 'tests/test_dcs_attitude.c') },
    @{ Name = 'sensor_transform'; Sources = @('dcs_sensor_transform.c',
        'tests/test_dcs_sensor_transform.c') },
    @{ Name = 'nav'; Sources = @('dcs_nav.c', 'tests/test_dcs_nav.c') },
    @{ Name = 'wind'; Sources = @('dcs_wind.c', 'tests/test_dcs_wind.c') },
    @{ Name = 'dubins'; Sources = @('dcs_dubins.c', 'tests/test_dcs_dubins.c') },
    @{ Name = 'planner'; Sources = @('dcs_planner.c', 'dcs_dubins.c', 'tests/test_dcs_planner.c') }
    @{ Name = 'control'; Sources = @('dcs_control.c', 'tests/test_dcs_control.c') }
    @{ Name = 'circle_fit'; Sources = @('dcs_circle_fit.c', 'tests/test_dcs_circle_fit.c') }
    @{ Name = 'guidance'; Sources = @('dcs_guidance.c', 'dcs_planner.c',
        'dcs_dubins.c', 'tests/test_dcs_guidance.c') }
    @{ Name = 'runtime'; Sources = @('dcs_runtime.c', 'dcs_guidance.c',
        'dcs_planner.c', 'dcs_dubins.c', 'dcs_circle_fit.c', 'dcs_control.c',
        'dcs_wind.c', 'tests/test_dcs_runtime.c') }
    @{ Name = 'flight_config'; Sources = @('dcs_flight_config.c', 'dcs_attitude.c',
        'dcs_sensor_transform.c',
        'third_party/Fusion/Fusion/FusionAhrs.c',
        'third_party/Fusion/Fusion/FusionConvention.c', 'dcs_nav.c',
        'dcs_runtime.c', 'dcs_guidance.c', 'dcs_planner.c', 'dcs_dubins.c',
        'dcs_circle_fit.c', 'dcs_control.c', 'dcs_wind.c',
        'tests/test_dcs_flight_config.c') }
)

Push-Location $repoRoot
try {
    foreach ($testCase in $testCases) {
        $executable = Join-Path $buildDirectory ($testCase.Name + '_test.exe')
        $compilerArguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
            '-pedantic', '-DFUSION_USE_NORMAL_SQRT', '-I.') + $testCase.Sources +
            @('-lm', '-o', $executable)
        & gcc @compilerArguments
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($testCase.Name)" }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw "Tests failed: $($testCase.Name)" }
    }
} finally {
    Pop-Location
}
