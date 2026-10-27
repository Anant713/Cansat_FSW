$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $repoRoot '.build/host'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$testCases = @(
    @{ Name = 'attitude'; Sources = @('dcs_attitude.c',
        'third_party/Fusion/Fusion/FusionAhrs.c',
        'third_party/Fusion/Fusion/FusionConvention.c', 'tests/test_dcs_attitude.c') },
    @{ Name = 'nav'; Sources = @('dcs_nav.c', 'tests/test_dcs_nav.c') },
    @{ Name = 'wind'; Sources = @('dcs_wind.c', 'tests/test_dcs_wind.c') }
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
