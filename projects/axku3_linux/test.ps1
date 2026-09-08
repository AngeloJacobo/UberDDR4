# Fast host-only tests: simulated bus transactions, mocked UART, and cache checks.
# Requires the pinned environment, but does not run Vivado or access the board.
param(
    [string]$LinuxDepsRoot = '',
    [string]$BuildRoot = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'environment.ps1')
$sourceRoot = $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $sourceRoot '..\..')).Path

& (Join-Path $sourceRoot 'check_environment.ps1') -BuildRoot $BuildRoot -LinuxDepsRoot $LinuxDepsRoot

$env:PYTHONPATH = $pythonPaths -join [IO.Path]::PathSeparator

Push-Location $repoRoot
try {
    Invoke-ProjectPython -ArgumentList @((Join-Path $sourceRoot 'test_target.py'))
    Invoke-ProjectPython -ArgumentList @((Join-Path $sourceRoot 'test_setup.py'))
}
finally {
    Pop-Location
}
