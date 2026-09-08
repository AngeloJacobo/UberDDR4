# One-time network setup: cache dependencies, then check the local environment.
# setup_dependencies.py does the downloads; this wrapper selects Windows paths.
param([string]$BuildRoot = '', [string]$LinuxDepsRoot = '')
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'environment.ps1')
Write-Host 'Fetching pinned source/tools into the project cache. No drivers or global settings are changed.'
Invoke-ProjectPython -ArgumentList @(
    (Join-Path $PSScriptRoot 'setup_dependencies.py'), '--deps-root', $depsRoot,
    '--tools-root', $toolsRoot, '--linux-root', $LinuxDepsRoot
)
& (Join-Path $PSScriptRoot 'check_environment.ps1') -BuildRoot $BuildRoot -LinuxDepsRoot $LinuxDepsRoot
