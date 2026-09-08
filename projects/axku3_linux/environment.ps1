# Shared host paths for the PowerShell entry scripts; dot-source, do not launch.
# local.ps1 overrides defaults; explicit BuildRoot/LinuxDepsRoot arguments take
# precedence. Variables and Python search paths affect only this PowerShell
# process and its children, not system-wide settings.
$settings = @{
    Python = 'python.exe'
    VivadoRoot = 'C:\Xilinx\Vivado\2022.2'
    GitRoot = 'C:\Program Files\Git'
    # Everything downloaded/generated for this example lives in its ignored
    # build directory by default, regardless of the caller's working directory.
    CacheRoot = (Join-Path $PSScriptRoot 'build')
}
$localFile = Join-Path $PSScriptRoot 'local.ps1'
if (Test-Path -LiteralPath $localFile) {
    $localSettings = & $localFile
    if ($localSettings -isnot [hashtable]) { throw 'local.ps1 must return a settings hashtable.' }
    foreach ($key in $localSettings.Keys) { $settings[$key] = $localSettings[$key] }
}
if (-not $BuildRoot) { $BuildRoot = Join-Path $settings.CacheRoot 'output' }
if (-not $LinuxDepsRoot) { $LinuxDepsRoot = Join-Path $settings.CacheRoot 'linux' }
if ($settings.ContainsKey('LinuxDepsRoot') -and -not $PSBoundParameters.ContainsKey('LinuxDepsRoot')) {
    $LinuxDepsRoot = $settings.LinuxDepsRoot
}
$depsRoot = if ($settings.ContainsKey('DependenciesRoot')) { $settings.DependenciesRoot } else { Join-Path $settings.CacheRoot 'dependencies' }
$toolsRoot = if ($settings.ContainsKey('ToolsRoot')) { $settings.ToolsRoot } else { Join-Path $settings.CacheRoot 'tools' }
foreach ($cachePath in @($BuildRoot, $LinuxDepsRoot, $depsRoot, $toolsRoot)) {
    if ($cachePath -match '\s') { throw 'Use cache/build paths without spaces; set CacheRoot in local.ps1.' }
}
$python = (Get-Command $settings.Python -ErrorAction Stop).Source
$vivadoBin = Join-Path $settings.VivadoRoot 'bin'
$makeBin = Join-Path $settings.VivadoRoot 'gnuwin\bin'
$gitUnixBin = Join-Path $settings.GitRoot 'usr\bin'
$gccBin = Join-Path $toolsRoot 'riscv-gcc\xpack-riscv-none-elf-gcc-15.2.0-1\bin'
$softwareBin = Join-Path $toolsRoot 'litex-software\Scripts'
# Prefer this project's helpers and pinned packages over global installations.
# Both CPU packages are present because the target retains a small test mode.
$pythonPaths = @(
    $PSScriptRoot,
    (Join-Path $depsRoot 'python-packages-windows'),
    (Join-Path $depsRoot 'python-packages'),
    (Join-Path $toolsRoot 'litex-software\Lib\site-packages'),
    (Join-Path $depsRoot 'migen'),
    (Join-Path $depsRoot 'litex'),
    (Join-Path $depsRoot 'pythondata-cpu-vexriscv'),
    (Join-Path $depsRoot 'pythondata-software-picolibc'),
    (Join-Path $depsRoot 'pythondata-software-compiler_rt'),
    (Join-Path $LinuxDepsRoot 'pythondata-cpu-vexriscv-smp'),
    (Join-Path $LinuxDepsRoot 'python-packages')
)
$env:PYTHONPATH = $pythonPaths -join [IO.Path]::PathSeparator
$env:PYTHONIOENCODING = 'utf-8'
$env:PYTHONUNBUFFERED = '1'

function Invoke-ProjectPython {
    # Batch Python output must pass through a PowerShell pipe. Direct Python
    # console writes can fail with WinError 1 in some Windows terminal hosts.
    # Leave stderr separate: merging it into stdout can turn ordinary logging
    # into terminating NativeCommandError records in Windows PowerShell 5.1.
    # Do not use this for miniterm, which needs interactive terminal ownership.
    param([Parameter(Mandatory = $true)] [string[]]$ArgumentList)
    & $python @ArgumentList | Out-Host
    if ($LASTEXITCODE -ne 0) {
        throw "Python command failed with exit code ${LASTEXITCODE}: $($ArgumentList[0])"
    }
}
