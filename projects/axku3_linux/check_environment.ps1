# Read-only preflight for installed tools, pinned Git revisions and imports.
# A failure stops the build early; this script does not repair or install tools.
param([string]$BuildRoot = '', [string]$LinuxDepsRoot = '')
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'environment.ps1')
$lock = Get-Content -Raw (Join-Path $PSScriptRoot 'dependencies.json') | ConvertFrom-Json
foreach ($entry in $lock.repositories.PSObject.Properties) {
    $root = if ($entry.Value.linux) { $LinuxDepsRoot } else { $depsRoot }
    $path = Join-Path $root $entry.Name
    if (-not (Test-Path -LiteralPath $path)) { throw "Missing dependency: $path. Run setup.ps1." }
    $actual = & git -c "safe.directory=$path" -C $path rev-parse HEAD
    if ($LASTEXITCODE -ne 0 -or $actual.Trim() -ne $entry.Value.commit) {
        throw "Wrong revision for $($entry.Name); expected $($entry.Value.commit)."
    }
    $dirty = & git -c "safe.directory=$path" -C $path status --porcelain --untracked-files=no
    if ($LASTEXITCODE -ne 0) { throw "Cannot inspect dependency: $path" }
    # The upstream LiteX flow can add this one CPU prefix. Accept only its exact
    # documented contents; arbitrary local edits are not a reproducible input.
    if ($dirty) {
        $allowed = ' M pythondata_cpu_vexriscv_smp/verilog/VexRiscvLitexSmpCluster_Cc1_Iw32Is4096Iy1_Dw32Ds4096Dy1_ITs4DTs4_Ood_Wm.v'
        if ($entry.Name -ne 'pythondata-cpu-vexriscv-smp' -or @($dirty).Count -ne 1 -or $dirty -cne $allowed) {
            throw "Dependency has modified tracked files: $path"
        }
        Invoke-ProjectPython -ArgumentList @((Join-Path $PSScriptRoot 'prepare_cpu.py'), '--repository', $path)
    }
}
foreach ($path in @((Join-Path $vivadoBin 'vivado.bat'), (Join-Path $makeBin 'make.exe'),
    (Join-Path $gitUnixBin 'sh.exe'), (Join-Path $gccBin 'riscv-none-elf-gcc.exe'),
    (Join-Path $softwareBin 'meson.exe'), (Join-Path $softwareBin 'ninja.exe'))) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Missing tool: $path" }
}
Invoke-ProjectPython -ArgumentList @('-c', "import sys, migen, litex, serial, colorama, requests, yaml, fdt; assert sys.version_info[:2] == (3,12), 'Use Python 3.12'; print('Python dependencies OK')")
Write-Host "Environment OK; output: $BuildRoot"
