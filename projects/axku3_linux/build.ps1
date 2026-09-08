# Generate the SoC and BIOS first; Vivado runs only with -SynthesizeOnly or -Build.
# Inputs: this project, ../../rtl, and the cache configured by environment.ps1.
# Outputs: source snapshots and build/<configuration> under BuildRoot.
param(
    [switch]$Build,
    [switch]$SynthesizeOnly,
    [switch]$Linux = $true,
    [ValidateSet(0, 1200, 1250, 1600, 1866, 2133, 2400)] [int]$DataRate = 0,
    [ValidateSet('serial', 'jtag_uart')] [string]$UartName = 'serial',
    [int]$UartBaudrate = 0,
    [string]$BuildVariant = '',
    [string]$LinuxDepsRoot = '',
    [string]$BuildRoot = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'environment.ps1')

if ($Build -and $SynthesizeOnly) {
    throw '-Build and -SynthesizeOnly are mutually exclusive'
}
if ($BuildVariant -notmatch '^[A-Za-z0-9_-]*$') {
    throw '-BuildVariant may contain only letters, digits, underscore, and hyphen'
}

$sourceRoot = $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $sourceRoot '..\..')).Path
$effectiveDataRate = if ($DataRate -ne 0) { $DataRate } else { 2400 }
$snapshotLeaf = if ($Linux) {
    "source-uberddr4-linux-$effectiveDataRate"
} elseif ($effectiveDataRate -eq 2400) {
    'source-uberddr4'
} else {
    "source-uberddr4-$effectiveDataRate"
}
if ($BuildVariant) { $snapshotLeaf += "-$BuildVariant" }
$snapshotRoot = Join-Path $BuildRoot $snapshotLeaf
$rtlSnapshot = Join-Path $snapshotRoot 'rtl'
$outputLeaf = if ($Linux) {
    "axku3_vexriscv_linux_uberddr4_$effectiveDataRate"
} elseif ($UartName -eq 'serial') {
    if ($effectiveDataRate -eq 2400) { 'axku3_vexriscv_uberddr4' } else { "axku3_vexriscv_uberddr4_$effectiveDataRate" }
} else {
    if ($effectiveDataRate -eq 2400) { 'axku3_vexriscv_uberddr4_jtag' } else { "axku3_vexriscv_uberddr4_jtag_$effectiveDataRate" }
}
if ($BuildVariant) { $outputLeaf += "_$BuildVariant" }
$outputDir = Join-Path $BuildRoot (Join-Path 'build' $outputLeaf)
$buildName = 'axku3_vexriscv_uberddr4'

& (Join-Path $sourceRoot 'check_environment.ps1') -BuildRoot $BuildRoot -LinuxDepsRoot $LinuxDepsRoot

if ($Linux) {
    if (-not $PSBoundParameters.ContainsKey('UartName')) { $UartName = 'serial' }
    if ($UartName -ne 'serial') { throw 'Linux hardware boot requires the physical serial UART' }
    $smpData = Join-Path $LinuxDepsRoot 'pythondata-cpu-vexriscv-smp\pythondata_cpu_vexriscv_smp\__init__.py'
    if (-not (Test-Path -LiteralPath $smpData)) {
        throw "Missing pinned VexRiscv-SMP data package: $smpData"
    }
}
if ($UartBaudrate -eq 0) { $UartBaudrate = if ($Linux) { 1000000 } else { 115200 } }

# Work from an isolated snapshot so OneDrive and source edits cannot interrupt
# a long Vivado run. Only debug-preservation attributes are stripped below.
New-Item -ItemType Directory -Force -Path $snapshotRoot | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $snapshotRoot 'constraints') | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $rtlSnapshot 'phy') | Out-Null
foreach ($name in @('axku3_platform.py', 'axku3_uberddr4.py', 'validate_generated.py')) {
    Copy-Item -Force (Join-Path $sourceRoot $name) $snapshotRoot
}
Copy-Item -Force (Join-Path $sourceRoot 'constraints\axku3_uberddr4.xdc') (Join-Path $snapshotRoot 'constraints')
foreach ($name in @('ddr4_top.v', 'ddr4_controller.v', 'ddr4_prober.v', 'ddr4_phy.v')) {
    Copy-Item -Force (Join-Path $repoRoot (Join-Path 'rtl' $name)) $rtlSnapshot
}
foreach ($name in @('ddr4_phy_native.v', 'ddr4_phy_native_adapter.v', 'ddr4_phy_native_byte.v', 'ddr4_phy_native_reset.v')) {
    Copy-Item -Force (Join-Path $repoRoot (Join-Path 'rtl\phy' $name)) (Join-Path $rtlSnapshot 'phy')
}

# The reusable controller RTL deliberately marks extensive bring-up probes.
# This Linux example has no ILA, so preserving those otherwise-dead
# nets blocks physical optimization and can create dangling-debug DRC warnings.
# Strip only MARK_DEBUG attributes from the isolated build snapshot; all RTL,
# CDC attributes, and the repository sources remain unchanged.
Get-ChildItem -LiteralPath $rtlSnapshot -Recurse -Filter '*.v' | ForEach-Object {
    $text = [IO.File]::ReadAllText($_.FullName)
    $text = $text.Replace('(* mark_debug = "true" *)', '')
    $text = $text.Replace(', mark_debug = "true"', '')
    [IO.File]::WriteAllText($_.FullName, $text, [Text.UTF8Encoding]::new($false))
}


if ($Linux) {
    $cpuSnapshot = Join-Path $snapshotRoot 'cpu-data'
    Invoke-ProjectPython -ArgumentList @(
        (Join-Path $sourceRoot 'prepare_cpu.py'), '--repository',
        (Join-Path $LinuxDepsRoot 'pythondata-cpu-vexriscv-smp'), '--output', $cpuSnapshot
    )
    $pythonPaths = @($cpuSnapshot) + $pythonPaths
}
$env:PYTHONPATH = $pythonPaths -join [IO.Path]::PathSeparator
$env:PATH = (@($softwareBin, $gccBin, $gitUnixBin, $makeBin, $vivadoBin) -join [IO.Path]::PathSeparator) + [IO.Path]::PathSeparator + $env:PATH
$env:LITEX_ENV_CC_TRIPLE = 'riscv-none-elf'
$env:PYTHONIOENCODING = 'utf-8'
$env:SHELL = 'sh.exe'
$env:MAKEFLAGS = 'SHELL=sh.exe'
$env:PYTHON = '"' + $python.Replace('\', '/') + '"'

$arguments = @(
    (Join-Path $snapshotRoot 'axku3_uberddr4.py'),
    '--output-dir', $outputDir,
    '--uberddr4-rtl-dir', $rtlSnapshot,
    '--uart-name', $UartName,
    '--uart-baudrate', $UartBaudrate,
    '--data-rate', $effectiveDataRate
)
if ($Linux) {
    $arguments += '--linux'
}

Push-Location $snapshotRoot
try {
    Invoke-ProjectPython -ArgumentList $arguments

    $validator = if ($Linux) { 'validate_linux_generated.py' } else { 'validate_generated.py' }
    if ($Linux) {
        Copy-Item -Force (Join-Path $sourceRoot $validator) $snapshotRoot
    }
    $validatorArguments = @(
        '--output-dir', $outputDir,
        '--uart-name', $UartName,
        '--reference-xdc', (Join-Path $repoRoot 'example_demo\axku3\axku3_uberddr4.xdc')
    )
    if ($Linux) {
        $validatorArguments += @(
            '--uart-baudrate', $UartBaudrate,
            '--data-rate', $effectiveDataRate
        )
    }
    Invoke-ProjectPython -ArgumentList (@((Join-Path $snapshotRoot $validator)) + $validatorArguments)

    # Stop here for the default command. The generated-file checks above catch
    # missing sources and wrong configuration before spending time on Vivado.
    if ($Build -or $SynthesizeOnly) {
        $gatewareDir = Join-Path $outputDir 'gateware'
        $vivadoScript = Join-Path $gatewareDir "build_$buildName.bat"
        if (-not (Test-Path -LiteralPath $vivadoScript)) {
            throw "Missing generated Vivado launcher: $vivadoScript"
        }
        Push-Location $gatewareDir
        try {
            if ($SynthesizeOnly) {
                $fullTcl = Join-Path $gatewareDir "$buildName.tcl"
                $synthTcl = Join-Path $gatewareDir "${buildName}_synth_only.tcl"
                $tcl = [IO.File]::ReadAllText($fullTcl)
                $marker = '# Add pre-optimize commands'
                $markerIndex = $tcl.IndexOf($marker, [StringComparison]::Ordinal)
                if ($markerIndex -lt 0) { throw "Cannot locate synthesis boundary in $fullTcl" }
                $tcl = $tcl.Substring(0, $markerIndex) + @"
report_clock_utilization -file ${buildName}_clock_utilization_synth.rpt
report_timing_summary -report_unconstrained -file ${buildName}_timing_unconstrained_synth.rpt
check_timing -verbose -file ${buildName}_check_timing_synth.rpt
quit
"@
                [IO.File]::WriteAllText($synthTcl, $tcl, [Text.UTF8Encoding]::new($false))
                & vivado.bat -mode batch -source $synthTcl
            }
            else {
                & cmd.exe /c $vivadoScript
            }
            if ($LASTEXITCODE -ne 0) { throw "Vivado build failed with exit code $LASTEXITCODE" }
            if ($SynthesizeOnly) {
                Invoke-ProjectPython -ArgumentList @(
                    (Join-Path $sourceRoot 'validate_synthesis.py'),
                    '--gateware-dir', $gatewareDir, '--data-rate', $effectiveDataRate
                )
            }
            elseif ($Build) {
                $implementationArguments = @(
                    (Join-Path $sourceRoot 'validate_implementation.py'),
                    '--gateware-dir', $gatewareDir,
                    '--uart-name', $UartName
                )
                if ($Linux) { $implementationArguments += '--linux' }
                $implementationArguments += @('--data-rate', $effectiveDataRate)
                Invoke-ProjectPython -ArgumentList $implementationArguments
            }
        }
        finally {
            Pop-Location
        }
    }
}
finally {
    Pop-Location
}
