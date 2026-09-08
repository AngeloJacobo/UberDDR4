# Continue a successful -SynthesizeOnly build from its saved .dcp checkpoint.
# Reuse the generated Tcl's implementation steps; do not synthesize again.
# Produces routed reports and a bitstream, then runs the hardware-use validator.
param(
    [ValidateSet(1200, 1250, 1600, 1866, 2133, 2400)] [int]$DataRate = 2400,
    [string]$BuildVariant = '',
    [string]$BuildRoot = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'environment.ps1')
$sourceRoot = $PSScriptRoot
if ($BuildVariant -notmatch '^[A-Za-z0-9_-]*$') { throw 'Invalid -BuildVariant' }
$outputLeaf = "axku3_vexriscv_linux_uberddr4_$DataRate"
if ($BuildVariant) { $outputLeaf += "_$BuildVariant" }
$gateware = Join-Path $BuildRoot (Join-Path 'build' (Join-Path $outputLeaf 'gateware'))
$stem = 'axku3_vexriscv_uberddr4'
$fullTcl = Join-Path $gateware "$stem.tcl"
$synthDcp = Join-Path $gateware "${stem}_synth.dcp"
$resumeTcl = Join-Path $gateware "${stem}_resume_implementation.tcl"

foreach ($path in @($gateware, $fullTcl, $synthDcp, $python, $vivadoBin)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Required implementation input is absent: $path" }
}

Invoke-ProjectPython -ArgumentList @(
    (Join-Path $sourceRoot 'validate_synthesis.py'),
    '--gateware-dir', $gateware, '--data-rate', $DataRate
)

$tcl = [IO.File]::ReadAllText($fullTcl)
# This boundary is supplied by the pinned LiteX Vivado backend. Refuse an
# unfamiliar script layout rather than accidentally rerunning synthesis.
$marker = '# Add pre-optimize commands'
$markerIndex = $tcl.IndexOf($marker, [StringComparison]::Ordinal)
if ($markerIndex -lt 0) { throw "Cannot locate implementation boundary in $fullTcl" }
$dcpTcl = $synthDcp.Replace('\', '/')
$resume = "open_checkpoint {$dcpTcl}`n" + $tcl.Substring($markerIndex)
[IO.File]::WriteAllText($resumeTcl, $resume, [Text.UTF8Encoding]::new($false))

$env:PATH = $vivadoBin + [IO.Path]::PathSeparator + $env:PATH
Push-Location $gateware
try {
    & vivado.bat -mode batch -source $resumeTcl
    if ($LASTEXITCODE -ne 0) { throw "Vivado implementation failed with exit code $LASTEXITCODE" }
}
finally {
    Pop-Location
}

Invoke-ProjectPython -ArgumentList @(
    (Join-Path $sourceRoot 'validate_implementation.py'), '--gateware-dir', $gateware,
    '--uart-name', 'serial', '--linux', '--data-rate', $DataRate
)
