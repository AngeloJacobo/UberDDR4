# Turn generated csr.json plus cached Linux images into a checked UART payload.
# Run after build.ps1; output goes under BuildRoot/payload-<configuration>.
# This does not program the FPGA or open a serial port.
param(
    [ValidateSet(1200, 1250, 1600, 1866, 2133, 2400)] [int]$DataRate = 2400,
    [string]$BuildVariant = '',
    [string]$LinuxDepsRoot = '',
    [string]$BuildRoot = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'environment.ps1')
$sourceRoot = $PSScriptRoot
if ($BuildVariant -notmatch '^[A-Za-z0-9_-]*$') { throw 'Invalid -BuildVariant' }
$variantSuffix = if ($BuildVariant) { "_$BuildVariant" } else { '' }
$csrJson = Join-Path $BuildRoot "build\axku3_vexriscv_linux_uberddr4_${DataRate}${variantSuffix}\csr.json"
$imageDir = Join-Path $LinuxDepsRoot 'images-2022'
$outputDir = Join-Path $BuildRoot "payload-axku3-uberddr4-${DataRate}${variantSuffix}"
$fdtPackage = Join-Path $LinuxDepsRoot 'python-packages'

foreach ($path in @($python, $csrJson, $imageDir, $fdtPackage)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Missing Linux payload dependency: $path" }
}


Invoke-ProjectPython -ArgumentList @(
    (Join-Path $sourceRoot 'prepare_linux_payload.py'),
    '--csr-json', $csrJson, '--image-dir', $imageDir, '--output-dir', $outputDir
)
