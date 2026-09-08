# Hardware entry point: validate the bitstream, then program/boot/check Linux.
# Close other serial terminals first. Default is one trial; -Trials 10 repeats
# the entire cycle. No synthesis or boot-flash programming occurs here.
param(
    [ValidateRange(1, 100)] [int]$Trials = 1,
    [ValidateRange(1, 256)] [int]$SoftwareMegabytes = 16,
    [ValidateRange(0, 251)] [int]$SflFrameBytes = 0,
    [ValidateRange(1, 8)] [int]$SflOutstanding = 1,
    [ValidateRange(9600, 3000000)] [int]$UartBaudrate = 1000000,
    [ValidateSet(1200, 1250, 1600, 1866, 2133, 2400)] [int]$DataRate = 2400,
    [string]$BuildVariant = '',
    [string]$Port = 'auto',
    [string]$LinuxDepsRoot = '',
    [string]$BuildRoot = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'environment.ps1')
$sourceRoot = $PSScriptRoot
if ($BuildVariant -notmatch '^[A-Za-z0-9_-]*$') { throw 'Invalid -BuildVariant' }
$variantSuffix = if ($BuildVariant) { "_$BuildVariant" } else { '' }
if ($SflFrameBytes -eq 0) {
    $SflFrameBytes = 251
}
$gateware = Join-Path $BuildRoot "build\axku3_vexriscv_linux_uberddr4_${DataRate}${variantSuffix}\gateware"
$bitstream = Join-Path $gateware 'axku3_vexriscv_uberddr4.bit'
$payload = Join-Path $BuildRoot "payload-axku3-uberddr4-${DataRate}${variantSuffix}"
$bootJson = Join-Path $payload 'boot.json'
$xsdb = (Join-Path $vivadoBin 'xsdb.bat')
$hwServer = (Join-Path $vivadoBin 'hw_server.bat')
$programTcl = Join-Path $sourceRoot 'linux_hardware_trials.tcl'

foreach ($path in @($gateware, $bitstream, $bootJson, $python, $xsdb, $hwServer, $programTcl)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Required path is absent: $path" }
}

# Do not program an image with missing reports or unreviewed timing/DRC issues.
Invoke-ProjectPython -ArgumentList @(
    (Join-Path $sourceRoot 'validate_implementation.py'), '--gateware-dir', $gateware,
    '--uart-name', 'serial', '--linux', '--data-rate', $DataRate
)

$env:PYTHONPATH = $pythonPaths -join [IO.Path]::PathSeparator
$env:PYTHONIOENCODING = 'utf-8'
$env:PYTHONUNBUFFERED = '1'

if (-not (Get-Process -Name hw_server -ErrorAction SilentlyContinue)) {
    Start-Process -FilePath $hwServer -ArgumentList '-s', 'tcp::3121' -WindowStyle Hidden
    Start-Sleep -Seconds 2
}

# Give each campaign a separate evidence directory; keep failed trial logs too.
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$outputDir = Join-Path $BuildRoot (Join-Path 'hardware' "linux_uberddr4_${DataRate}_$stamp")
Invoke-ProjectPython -ArgumentList @(
    (Join-Path $sourceRoot 'linux_hardware_trials.py'),
    '--port', $Port, '--baudrate', $UartBaudrate, '--trials', $Trials,
    '--software-megabytes', $SoftwareMegabytes, '--sfl-frame-bytes', $SflFrameBytes,
    '--sfl-outstanding', $SflOutstanding, '--data-rate', $DataRate,
    '--bitstream', $bitstream, '--boot-json', $bootJson, '--xsdb', $xsdb,
    '--program-tcl', $programTcl, '--output-dir', $outputDir
)
