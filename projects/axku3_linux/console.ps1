# Interactive terminal for Linux already loaded by boot.ps1. It does not boot,
# reset or test the board. Ctrl+] releases the COM port before another boot.
param([string]$Port = 'COM6')
$ErrorActionPreference = 'Stop'
$BuildRoot = ''; $LinuxDepsRoot = ''
. (Join-Path $PSScriptRoot 'environment.ps1')
Write-Host 'Press Enter for the Linux prompt. Ctrl+] closes the console.'
& $python -m serial.tools.miniterm $Port 1000000 --raw --eol LF
if ($LASTEXITCODE -ne 0) { throw 'Console failed. Check the COM port and close other serial programs.' }
