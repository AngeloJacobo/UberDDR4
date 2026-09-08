# Interactive terminal for Linux already loaded by boot.ps1. It does not boot,
# reset or test the board. Ctrl+] releases the COM port before another boot.
param([string]$Port = 'COM6')
$ErrorActionPreference = 'Stop'
$BuildRoot = ''; $LinuxDepsRoot = ''
. (Join-Path $PSScriptRoot 'environment.ps1')
Write-Host 'Press Enter for the Linux prompt. Ctrl+] closes the console.'
# Pace every transmitted byte, including separate writes from pasted keystrokes.
# Linux 5.14 LiteUART polls a small RX FIFO; reads/output remain unrestricted.
$consoleCode = @'
import time
import serial
from serial.tools import miniterm

original_write = serial.Serial.write

def paced_write(port, data):
    for value in data:
        if original_write(port, bytes([value])) != 1:
            raise serial.SerialException('Short console write')
        time.sleep(0.005)
    return len(data)

serial.Serial.write = paced_write
miniterm.main()
'@
Write-Host 'Input is paced at up to 200 characters/second for reliable command pasting.'
& $python -c $consoleCode $Port 1000000 --raw --eol LF
if ($LASTEXITCODE -ne 0) { throw 'Console failed. Check the COM port and close other serial programs.' }
