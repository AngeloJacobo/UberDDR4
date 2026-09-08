# XSDB helper called by the Python runner: load one bitstream into FPGA SRAM.
# Match the AXKU3's KU3P explicitly; never choose an arbitrary JTAG target.
# This is volatile configuration, not an SPI flash programming operation.
if {$argc != 1} {
    error "usage: linux_hardware_trials.tcl <bitstream>"
}

set bitstream [file normalize [lindex $argv 0]]
if {![file exists $bitstream]} {
    error "bitstream not found: $bitstream"
}

connect -url tcp:localhost:3121
jtag targets -set -filter {level == 1 && name == "xcku3p"}
fpga -file $bitstream
puts "AXKU3_PROGRAM_PASS"
disconnect
exit
