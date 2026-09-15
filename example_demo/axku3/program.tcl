################################################################################
# Program the AXKU3 with the bitstream built by build.tcl
#
#   vivado -mode batch -source program.tcl -tclargs build/axku3_uberddr4.bit
#
# A .ltx of the same name is loaded as the probe file when it exists, so the
# ILA cores this design builds come up in the hardware manager. Programming is
# volatile: this writes the device, not the board's configuration flash.
################################################################################

if {[llength $argv] < 1} {
    puts "ERROR: usage: vivado -mode batch -source program.tcl -tclargs <bitstream>"
    exit 1
}

set bitstream [file normalize [lindex $argv 0]]
if {![file exists $bitstream]} {
    puts "ERROR: no such bitstream: $bitstream"
    exit 1
}
set probes [file rootname $bitstream].ltx

open_hw_manager
connect_hw_server
open_hw_target

set devices [get_hw_devices -quiet xcku3p*]
if {[llength $devices] == 0} {
    puts "ERROR: no XCKU3P on the JTAG chain: [get_hw_devices -quiet]"
    puts "Check board power and the JTAG USB cable."
    close_hw_manager
    exit 1
}
current_hw_device [lindex $devices 0]
refresh_hw_device -update_hw_probes false [current_hw_device]

set_property PROGRAM.FILE $bitstream [current_hw_device]
if {[file exists $probes]} {
    set_property PROBES.FILE $probes [current_hw_device]
} else {
    puts "NOTE: no probe file at $probes; the ILA cores will not be decoded."
}
program_hw_devices [current_hw_device]
refresh_hw_device [current_hw_device]

puts "PROGRAMMED: [current_hw_device] with $bitstream"
close_hw_manager
