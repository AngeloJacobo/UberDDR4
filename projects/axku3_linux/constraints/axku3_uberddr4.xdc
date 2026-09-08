################################################################################
# AXKU3 UberDDR4 native-PHY electrical and bitstream constraints.
# Pin locations and I/O standards are declared in axku3_platform.py. The values
# below are retained from the hardware-qualified UberDDR4 AXKU3 design and the
# seller's working MIG project; no ILA is included in this application build.
################################################################################

set_property INTERNAL_VREF 0.84 [get_iobanks 67]

set ddr_outputs [get_ports {ddr4_ck_p ddr4_ck_n ddr4_dqs_p[*] ddr4_dqs_n[*] ddr4_dq[*] ddr4_dm_n[*] ddr4_addr[*] ddr4_ba[*] ddr4_bg ddr4_cke ddr4_cs_n ddr4_act_n ddr4_odt}]
set_property OUTPUT_IMPEDANCE RDRV_40_40 $ddr_outputs
set_property SLEW FAST $ddr_outputs

set ddr_inputs [get_ports {ddr4_dqs_p[*] ddr4_dqs_n[*] ddr4_dq[*]}]
set_property IBUF_LOW_PWR FALSE $ddr_inputs
set_property ODT RTT_40 $ddr_inputs
set_property EQUALIZATION EQ_LEVEL2 $ddr_inputs

set ddr_data [get_ports {ddr4_dqs_p[*] ddr4_dqs_n[*] ddr4_dq[*] ddr4_dm_n[*]}]
set_property PRE_EMPHASIS RDRV_240 $ddr_data
set_property DATA_RATE DDR [get_ports {ddr4_ck_p ddr4_ck_n ddr4_dqs_p[*] ddr4_dqs_n[*] ddr4_dq[*] ddr4_dm_n[*]}]
set_property DATA_RATE SDR [get_ports {ddr4_addr[*] ddr4_ba[*] ddr4_bg ddr4_cke ddr4_cs_n ddr4_act_n ddr4_odt}]
set_property DRIVE 8 [get_ports ddr4_reset_n]

# Native-PHY controller/RIU toggle synchronizers are explicitly marked by RTL.
set_false_path -to [get_cells -hierarchical -filter {ASYNC_REG == TRUE}]

# Bitstream configuration properties only; setting these does not write flash.
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]
set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]
