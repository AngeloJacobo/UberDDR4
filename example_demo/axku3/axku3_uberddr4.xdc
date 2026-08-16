################################################################################
# AXKU3 DDR4 bring-up constraints
# Board: ALINX AXKU3, XCKU3P-FFVB676-2-I
# DRAM:  2 x MT40A512M16LY-062E (32-bit, DDR4-1600)
#
# Pin locations and electrical standards are taken from the board vendor's
# working DDR4 example project.  Add this file as a constraints source and use
# axku3_uberddr4 as the Vivado top-level module.
################################################################################

create_clock -period 5.000 -name sys_clk_200 -waveform {0.000 2.500} [get_ports sys_clk_p]

############################ Clock and reset #################################
set_property PACKAGE_PIN K22 [get_ports sys_clk_p]
set_property PACKAGE_PIN K23 [get_ports sys_clk_n]
set_property IOSTANDARD DIFF_SSTL12 [get_ports sys_clk_p]
set_property IOSTANDARD DIFF_SSTL12 [get_ports sys_clk_n]

# S1 / reset pushbutton, active low
set_property PACKAGE_PIN J14 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

################################# LEDs ########################################
set_property PACKAGE_PIN J12 [get_ports {led[0]}]
set_property PACKAGE_PIN H14 [get_ports {led[1]}]
set_property PACKAGE_PIN F13 [get_ports {led[2]}]
set_property PACKAGE_PIN H12 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[*]}]
set_property DRIVE 8 [get_ports {led[*]}]

################################## Fan ########################################
# AXKU3 board fan PWM control.  The board reference design drives this low
# for continuous fan operation.
set_property PACKAGE_PIN Y16 [get_ports fan_pwm]
set_property IOSTANDARD LVCMOS33 [get_ports fan_pwm]

############################ DDR4 pin locations ##############################
set_property PACKAGE_PIN G24 [get_ports ddr4_ck_p]
set_property PACKAGE_PIN G25 [get_ports ddr4_ck_n]
set_property PACKAGE_PIN J26 [get_ports ddr4_act_n]
set_property PACKAGE_PIN L25 [get_ports ddr4_reset_n]
set_property PACKAGE_PIN K26 [get_ports ddr4_bg]
set_property PACKAGE_PIN D24 [get_ports ddr4_cs_n]
set_property PACKAGE_PIN L24 [get_ports ddr4_cke]
set_property PACKAGE_PIN H24 [get_ports ddr4_odt]
set_property PACKAGE_PIN M25 [get_ports {ddr4_ba[0]}]
set_property PACKAGE_PIN F23 [get_ports {ddr4_ba[1]}]

set_property PACKAGE_PIN D26 [get_ports {ddr4_addr[0]}]
set_property PACKAGE_PIN D25 [get_ports {ddr4_addr[1]}]
set_property PACKAGE_PIN E26 [get_ports {ddr4_addr[2]}]
set_property PACKAGE_PIN C24 [get_ports {ddr4_addr[3]}]
set_property PACKAGE_PIN C26 [get_ports {ddr4_addr[4]}]
set_property PACKAGE_PIN F24 [get_ports {ddr4_addr[5]}]
set_property PACKAGE_PIN M26 [get_ports {ddr4_addr[6]}]
set_property PACKAGE_PIN B25 [get_ports {ddr4_addr[7]}]
set_property PACKAGE_PIN G26 [get_ports {ddr4_addr[8]}]
set_property PACKAGE_PIN B26 [get_ports {ddr4_addr[9]}]
set_property PACKAGE_PIN E25 [get_ports {ddr4_addr[10]}]
set_property PACKAGE_PIN H26 [get_ports {ddr4_addr[11]}]
set_property PACKAGE_PIN D23 [get_ports {ddr4_addr[12]}]
set_property PACKAGE_PIN F25 [get_ports {ddr4_addr[13]}]
set_property PACKAGE_PIN K25 [get_ports {ddr4_addr[14]}]
set_property PACKAGE_PIN E23 [get_ports {ddr4_addr[15]}]
set_property PACKAGE_PIN F22 [get_ports {ddr4_addr[16]}]

set_property PACKAGE_PIN E16 [get_ports {ddr4_dqs_p[0]}]
set_property PACKAGE_PIN E17 [get_ports {ddr4_dqs_n[0]}]
set_property PACKAGE_PIN A17 [get_ports {ddr4_dqs_p[1]}]
set_property PACKAGE_PIN A18 [get_ports {ddr4_dqs_n[1]}]
set_property PACKAGE_PIN F20 [get_ports {ddr4_dqs_p[2]}]
set_property PACKAGE_PIN E20 [get_ports {ddr4_dqs_n[2]}]
set_property PACKAGE_PIN C21 [get_ports {ddr4_dqs_p[3]}]
set_property PACKAGE_PIN B21 [get_ports {ddr4_dqs_n[3]}]

set_property PACKAGE_PIN G15 [get_ports {ddr4_dm_n[0]}]
set_property PACKAGE_PIN C18 [get_ports {ddr4_dm_n[1]}]
set_property PACKAGE_PIN H18 [get_ports {ddr4_dm_n[2]}]
set_property PACKAGE_PIN A22 [get_ports {ddr4_dm_n[3]}]

set_property PACKAGE_PIN C16 [get_ports {ddr4_dq[0]}]
set_property PACKAGE_PIN G16 [get_ports {ddr4_dq[1]}]
set_property PACKAGE_PIN D15 [get_ports {ddr4_dq[2]}]
set_property PACKAGE_PIN G17 [get_ports {ddr4_dq[3]}]
set_property PACKAGE_PIN H17 [get_ports {ddr4_dq[4]}]
set_property PACKAGE_PIN H16 [get_ports {ddr4_dq[5]}]
set_property PACKAGE_PIN D16 [get_ports {ddr4_dq[6]}]
set_property PACKAGE_PIN E15 [get_ports {ddr4_dq[7]}]
set_property PACKAGE_PIN B19 [get_ports {ddr4_dq[8]}]
set_property PACKAGE_PIN C17 [get_ports {ddr4_dq[9]}]
set_property PACKAGE_PIN B20 [get_ports {ddr4_dq[10]}]
set_property PACKAGE_PIN B15 [get_ports {ddr4_dq[11]}]
set_property PACKAGE_PIN A19 [get_ports {ddr4_dq[12]}]
set_property PACKAGE_PIN A15 [get_ports {ddr4_dq[13]}]
set_property PACKAGE_PIN A20 [get_ports {ddr4_dq[14]}]
set_property PACKAGE_PIN B17 [get_ports {ddr4_dq[15]}]
set_property PACKAGE_PIN G20 [get_ports {ddr4_dq[16]}]
set_property PACKAGE_PIN D19 [get_ports {ddr4_dq[17]}]
set_property PACKAGE_PIN D20 [get_ports {ddr4_dq[18]}]
set_property PACKAGE_PIN F19 [get_ports {ddr4_dq[19]}]
set_property PACKAGE_PIN G21 [get_ports {ddr4_dq[20]}]
set_property PACKAGE_PIN E18 [get_ports {ddr4_dq[21]}]
set_property PACKAGE_PIN D18 [get_ports {ddr4_dq[22]}]
set_property PACKAGE_PIN F18 [get_ports {ddr4_dq[23]}]
set_property PACKAGE_PIN C23 [get_ports {ddr4_dq[24]}]
set_property PACKAGE_PIN C22 [get_ports {ddr4_dq[25]}]
set_property PACKAGE_PIN A24 [get_ports {ddr4_dq[26]}]
set_property PACKAGE_PIN B22 [get_ports {ddr4_dq[27]}]
set_property PACKAGE_PIN A25 [get_ports {ddr4_dq[28]}]
set_property PACKAGE_PIN D21 [get_ports {ddr4_dq[29]}]
set_property PACKAGE_PIN B24 [get_ports {ddr4_dq[30]}]
set_property PACKAGE_PIN E21 [get_ports {ddr4_dq[31]}]

########################## DDR4 electrical standards #########################
# The AXKU3 DDR4 bank is wired for POD12 data and SSTL12 command/address.
set_property IOSTANDARD DIFF_SSTL12_DCI [get_ports ddr4_ck_p]
set_property IOSTANDARD DIFF_SSTL12_DCI [get_ports ddr4_ck_n]
set_property IOSTANDARD DIFF_POD12_DCI [get_ports {{ddr4_dqs_p[*]} {ddr4_dqs_n[*]}}]
set_property IOSTANDARD POD12_DCI [get_ports {{ddr4_dq[*]} {ddr4_dm_n[*]}}]
set_property IOSTANDARD SSTL12_DCI [get_ports {{ddr4_addr[*]} {ddr4_ba[*]} ddr4_bg ddr4_cke ddr4_cs_n ddr4_act_n ddr4_odt}]
set_property IOSTANDARD LVCMOS12 [get_ports ddr4_reset_n]
set_property DRIVE 8 [get_ports ddr4_reset_n]

# Bank 67 contains all POD12_DCI DQ receivers.  The AXKU3 vendor MIG design
# uses the bank's internally generated 0.84 V VREF; use the same setting here
# rather than the external-VREF default.  INTERNAL_VREF is bank-wide, so it is
# intentionally applied once to bank 67, not separately to individual ports.
set_property INTERNAL_VREF 0.84 [get_iobanks 67]

# Match the vendor MIG example's output and receiver tuning for this bank.
set_property OUTPUT_IMPEDANCE RDRV_40_40 [get_ports {ddr4_ck_p ddr4_ck_n {ddr4_dqs_p[*]} {ddr4_dqs_n[*]} {ddr4_dq[*]} {ddr4_dm_n[*]} {ddr4_addr[*]} {ddr4_ba[*]} ddr4_bg ddr4_cke ddr4_cs_n ddr4_act_n ddr4_odt}]
set_property SLEW FAST [get_ports {ddr4_ck_p ddr4_ck_n {ddr4_dqs_p[*]} {ddr4_dqs_n[*]} {ddr4_dq[*]} {ddr4_dm_n[*]} {ddr4_addr[*]} {ddr4_ba[*]} ddr4_bg ddr4_cke ddr4_cs_n ddr4_act_n ddr4_odt}]
# ddr4_dm_n is output-only in this design (read DBI is not implemented), so
# receiver-only attributes must apply only to the bidirectional DQ/DQS pins.
set_property IBUF_LOW_PWR false [get_ports {{ddr4_dqs_p[*]} {ddr4_dqs_n[*]} {ddr4_dq[*]}}]
set_property ODT RTT_40 [get_ports {{ddr4_dqs_p[*]} {ddr4_dqs_n[*]} {ddr4_dq[*]}}]
set_property EQUALIZATION EQ_LEVEL2 [get_ports {{ddr4_dqs_p[*]} {ddr4_dqs_n[*]} {ddr4_dq[*]}}]
set_property PRE_EMPHASIS RDRV_240 [get_ports {{ddr4_dqs_p[*]} {ddr4_dqs_n[*]} {ddr4_dq[*]} {ddr4_dm_n[*]}}]
set_property DATA_RATE SDR [get_ports {{ddr4_addr[*]} {ddr4_ba[*]} ddr4_bg ddr4_cke ddr4_cs_n ddr4_act_n ddr4_odt}]
set_property DATA_RATE DDR [get_ports {ddr4_ck_p ddr4_ck_n {ddr4_dqs_p[*]} {ddr4_dqs_n[*]} {ddr4_dq[*]} {ddr4_dm_n[*]}}]

############################ Bitstream settings ##############################
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]
# AXKU3 VCCO_0 and its MT25QU256 QSPI devices use the 1.8 V rail.  KU3P
# therefore supports only the grounded CFGBVS selection for this package.
set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]

# The only controller-to-ref_clk crossing is the monotonic IDELAYCTRL release
# request.  rtl/ddr4_phy.v receives it with an ASYNC_REG two-flop synchronizer;
# do not time an asynchronous CDC path into either synchronizer stage.
set_false_path -to [get_cells -hierarchical -filter {ASYNC_REG == TRUE}]
