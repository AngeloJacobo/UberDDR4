################################################################################
# AXKU3 DDR4 bring-up example: batch synthesis, implementation and bitstream
#
# Run it through the Makefile in this directory, which starts Vivado here:
#
#   vivado -mode batch -source build.tcl [-tclargs synth allow-failing-timing]
#
#   synth                  Stop after synthesis and its reports
#   allow-failing-timing   Write a bitstream even when routed timing fails
#
# Everything this writes goes under build/ in this directory. The Clocking
# Wizard IP that README.md describes creating by hand is created here instead,
# with the same configuration; nothing else about the design changes.
################################################################################

set demo_dir  [pwd]
set build_dir [file join $demo_dir build]
set project   axku3_uberddr4
set part      xcku3p-ffvb676-2-i
set top       axku3_uberddr4

set stop_after_synthesis [expr {[lsearch -exact $argv synth] >= 0}]
set allow_failing_timing [expr {[lsearch -exact $argv allow-failing-timing] >= 0}]

set sources [list \
    [file join $demo_dir axku3_uberddr4.v] \
    [file join $demo_dir .. .. rtl ddr4_top.v] \
    [file join $demo_dir .. .. rtl ddr4_controller.v] \
    [file join $demo_dir .. .. rtl ddr4_phy.v] \
    [file join $demo_dir .. .. rtl ddr4_prober.v] \
    [file join $demo_dir .. .. rtl phy ddr4_phy_native.v] \
    [file join $demo_dir .. .. rtl phy ddr4_phy_native_adapter.v] \
    [file join $demo_dir .. .. rtl phy ddr4_phy_native_byte.v] \
    [file join $demo_dir .. .. rtl phy ddr4_phy_native_reset.v]]
set constraints [file join $demo_dir axku3_uberddr4.xdc]

# Name every missing file at once rather than failing minutes apart.
set missing {}
foreach path [concat $sources [list $constraints]] {
    if {![file exists $path]} {
        lappend missing $path
    }
}
if {[llength $missing] > 0} {
    puts "ERROR: missing build inputs:"
    foreach path $missing {
        puts "  $path"
    }
    exit 1
}

file mkdir $build_dir
cd $build_dir

create_project -force -part $part $project [file join $build_dir project]
set_property target_language Verilog [current_project]

################################################################################
# Clocking Wizard
#
# Both clocks must come from one MMCM with the same phase: UG571 Table 2-54
# requires the PLLE4 input and the BITSLICE_CONTROL RIU clock to share a source
# when RL_DLY_RNK is used. The wrapper already buffers the board oscillator with
# its own IBUFDS and BUFG, so the input is single-ended with no buffer. The
# historical port names do not describe the frequencies: ddr4_clk is the 300 MHz
# quarter-rate fabric clock and ref300_clk is the 150 MHz RIU clock.
################################################################################
create_ip -name clk_wiz -vendor xilinx.com -library ip -module_name clk_wiz_0
set_property -dict [list \
    CONFIG.PRIMITIVE {MMCM} \
    CONFIG.PRIM_SOURCE {No_buffer} \
    CONFIG.PRIM_IN_FREQ {200.000} \
    CONFIG.NUM_OUT_CLKS {2} \
    CONFIG.CLKOUT1_USED {true} \
    CONFIG.CLK_OUT1_PORT {ddr4_clk} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {300.000} \
    CONFIG.CLKOUT1_REQUESTED_PHASE {0.000} \
    CONFIG.CLKOUT1_DRIVES {BUFG} \
    CONFIG.CLKOUT2_USED {true} \
    CONFIG.CLK_OUT2_PORT {ref300_clk} \
    CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {150.000} \
    CONFIG.CLKOUT2_REQUESTED_PHASE {0.000} \
    CONFIG.CLKOUT2_DRIVES {BUFG} \
    CONFIG.USE_RESET {true} \
    CONFIG.RESET_TYPE {ACTIVE_HIGH} \
    CONFIG.RESET_PORT {reset} \
    CONFIG.USE_LOCKED {true} \
    ] [get_ips clk_wiz_0]
generate_target all [get_ips clk_wiz_0]
synth_ip [get_ips clk_wiz_0]

################################################################################
# Synthesis
#
# The constraint file carries the package pins and the ILA cores together, as
# the manual project in README.md does. Its create_debug_core commands match
# nothing until the netlist exists, so synthesis reports them as unmatched.
################################################################################
add_files -norecurse $sources
add_files -fileset constrs_1 -norecurse $constraints
set_property top $top [current_fileset]

synth_design -top $top -part $part
write_checkpoint -force post_synth.dcp
report_utilization -file post_synth_utilization.rpt
report_timing_summary -file post_synth_timing.rpt

if {$stop_after_synthesis} {
    puts "SYNTHESIS_DONE: [file join $build_dir post_synth.dcp]"
    exit 0
}

################################################################################
# Implementation
#
# The directives are named rather than left at Default because of how this
# design misses: its worst failing path carries 12 logic levels but spends two
# thirds of its delay in routing, and every failing endpoint sits inside the
# native PHY. That is placement and congestion, not logic depth, so the effort
# goes into placement and routing rather than into remapping logic.
#
# Hold is the constraint on how far this can be pushed. The routed design holds
# by +0.001 ns, so setup optimization has almost nothing to spend; both
# phys_opt_design passes therefore use hold-aware directives.
################################################################################
set opt_directive        Explore
set place_directive      ExtraTimingOpt
set phys_opt_directive   AggressiveExplore
set route_directive      Explore
set post_route_directive ExploreWithAggressiveHoldFix

opt_design      -directive $opt_directive
place_design    -directive $place_directive
phys_opt_design -directive $phys_opt_directive
route_design    -directive $route_directive

report_utilization -file post_route_utilization.rpt
report_clock_utilization -file post_route_clock_utilization.rpt
report_timing_summary -file post_route_timing.rpt
report_drc -file post_route_drc.rpt
report_cdc -file post_route_cdc.rpt

# The Design Timing Summary table carries setup, hold and pulse width together
# with their failing endpoint counts. Reading the report is how the Linux
# project's validator collects them as well; get_timing_paths cannot report
# pulse width, so there is no equivalent query to make.
proc timing_summary {path} {
    set fh [open $path r]
    set lines [split [read $fh] \n]
    close $fh
    for {set i 0} {$i < [llength $lines]} {incr i} {
        set line [lindex $lines $i]
        if {[string first "WNS(ns)" $line] >= 0
            && [string first "TNS Failing Endpoints" $line] >= 0
            && [string first "WPWS(ns)" $line] >= 0} {
            set values [lindex $lines [expr {$i + 2}]]
            if {[llength $values] == 12} {
                return $values
            }
        }
    }
    return {}
}

# A bitstream is fit to program only when all three checks pass with no failing
# endpoint. Slack alone would call a design met that has failing endpoints an
# unconstrained path hides.
proc timing_met {summary} {
    lassign $summary wns tns setup_fail setup_total \
                     whs ths hold_fail hold_total \
                     wpws tpws pw_fail pw_total
    return [expr {$wns >= 0 && $setup_fail == 0
               && $whs >= 0 && $hold_fail == 0
               && $wpws >= 0 && $pw_fail == 0}]
}

proc report_timing_state {summary} {
    lassign $summary wns tns setup_fail setup_total \
                     whs ths hold_fail hold_total \
                     wpws tpws pw_fail pw_total
    puts [format "TIMING WNS %s ns, %s of %s setup endpoints failing" \
          $wns $setup_fail $setup_total]
    puts [format "TIMING WHS %s ns, %s of %s hold endpoints failing" \
          $whs $hold_fail $hold_total]
    puts [format "TIMING WPWS %s ns, %s of %s pulse width endpoints failing" \
          $wpws $pw_fail $pw_total]
}

set summary [timing_summary post_route_timing.rpt]
if {[llength $summary] != 12} {
    puts "ERROR: could not read the routed timing summary from post_route_timing.rpt"
    exit 1
}
report_timing_state $summary

# One more physical optimization pass when the routed design still misses. This
# is the ordinary closure step for a design that lands slightly negative, not a
# search through directives for one that happens to pass. It runs with a
# hold-aware directive because the routed hold margin here is a single
# picosecond, and a pass that trades hold for setup would fail the gate below on
# the other side.
if {![timing_met $summary]} {
    puts "TIMING_RETRY: running phys_opt_design -directive $post_route_directive"
    phys_opt_design -directive $post_route_directive
    report_timing_summary -file post_route_timing.rpt
    report_drc -file post_route_drc.rpt
    set summary [timing_summary post_route_timing.rpt]
    if {[llength $summary] != 12} {
        puts "ERROR: could not read the routed timing summary after phys_opt_design"
        exit 1
    }
    report_timing_state $summary
}

write_checkpoint -force post_route.dcp

if {![timing_met $summary] && !$allow_failing_timing} {
    puts "TIMING_FAIL: the routed design misses timing; see post_route_timing.rpt"
    puts "No bitstream was written. post_route.dcp and the reports are kept."
    puts "Set ALLOW_FAILING_TIMING=1 to write one anyway, for debugging only."
    exit 1
}
if {![timing_met $summary]} {
    puts "TIMING_FAIL: writing a bitstream that misses timing, as requested."
    puts "Do not treat this bitstream as a qualified build."
}

write_bitstream -force [file join $build_dir $project.bit]
# The constraint file builds ILA cores, so the matching probe file has to be
# saved with the bitstream; the hardware manager cannot recover it afterwards.
if {[llength [get_debug_cores -quiet]] > 0} {
    write_debug_probes -force [file join $build_dir $project.ltx]
}

puts "BITSTREAM: [file join $build_dir $project.bit]"
