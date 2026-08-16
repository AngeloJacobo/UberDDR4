#!/usr/bin/env bash
#
# run_xsim.sh  -  Compile and run UberDDR4 simulation with Vivado xsim
#
# Compiles the controller RTL, Micron DDR4 behavioral model, and the
# testbench (ddr4_sim_top.sv), then elaborates and runs the simulation.
# The result is checked by grepping the log for PASS/FAIL/TIMEOUT markers
# that the testbench $display statements emit.
#
# Prerequisites:
#   source /path/to/Vivado/2023.1/settings64.sh   # sets $XILINX_VIVADO
#   ./UberDDR4/testbench/setup_micron_model.sh     # creates model symlinks
#
# Usage:
#   ./UberDDR4/testbench/run_xsim.sh          # from repo root
#   ./UberDDR4/testbench/run_xsim.sh --clean   # delete xsim.dir first
#
# Environment:
#   EXTRA_DEFINES  -  extra xvlog -d flags, e.g. "-d SIM_FLY_BY_DELAY=200"
#                     (used by regression_test.sh to sweep configurations)
#   PHY_IMPL       -  component (default) or native.  This defines the
#                     testbench's TB_USE_NATIVE_PHY selector, which drives
#                     ddr4_top.PHY_IMPL. Both implementations are compiled.
#
set -euo pipefail

BOLD="\033[1m"
DIM="\033[2m"
GREEN="\033[32m"
RED="\033[31m"
CYAN="\033[36m"
RESET="\033[0m"

step() {
    echo ""
    echo -e "${BOLD}${CYAN}> $1${RESET}"
}

pass() {
    echo -e "${BOLD}${GREEN}OK $1${RESET}"
}

fail() {
    echo -e "${BOLD}${RED}X $1${RESET}" >&2
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

CLEAN_XSIM=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)
            CLEAN_XSIM=true
            ;;
        *)
            fail "Unknown option: $1"
            exit 2
            ;;
    esac
    shift
done

if [[ -z "${XILINX_VIVADO:-}" ]]; then
    fail "XILINX_VIVADO is not set. Source Vivado settings64.sh first."
    exit 1
fi

XVLOG="$XILINX_VIVADO/bin/xvlog"
XELAB="$XILINX_VIVADO/bin/xelab"
XSIM="$XILINX_VIVADO/bin/xsim"

EXTRA_DEFS="${EXTRA_DEFINES:-}"
PHY_IMPL="${PHY_IMPL:-component}"
MICRON_DENSITY="${MICRON_DENSITY:-DDR4_8G_X8}"
MICRON_SPEED="${MICRON_SPEED:-FIXED_2400}"
SIM_CONFIG_FILE="${SIM_CONFIG_FILE:-}"
SIM_CONFIG_SOURCE=""
[[ -n "$SIM_CONFIG_FILE" && -f "$SIM_CONFIG_FILE" ]] && SIM_CONFIG_SOURCE="$SIM_CONFIG_FILE"

if [[ ! -f "$REPO_ROOT/testbench/micron/ddr4_model.sv" ]]; then
    # On Linux setup_micron_model.sh creates symbolic links.  Git Bash may
    # materialize ordinary files instead when Windows symlinks are disabled;
    # both forms are valid simulation inputs.
    fail "Micron DDR4 model sources not found in testbench/micron/"
    fail "Run first:  bash testbench/setup_micron_model.sh"
    exit 1
fi

if $CLEAN_XSIM; then
    step "Cleaning xsim.dir"
    rm -rf xsim.dir 2>/dev/null; rm -rf xsim.dir 2>/dev/null
fi

echo ""
echo -e "${DIM}Vivado: $XILINX_VIVADO${RESET}"

case "$PHY_IMPL" in
    component)
        PHY_TB_DEFINE=()
        ;;
    native)
        PHY_TB_DEFINE=(-d TB_USE_NATIVE_PHY)
        ;;
    *)
        fail "Unknown PHY_IMPL='$PHY_IMPL' (use component or native)"
        exit 1
        ;;
esac

echo -e "${DIM}PHY implementation: $PHY_IMPL${RESET}"

step "Compiling RTL"
"$XVLOG" -sv \
  $EXTRA_DEFS \
  rtl/ddr4_controller.v \
  rtl/ddr4_prober.v \
  rtl/ddr4_top.v

# Micron model defines:
#   DDR4_8G_X8   - 8Gbit x8 density/width (must match DUT DENSITY param)
#   FIXED_2400   - lock speed grade to DDR4-2400 (834ps tCK)
#   ALLOW_JITTER - relax Micron model timing checks for sim clock jitter
#   VCD_DUMP     - tell the TB to dump VCD (very slow — use only for debug)
VCD_FLAG=""
if [[ "${DUMP_VCD:-0}" == "1" ]]; then
    VCD_FLAG="-d VCD_DUMP"
fi

step "Compiling simulation sources"
"$XVLOG" -sv -d $MICRON_DENSITY -d $MICRON_SPEED -d ALLOW_JITTER $VCD_FLAG \
  "${PHY_TB_DEFINE[@]}" \
  $EXTRA_DEFS \
  -i testbench/micron \
  testbench/micron/arch_package.sv \
  testbench/micron/proj_package.sv \
  testbench/micron/interface.sv \
  testbench/micron/StateTable.sv \
  testbench/micron/StateTableCore.sv \
  testbench/micron/MemoryArray.sv \
  testbench/micron/ddr4_model.sv \
  $SIM_CONFIG_SOURCE \
  testbench/ddr4_sim_top.sv

step "Compiling Xilinx glbl"
"$XVLOG" "$XILINX_VIVADO"/data/verilog/src/glbl.v

# -L unisims_ver  - Xilinx unisim primitives (ISERDESE3, OSERDESE3, etc.)
# -L secureip     - encrypted Xilinx IP (needed by some unisim models)
# glbl            - Xilinx global reset/GTS module (always required for xsim)
step "Elaborating"
"$XELAB" -timescale 1ps/1ps \
  -L unisims_ver -L secureip ddr4_sim_top glbl -s sim_snapshot

step "Running simulation"
# Use Tcl batch mode explicitly.  On Windows, -runall can still launch an
# interactive GUI session through a saved WCFG and leave regressions paused.
"$XSIM" sim_snapshot -tclbatch testbench/xsim_batch.tcl \
    2>&1 | tee sim_result.log || true

echo ""
if grep -q "TIMEOUT:" sim_result.log; then
    fail "Simulation TIMED OUT"
    exit 1
elif grep -q "PASS: init_failed asserted as expected" sim_result.log; then
    pass "Training failure test PASSED (init_failed correctly detected)"
elif grep -q "PASS: CSR reset test" sim_result.log; then
    pass "CSR reset test PASSED"
elif grep -q "BIST FAIL:\|FATAL: o_init_failed asserted\|FAIL: rd_err\|FAIL: CSR reset test\|NATIVE_TX_DEBUG: FAIL" sim_result.log; then
    fail "Simulation FAILED (data mismatch)"
    exit 1
elif grep -q "PASS:" sim_result.log; then
    pass "Simulation PASSED"
else
    fail "Simulation FAILED (no PASS marker found)"
    exit 1
fi
