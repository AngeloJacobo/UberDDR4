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
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"

if [[ -z "${XILINX_VIVADO:-}" ]]; then
    fail "XILINX_VIVADO is not set. Source Vivado settings64.sh first."
    exit 1
fi

XVLOG="$XILINX_VIVADO/bin/xvlog"
XELAB="$XILINX_VIVADO/bin/xelab"
XSIM="$XILINX_VIVADO/bin/xsim"

EXTRA_DEFS="${EXTRA_DEFINES:-}"

if [[ ! -L "$REPO_ROOT/UberDDR4/testbench/micron/ddr4_model.sv" ]]; then
    fail "Micron DDR4 model symlinks not found in UberDDR4/testbench/micron/"
    fail "Run first:  ./UberDDR4/testbench/setup_micron_model.sh"
    exit 1
fi

if [[ "${1:-}" == "--clean" ]]; then
    step "Cleaning xsim.dir"
    rm -rf xsim.dir
fi

echo ""
echo -e "${DIM}Vivado: $XILINX_VIVADO${RESET}"

step "Compiling RTL"
"$XVLOG" -sv \
  UberDDR4/rtl/ddr4_controller.v \
  UberDDR4/rtl/ddr4_phy.v \
  UberDDR4/rtl/ddr4_prober.v \
  UberDDR4/rtl/ddr4_top.v

# Micron model defines:
#   DDR4_8G_X8   - 8Gbit x8 density/width (must match DUT DENSITY param)
#   FIXED_2400   - lock speed grade to DDR4-2400 (834ps tCK)
#   ALLOW_JITTER - relax Micron model timing checks for sim clock jitter
#   VCD_DUMP     - tell the TB to dump VCD (xsim can't do SHM)
step "Compiling simulation sources"
"$XVLOG" -sv -d DDR4_8G_X8 -d FIXED_2400 -d ALLOW_JITTER -d VCD_DUMP \
  $EXTRA_DEFS \
  -i UberDDR4/testbench/micron \
  UberDDR4/testbench/micron/arch_package.sv \
  UberDDR4/testbench/micron/proj_package.sv \
  UberDDR4/testbench/micron/interface.sv \
  UberDDR4/testbench/micron/StateTable.sv \
  UberDDR4/testbench/micron/StateTableCore.sv \
  UberDDR4/testbench/micron/MemoryArray.sv \
  UberDDR4/testbench/micron/ddr4_model.sv \
  UberDDR4/testbench/ddr4_sim_top.sv

step "Compiling Xilinx glbl"
"$XVLOG" "$XILINX_VIVADO"/data/verilog/src/glbl.v

# -L unisims_ver  - Xilinx unisim primitives (ISERDESE3, OSERDESE3, etc.)
# -L secureip     - encrypted Xilinx IP (needed by some unisim models)
# glbl            - Xilinx global reset/GTS module (always required for xsim)
step "Elaborating"
"$XELAB" -timescale 1ps/1ps \
  -L unisims_ver -L secureip ddr4_sim_top glbl -s sim_snapshot

step "Running simulation"
"$XSIM" sim_snapshot -runall 2>&1 | tee sim_result.log || true

echo ""
if grep -q "TIMEOUT:" sim_result.log; then
    fail "Simulation TIMED OUT"
    exit 1
elif grep -q "PASS: init_failed asserted as expected" sim_result.log; then
    pass "Training failure test PASSED (init_failed correctly detected)"
elif grep -q "FAIL:" sim_result.log; then
    fail "Simulation FAILED (data mismatch)"
    exit 1
elif grep -q "PASS:" sim_result.log; then
    pass "Simulation PASSED"
else
    fail "Simulation FAILED (no PASS marker found)"
    exit 1
fi
