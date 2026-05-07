#!/bin/bash
#
# run_compile.sh — UberDDR4 build & verification sweep
#
# Runs: Yosys synthesis (controller only) → SymbiYosys formal proofs →
#       Vivado xsim simulation → summary with PASS/FAIL.
#
# Usage:
#   ./run_compile.sh              # full sweep
#   ./run_compile.sh lint         # Yosys synthesis check only
#   ./run_compile.sh formal       # formal proofs only
#   ./run_compile.sh sim          # simulation only
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS_COUNT=0
FAIL_COUNT=0
RESULTS=()

record() {
    local name="$1" status="$2"
    if [ "$status" = "PASS" ]; then
        RESULTS+=("${GREEN}PASS${NC}  $name")
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        RESULTS+=("${RED}FAIL${NC}  $name")
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
}

run_yosys() {
    echo -e "${CYAN}▸ Yosys synthesis check (controller)${NC}"
    if yosys -q -p "
        read_verilog -sv ./rtl/ddr4_controller.v;
        synth -top ddr4_controller" 2>&1; then
        record "yosys_synth" "PASS"
    else
        record "yosys_synth" "FAIL"
    fi
}

run_formal() {
    echo ""
    echo -e "${CYAN}▸ SymbiYosys formal verification (single-config)${NC}"
    rm -rf formal/ddr4_singleconfig_*

    if sby -f formal/ddr4_singleconfig.sby 2>&1; then
        for task_dir in formal/ddr4_singleconfig_*/; do
            task_name=$(basename "$task_dir")
            if [ -e "${task_dir}PASS" ]; then
                record "$task_name" "PASS"
            else
                record "$task_name" "FAIL"
            fi
        done
    else
        record "formal_singleconfig" "FAIL"
    fi
}

run_sim() {
    echo ""
    echo -e "${CYAN}▸ Vivado xsim simulation (default config)${NC}"

    if [ -z "$XILINX_VIVADO" ]; then
        echo "ERROR: XILINX_VIVADO not set. Source Vivado settings64.sh first."
        record "xsim_default" "FAIL"
        return
    fi

    cd "$SCRIPT_DIR/.."
    rm -rf xsim.dir

    if bash "$SCRIPT_DIR/testbench/run_xsim.sh" 2>&1 | tee /tmp/uberddr4_sim.log; then
        if grep -q "Simulation PASSED\|Simulation finished successfully" /tmp/uberddr4_sim.log; then
            record "xsim_default" "PASS"
        else
            record "xsim_default" "FAIL"
        fi
    else
        record "xsim_default" "FAIL"
    fi
    cd "$SCRIPT_DIR"
}

print_summary() {
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  UberDDR4 Verification Summary${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════${NC}"
    for r in "${RESULTS[@]}"; do
        echo -e "  $r"
    done
    echo ""
    if [ "$FAIL_COUNT" -eq 0 ]; then
        echo -e "  ${GREEN}ALL $PASS_COUNT CHECKS PASSED${NC}"
    else
        echo -e "  ${RED}$FAIL_COUNT FAILED${NC}, $PASS_COUNT passed"
    fi
    echo -e "${CYAN}═══════════════════════════════════════════════${NC}"
}

case "${1:-all}" in
    lint)
        run_yosys
        ;;
    formal)
        run_formal
        ;;
    sim)
        run_sim
        ;;
    all)
        run_yosys
        run_formal
        run_sim
        ;;
    *)
        echo "Usage: $0 [lint|formal|sim|all]"
        exit 1
        ;;
esac

print_summary
exit $FAIL_COUNT