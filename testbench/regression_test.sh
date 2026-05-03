#!/usr/bin/env bash
#
# regression_test.sh — UberDDR4 calibration regression suite
#
# Runs multiple simulation configurations to stress-test the PHY training
# FSM with realistic fly-by delays, address mapping variants, and
# SKIP_CALIB bypass mode.
#
# Usage:
#   export XILINX_VIVADO=/path/to/Vivado/2023.1
#   ./UberDDR4/testbench/regression_test.sh          # run all tests
#   ./UberDDR4/testbench/regression_test.sh 3 5 7    # run specific tests
#
# Engineer: Angelo C. Jacobo
#
set -uo pipefail

BOLD="\033[1m"
DIM="\033[2m"
GREEN="\033[32m"
RED="\033[31m"
CYAN="\033[36m"
YELLOW="\033[33m"
RESET="\033[0m"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$REPO_ROOT/UberDDR4/testbench/regression_logs"

# Test definitions: "NAME|EXTRA_DEFINES"
# Fly-by values model real PCB CK daisy-chain routing skew (DDR4-2400, tCK=833ps)
ALL_TESTS=(
    "calib_baseline|"
    "calib_flyby_50|-d SIM_FLY_BY_DELAY=50"
    "calib_flyby_100|-d SIM_FLY_BY_DELAY=100"
    "calib_flyby_200|-d SIM_FLY_BY_DELAY=200"
    "calib_flyby_300|-d SIM_FLY_BY_DELAY=300"
    "calib_flyby_400|-d SIM_FLY_BY_DELAY=400"
    "calib_map0|-d SIM_ADDR_MAPPING=0"
    "calib_map0_flyby_200|-d SIM_ADDR_MAPPING=0 -d SIM_FLY_BY_DELAY=200"
    "skip_calib_map1|-d SIM_SKIP_CALIB=1"
    "skip_calib_map0|-d SIM_SKIP_CALIB=1 -d SIM_ADDR_MAPPING=0"
)

if [[ -z "${XILINX_VIVADO:-}" ]]; then
    echo -e "${RED}ERROR: XILINX_VIVADO is not set. Source Vivado settings64.sh first.${RESET}"
    exit 1
fi

# Select tests: all by default, or specific indices from command line
TESTS=()
if [[ $# -gt 0 ]]; then
    for idx in "$@"; do
        if [[ $idx -ge 1 && $idx -le ${#ALL_TESTS[@]} ]]; then
            TESTS+=("${ALL_TESTS[$((idx-1))]}")
        else
            echo -e "${RED}ERROR: Test index $idx out of range (1-${#ALL_TESTS[@]})${RESET}"
            exit 1
        fi
    done
else
    TESTS=("${ALL_TESTS[@]}")
fi

total=${#TESTS[@]}
mkdir -p "$LOG_DIR"

echo ""
echo -e "${BOLD}${CYAN}═══ UberDDR4 Calibration Regression Suite ═══${RESET}"
echo -e "${DIM}Vivado: $XILINX_VIVADO${RESET}"
echo -e "${DIM}Tests:  $total${RESET}"
echo ""

pass_count=0
fail_count=0
index=0

declare -a RESULTS
declare -a TIMES

for entry in "${TESTS[@]}"; do
    IFS='|' read -r NAME DEFINES <<< "$entry"
    ((index++))

    echo -e "${BOLD}[$index/$total] ${CYAN}$NAME${RESET}"
    if [[ -n "$DEFINES" ]]; then
        echo -e "  ${DIM}defines: $DEFINES${RESET}"
    fi

    cd "$REPO_ROOT"
    rm -rf xsim.dir

    export EXTRA_DEFINES="$DEFINES"

    LOG="$LOG_DIR/${NAME}.log"

    start_time=$(date +%s)
    if timeout 10m bash UberDDR4/testbench/run_xsim.sh > "$LOG" 2>&1; then
        sim_ok=true
    else
        sim_ok=false
    fi
    end_time=$(date +%s)
    elapsed=$(( end_time - start_time ))
    TIMES+=("$elapsed")

    if $sim_ok && grep -q "PASS: All.*test phases\|PASS: All 10 write" "$LOG"; then
        RESULTS+=("PASS")
        ((pass_count++))
        echo -e "  ${GREEN}PASS${RESET} (${elapsed}s)"
    else
        RESULTS+=("FAIL")
        ((fail_count++))
        echo -e "  ${RED}FAIL${RESET} (${elapsed}s)"
        if grep -q "TIMEOUT:" "$LOG"; then
            echo -e "  ${RED}  Reason: simulation timeout${RESET}"
        elif grep -q "CALIB_ERROR" "$LOG"; then
            echo -e "  ${RED}  Reason: calibration error${RESET}"
        elif grep -q "FAIL:" "$LOG"; then
            echo -e "  ${RED}  Reason: data mismatch${RESET}"
        fi
    fi

    # Extract calibration results if present
    if grep -q "CALIBRATION RESULTS" "$LOG"; then
        grep -A6 "CALIBRATION RESULTS" "$LOG" | grep -E "Gate:|Eye:|WL:|OK:|WARNING:" | while read -r line; do
            echo -e "  ${DIM}$line${RESET}"
        done
    fi

    # Rename log to indicate result
    result_tag="${RESULTS[$((index-1))]}"
    mv "$LOG" "$LOG_DIR/${result_tag}_${NAME}.log"
done

unset EXTRA_DEFINES

# Summary
echo ""
echo -e "${BOLD}═══ REGRESSION SUMMARY ═══${RESET}"
echo ""
printf "  %-30s %-6s %s\n" "TEST" "RESULT" "TIME"
printf "  %-30s %-6s %s\n" "----" "------" "----"

index=0
for entry in "${TESTS[@]}"; do
    IFS='|' read -r NAME _ <<< "$entry"
    result="${RESULTS[$index]}"
    elapsed="${TIMES[$index]}"
    if [[ "$result" == "PASS" ]]; then
        printf "  %-30s ${GREEN}%-6s${RESET} %ss\n" "$NAME" "$result" "$elapsed"
    else
        printf "  %-30s ${RED}%-6s${RESET} %ss\n" "$NAME" "$result" "$elapsed"
    fi
    ((index++))
done

echo ""
echo -e "  ${BOLD}PASS: $pass_count / $total${RESET}"
if [[ $fail_count -gt 0 ]]; then
    echo -e "  ${RED}FAIL: $fail_count / $total${RESET}"
    echo ""
    echo -e "  Logs: $LOG_DIR/"
    exit 1
else
    echo -e "  ${GREEN}All tests passed!${RESET}"
    echo ""
    exit 0
fi