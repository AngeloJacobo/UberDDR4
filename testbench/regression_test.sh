#!/usr/bin/env bash
#
# regression_test.sh  -  UberDDR4 calibration regression suite
#
# Runs multiple simulation configurations to stress-test the PHY training
# FSM with realistic fly-by delays and address mapping variants.
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

# Each line: NAME  DDR4_CLK  DQ  FLYBY  MAP  BIST  DENSITY  MICRON_DEF     MICRON_SPEED  SPECIAL
# To add a new test configuration, just add a new line to this array.
ALL_TESTS=(
    # Core x8 DDR4-2400 sweep
    "baseline          834  8   0    1  1  8  DDR4_8G_X8   FIXED_2400  -"
    "flyby_50          834  8   50   1  1  8  DDR4_8G_X8   FIXED_2400  -"
    "flyby_100         834  8   100  1  1  8  DDR4_8G_X8   FIXED_2400  -"
    "flyby_200         834  8   200  1  1  8  DDR4_8G_X8   FIXED_2400  -"
    "flyby_300         834  8   300  1  1  8  DDR4_8G_X8   FIXED_2400  -"
    "flyby_400         834  8   400  1  1  8  DDR4_8G_X8   FIXED_2400  -"
    "map0              834  8   0    0  1  8  DDR4_8G_X8   FIXED_2400  -"
    "map0_flyby_200    834  8   200  0  1  8  DDR4_8G_X8   FIXED_2400  -"
    "bist_full         834  8   0    1  2  8  DDR4_8G_X8   FIXED_2400  -"
    # x16 PHY support requires 2 DQS/DM per lane (V2 item).
    # x16 controller logic is verified by formal multiconfig (DQ_BITS=16, BG_BITS=1).
    # Speed grade sweep
    "ddr4_1600         1250 8   0    1  1  8  DDR4_8G_X8   FIXED_1600  -"
    "ddr4_1600_flyby   1250 8   200  1  1  8  DDR4_8G_X8   FIXED_1600  -"
    "ddr4_2133         937  8   0    1  1  8  DDR4_8G_X8   FIXED_2133  -"
    "ddr4_2133_flyby   937  8   200  1  1  8  DDR4_8G_X8   FIXED_2133  -"
    # Density sweep
    "density_4g        834  8   0    1  1  4  DDR4_4G_X8   FIXED_2400  -"
    # Error path
    "train_fail        834  8   0    1  1  8  DDR4_8G_X8   FIXED_2400  TRAIN_FAIL"
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
echo -e "${BOLD}${CYAN}=== UberDDR4 Calibration Regression Suite ===${RESET}"
echo -e "${DIM}Vivado: $XILINX_VIVADO${RESET}"
echo -e "${DIM}Tests:  $total${RESET}"
echo ""

pass_count=0
fail_count=0
index=0

declare -a RESULTS
declare -a TIMES

for entry in "${TESTS[@]}"; do
    read -r NAME DDR4_CLK DQ FLYBY MAP BIST DENS MICRON_DEF MICRON_SPD SPECIAL <<< "$entry"

    DEFINES="-d SIM_DDR4_CLK_PERIOD=$DDR4_CLK"
    [[ "$DQ" != "8" ]]      && DEFINES="$DEFINES -d SIM_DQ_BITS=$DQ"
    [[ "$FLYBY" != "0" ]]   && DEFINES="$DEFINES -d SIM_FLY_BY_DELAY=$FLYBY"
    [[ "$MAP" != "1" ]]     && DEFINES="$DEFINES -d SIM_ADDR_MAPPING=$MAP"
    [[ "$BIST" != "1" ]]    && DEFINES="$DEFINES -d SIM_BIST_MODE=$BIST"
    [[ "$DENS" == "4" ]]    && DEFINES="$DEFINES -d SIM_DENSITY_4G"
    [[ "$SPECIAL" == "TRAIN_FAIL" ]] && DEFINES="$DEFINES -d SIM_FORCE_TRAIN_FAIL"

    export EXTRA_DEFINES="$DEFINES"
    export MICRON_DENSITY="$MICRON_DEF"
    export MICRON_SPEED="$MICRON_SPD"

    ((index++))

    echo -e "${BOLD}[$index/$total] ${CYAN}$NAME${RESET}"
    if [[ -n "$DEFINES" ]]; then
        echo -e "  ${DIM}defines: $DEFINES${RESET}"
    fi

    cd "$REPO_ROOT"
    rm -rf xsim.dir

    LOG="$LOG_DIR/${NAME}.log"

    start_time=$(date +%s)
    if timeout 30m bash UberDDR4/testbench/run_xsim.sh > "$LOG" 2>&1; then
        sim_ok=true
    else
        sim_ok=false
    fi
    end_time=$(date +%s)
    elapsed=$(( end_time - start_time ))
    TIMES+=("$elapsed")

    if $sim_ok && grep -q "PASS: All.*test phases\|PASS: All 10 write\|PASS: init_failed asserted as expected" "$LOG"; then
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

unset EXTRA_DEFINES MICRON_DENSITY MICRON_SPEED

# Summary
echo ""
echo -e "${BOLD}=== REGRESSION SUMMARY ===${RESET}"
echo ""
printf "  %-30s %-6s %s\n" "TEST" "RESULT" "TIME"
printf "  %-30s %-6s %s\n" "----" "------" "----"

index=0
for entry in "${TESTS[@]}"; do
    read -r NAME _ <<< "$entry"
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