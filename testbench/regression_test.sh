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

SIM_HAS_OWN_PG=false
SIM_PID=""
# Ctrl+C is broadcast to every foreground Git-Bash pipeline member.  Make the
# handler idempotent so only one shell owns termination of the XSim tree.
CLEANUP_RUNNING=0

# setsid is available on typical Linux hosts but absent from Git Bash.  It is
# used only for Ctrl+C process-group cleanup, not for simulation correctness.
start_simulation() {
    local log_file="$1"
    shift
    SIM_HAS_OWN_PG=false
    if command -v setsid >/dev/null 2>&1; then
        setsid "$@" > "$log_file" 2>&1 &
        SIM_HAS_OWN_PG=true
    else
        "$@" > "$log_file" 2>&1 &
    fi
    SIM_PID=$!
}

# setsid lets Linux terminate an entire process group.  Git Bash has no
# setsid, so use Windows taskkill /T to terminate Bash, pipelines, and XSim.
terminate_simulation_tree() {
    local pid="$1"
    local own_pg="$2"
    if $own_pg; then
        kill -- -"$pid" 2>/dev/null
    elif command -v taskkill.exe >/dev/null 2>&1; then
        taskkill.exe /PID "$pid" /T /F >/dev/null 2>&1
    else
        local child
        for child in $(pgrep -P "$pid" 2>/dev/null); do
            terminate_simulation_tree "$child" false
        done
        kill "$pid" 2>/dev/null
    fi
}

cleanup() {
    local exit_code="${1:-130}"
    if (( CLEANUP_RUNNING )); then
        trap - INT TERM HUP
        exit "$exit_code"
    fi
    CLEANUP_RUNNING=1
    trap - INT TERM HUP
    echo -e "\n${RED}Interrupted — killing child processes...${RESET}"
    if [[ -n "${SIM_PID:-}" ]]; then
        terminate_simulation_tree "$SIM_PID" "$SIM_HAS_OWN_PG"
        # Never use an unqualified `wait` here: it can wait on unrelated
        # pipeline members and is susceptible to another Ctrl+C delivery.
        wait "$SIM_PID" 2>/dev/null || true
        SIM_PID=""
    fi
    exit "$exit_code"
}
trap 'cleanup 130' INT TERM HUP

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# The regression changes to REPO_ROOT before each test, so keeping this
# relative avoids Windows/Git-Bash issues with absolute paths containing
# spaces while remaining portable on Linux.
LOG_DIR="testbench/regression_logs"
CONFIG_FILE="$LOG_DIR/regression_config.vh"

# Each line: NAME  DDR4_CLK  DW  LANES  FLYBY  MAP  BIST  DM  DENS  ROWS  MICRON_DEF     MICRON_SPEED  SPECIAL
#   DW = DEVICE_WIDTH (4, 8, or 16)
#   LANES = BYTE_LANES (number of 8-bit byte lanes)
#   DM = BIST_DM_TEST (0=full-word burst writes, 1=per-byte-lane DM stress)
#   ROWS = ROW_BITS (14-17, default 16)
# To add a new test configuration, just add a new line to this array.
ALL_TESTS=(
    # Core x8 DDR4-2400 sweep
    "baseline          834  8   2  0    1  1  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    "flyby_50          834  8   2  50   1  1  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    "flyby_100         834  8   2  100  1  1  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    "flyby_200         834  8   2  200  1  1  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    "flyby_300         834  8   2  300  1  1  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    "flyby_400         834  8   2  400  1  1  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    "map0              834  8   2  0    0  1  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    "map0_flyby_200    834  8   2  200  0  1  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    "bist_full         834  8   2  0    1  2  0  8  16  DDR4_8G_X8   FIXED_2400  -"
    # x16: 1 chip = 2 byte lanes (BG_BITS=1, DM enabled)
    "x16               834  16  2  0    1  1  0  8  16  DDR4_8G_X16  FIXED_2400  -"
    "x16_map0          834  16  2  0    0  1  0  8  16  DDR4_8G_X16  FIXED_2400  -"
    "x16_bist_full     834  16  2  0    1  2  0  8  16  DDR4_8G_X16  FIXED_2400  -"
    "x16_flyby_4lane   834  16  4  200  1  1  0  8  16  DDR4_8G_X16  FIXED_2400  -"
    # x4: 2 chips paired per byte lane (BG_BITS=2, no DM)
    "x4                834  4   2  0    1  1  0  8  16  DDR4_8G_X4   FIXED_2400  -"
    "x4_map0           834  4   2  0    0  1  0  8  16  DDR4_8G_X4   FIXED_2400  -"
    # Speed grade sweep
    "ddr4_1600         1250 8   2  0    1  1  0  8  16  DDR4_8G_X8   FIXED_1600  -"
    "ddr4_1600_flyby   1250 8   2  200  1  1  0  8  16  DDR4_8G_X8   FIXED_1600  -"
    "ddr4_2133         937  8   2  0    1  1  0  8  16  DDR4_8G_X8   FIXED_2133  -"
    "ddr4_2133_flyby   937  8   2  200  1  1  0  8  16  DDR4_8G_X8   FIXED_2133  -"
    # Density sweep
    "density_4g        834  8   2  0    1  1  0  4  15  DDR4_4G_X8   FIXED_2400  -"
    # ROW_BITS sweep (min/max supported width)
    "row_bits_14       834  8   2  0    1  1  0  8  14  DDR4_8G_X8   FIXED_2400  -"
    "row_bits_17       834  8   2  0    1  1  0  8  17  DDR4_8G_X8   FIXED_2400  -"
    # Error path
    "train_fail        834  8   2  0    1  1  0  8  16  DDR4_8G_X8   FIXED_2400  TRAIN_FAIL"
    # CSR reset test
    "csr_reset         834  8   2  0    1  1  0  8  16  DDR4_8G_X8   FIXED_2400  CSR_RESET"
    # Data-mask stress test (per-byte-lane writes, ~16x longer burst phase)
    "dm_stress         834  8   2  0    1  1  1  8  16  DDR4_8G_X8   FIXED_2400  -"
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
rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"

write_regression_config() {
    {
        printf '`define SIM_DDR4_CLK_PERIOD %s\n' "$DDR4_CLK"
        printf '`define SIM_DEVICE_WIDTH %s\n' "$DW"
        printf '`define SIM_BYTE_LANES %s\n' "$LANES"
        printf '`define SIM_FLY_BY_DELAY %s\n' "$FLYBY"
        printf '`define SIM_ADDR_MAPPING %s\n' "$MAP"
        printf '`define SIM_BIST_MODE %s\n' "$BIST"
        printf '`define SIM_BIST_DM_TEST %s\n' "$DM"
        printf '`define SIM_ROW_BITS %s\n' "$ROWS"
        [[ "$DENS" == "4" ]] && printf '`define SIM_DENSITY_4G\n'
        [[ "$SPECIAL" == "TRAIN_FAIL" ]] && printf '`define SIM_FORCE_TRAIN_FAIL\n'
        [[ "$SPECIAL" == "CSR_RESET" ]] && printf '`define SIM_CSR_RESET_TEST\n'
    } > "$CONFIG_FILE"
}

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
    read -r NAME DDR4_CLK DW LANES FLYBY MAP BIST DM DENS ROWS MICRON_DEF MICRON_SPD SPECIAL <<< "$entry"

    # Vivado 2022.2 on Windows misparses -d NAME=<numeric-value> and
    # treats the value as a source filename.  The baseline tCK is already
    # the testbench default, so do not pass a redundant numeric define.
    DEFINES=""
    [[ "$DDR4_CLK" != "834" ]] && DEFINES="-d SIM_DDR4_CLK_PERIOD=$DDR4_CLK"
    [[ "$DW" != "8" ]]      && DEFINES="$DEFINES -d SIM_DEVICE_WIDTH=$DW"
    [[ "$LANES" != "2" ]]   && DEFINES="$DEFINES -d SIM_BYTE_LANES=$LANES"
    [[ "$FLYBY" != "0" ]]   && DEFINES="$DEFINES -d SIM_FLY_BY_DELAY=$FLYBY"
    [[ "$MAP" != "1" ]]     && DEFINES="$DEFINES -d SIM_ADDR_MAPPING=$MAP"
    [[ "$BIST" != "1" ]]    && DEFINES="$DEFINES -d SIM_BIST_MODE=$BIST"
    [[ "$DM" != "0" ]]      && DEFINES="$DEFINES -d SIM_BIST_DM_TEST=$DM"
    [[ "$DENS" == "4" ]]    && DEFINES="$DEFINES -d SIM_DENSITY_4G"
    [[ "$ROWS" != "16" ]]   && DEFINES="$DEFINES -d SIM_ROW_BITS=$ROWS"
    [[ "$SPECIAL" == "TRAIN_FAIL" ]] && DEFINES="$DEFINES -d SIM_FORCE_TRAIN_FAIL"
    [[ "$SPECIAL" == "CSR_RESET" ]] && DEFINES="$DEFINES -d SIM_CSR_RESET_TEST"

    write_regression_config
    DEFINES=""
    export EXTRA_DEFINES="$DEFINES"
    export SIM_CONFIG_FILE="$CONFIG_FILE"
    export MICRON_DENSITY="$MICRON_DEF"
    export MICRON_SPEED="$MICRON_SPD"

    ((index++))

    echo -e "${BOLD}[$index/$total] ${CYAN}$NAME${RESET}"
    if [[ -n "$DEFINES" ]]; then
        echo -e "  ${DIM}defines: $DEFINES${RESET}"
    fi

    cd "$REPO_ROOT"
    rm -rf xsim.dir 2>/dev/null; rm -rf xsim.dir 2>/dev/null

    LOG="$LOG_DIR/${NAME}.log"

    start_time=$(date +%s)
    # timeout is also a GNU/Linux utility and is not bundled with Git Bash.
    # On Windows, let XSim run normally; Ctrl+C still terminates the child.
    if command -v timeout >/dev/null 2>&1; then
        start_simulation "$LOG" bash -c 'set -o pipefail; timeout 60m bash testbench/run_xsim.sh 2>&1 | sed "s/\x1b\[[0-9;]*m//g"'
    else
        start_simulation "$LOG" bash -c 'set -o pipefail; bash testbench/run_xsim.sh 2>&1 | sed "s/\x1b\[[0-9;]*m//g"'
    fi
    wait $SIM_PID
    if [[ $? -eq 0 ]]; then
        sim_ok=true
    else
        sim_ok=false
    fi
    end_time=$(date +%s)
    elapsed=$(( end_time - start_time ))
    TIMES+=("$elapsed")

    violation_count=$(grep -c "VIOLATION" "$LOG" 2>/dev/null)
    violation_count=${violation_count:-0}

    if $sim_ok && grep -q "PASS: All test phases + BIST\|PASS: init_failed asserted as expected\|PASS: CSR reset test" "$LOG" \
              && ! grep -q "Simulation FAILED\|FAIL: rd_err\|FAIL: bist" "$LOG" \
              && [[ "$violation_count" -eq 0 ]]; then
        RESULTS+=("PASS")
        ((pass_count++))
        echo -e "  ${GREEN}PASS${RESET} (${elapsed}s)"
    else
        RESULTS+=("FAIL")
        ((fail_count++))
        echo -e "  ${RED}FAIL${RESET} (${elapsed}s)"
        if [[ "$violation_count" -gt 0 ]]; then
            echo -e "  ${RED}  Reason: ${violation_count} Micron model VIOLATION(s)${RESET}"
        elif grep -q "TIMEOUT:" "$LOG"; then
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
