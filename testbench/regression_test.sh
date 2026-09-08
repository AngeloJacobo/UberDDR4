#!/usr/bin/env bash
#
# regression_test.sh  -  UberDDR4 calibration regression suite
#
# Runs multiple simulation configurations to stress-test the PHY training
# FSM with realistic fly-by delays and address mapping variants.
#
# Usage:
#   export XILINX_VIVADO=/path/to/Vivado/2023.1
#   bash testbench/regression_test.sh          # run all tests
#   bash testbench/regression_test.sh 3 5 7    # run specific tests
#
# The 26-entry matrix is ALL_TESTS below. Recreates regression_logs and
# uses a per-checkout regression lock; direct simulator runs bypass that lock.
# Native/component wall limits and Windows cleanup scope: docs/VERIFICATION.md.
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
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WINDOWS_JOB="$SCRIPT_DIR/windows_xsim_job.ps1"
LOCK_DIR="$REPO_ROOT/.uberddr4_xsim.lock"
STOP_FILE="$REPO_ROOT/.uberddr4_xsim.stop"

SIM_PID=""
SIM_HAS_OWN_PG=false
SIM_IS_WINDOWS=false
LOCK_HELD=false
CLEANUP_RUNNING=0

# Keep terminal rendering in this shell.  There is no background status
# process that can survive Ctrl+C and continue writing over the prompt.
STATUS_TTY_FD=""
if { exec {STATUS_TTY_FD}>/dev/tty; } 2>/dev/null; then :; else STATUS_TTY_FD=""; fi

acquire_lock() {
    local owner=""
    if mkdir "$LOCK_DIR" 2>/dev/null; then
        printf '%s\n' "$$" > "$LOCK_DIR/pid"
        LOCK_HELD=true
        return 0
    fi
    [[ -r "$LOCK_DIR/pid" ]] && owner=$(<"$LOCK_DIR/pid")
    if [[ "$owner" =~ ^[0-9]+$ ]] && kill -0 "$owner" 2>/dev/null; then
        echo -e "${RED}ERROR: another XSim regression is already running (PID $owner).${RESET}" >&2
        return 1
    fi
    rm -f "$LOCK_DIR/pid" 2>/dev/null || true
    rm -f "$STOP_FILE" 2>/dev/null || true
    rmdir "$LOCK_DIR" 2>/dev/null || true
    mkdir "$LOCK_DIR" 2>/dev/null || return 1
    printf '%s\n' "$$" > "$LOCK_DIR/pid"
    LOCK_HELD=true
}

release_lock() {
    rm -f "$STOP_FILE" 2>/dev/null || true
    if $LOCK_HELD; then
        rm -f "$LOCK_DIR/pid" 2>/dev/null || true
        rmdir "$LOCK_DIR" 2>/dev/null || true
        LOCK_HELD=false
    fi
}

start_simulation() {
    local log_file="$1"
    SIM_HAS_OWN_PG=false
    SIM_IS_WINDOWS=false
    if command -v cygpath >/dev/null 2>&1 && command -v powershell.exe >/dev/null 2>&1; then
        rm -f "$STOP_FILE" 2>/dev/null || true
        MSYS2_ARG_CONV_EXCL='*' powershell.exe -NoProfile -NonInteractive \
            -ExecutionPolicy Bypass -File "$(cygpath -w "$WINDOWS_JOB")" \
            -RepositoryRoot "$(cygpath -w "$REPO_ROOT")" \
            -StopFile "$(cygpath -w "$STOP_FILE")" \
            -TimeoutSeconds "$((SIM_TIMEOUT_MINUTES * 60))" \
            > "$log_file" 2>&1 &
        SIM_IS_WINDOWS=true
    elif command -v setsid >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
        setsid timeout --signal=TERM --kill-after=10s \
            "${SIM_TIMEOUT_MINUTES}m" bash testbench/run_xsim.sh \
            > "$log_file" 2>&1 &
        SIM_HAS_OWN_PG=true
    elif command -v timeout >/dev/null 2>&1; then
        timeout --signal=TERM --kill-after=10s \
            "${SIM_TIMEOUT_MINUTES}m" bash testbench/run_xsim.sh \
            > "$log_file" 2>&1 &
    elif command -v setsid >/dev/null 2>&1; then
        setsid bash testbench/run_xsim.sh > "$log_file" 2>&1 &
        SIM_HAS_OWN_PG=true
    else
        bash testbench/run_xsim.sh > "$log_file" 2>&1 &
    fi
    SIM_PID=$!
}

stop_simulation() {
    local attempt
    [[ -n "${SIM_PID:-}" ]] || return 0
    if $SIM_IS_WINDOWS; then
        # Request an orderly stop so the owner can also terminate Xilinx tools
        # whose loader deliberately breaks them away from the Windows Job.
        : > "$STOP_FILE"
    elif $SIM_HAS_OWN_PG; then
        kill -TERM -- -"$SIM_PID" 2>/dev/null || true
    else
        # Portable fallback for hosts without setsid.
        kill -TERM "$SIM_PID" 2>/dev/null || true
    fi
    for attempt in {1..100}; do
        kill -0 "$SIM_PID" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "$SIM_PID" 2>/dev/null; then
        kill -KILL "$SIM_PID" 2>/dev/null || true
    fi
    wait "$SIM_PID" 2>/dev/null || true
    rm -f "$STOP_FILE" 2>/dev/null || true
    SIM_PID=""
}

show_progress() {
    local log_file="$1"
    local width start line elapsed frame spin=0 spinner='|/-\\'
    local tty=false
    [[ -n "$STATUS_TTY_FD" ]] && tty=true
    width=$(( ${COLUMNS:-120} - 24 ))
    (( width < 40 )) && width=40
    start=$(date +%s)
    while kill -0 "$SIM_PID" 2>/dev/null; do
        if $tty; then
            elapsed=$(( $(date +%s) - start ))
            frame="${spinner:$((spin++ % 4)):1}"
            line="Starting XSim..."
            [[ -s "$log_file" ]] && line=$(tail -n 1 "$log_file")
            line=${line//$'\r'/}
            (( ${#line} > width )) && line="...${line: -$((width - 3))}"
            printf '\r\033[K  [%02dm%02ds] %s %s' \
                $((elapsed / 60)) $((elapsed % 60)) "$frame" "$line" \
                >&"$STATUS_TTY_FD"
        fi
        sleep 0.5
    done
    $tty && printf '\r\033[K' >&"$STATUS_TTY_FD"
}

cleanup() {
    local exit_code="${1:-130}"
    (( CLEANUP_RUNNING )) && return
    CLEANUP_RUNNING=1
    trap '' INT TERM HUP
    echo -e "\n${RED}Interrupted — stopping simulation...${RESET}"
    stop_simulation
    release_lock
    exit "$exit_code"
}

trap 'cleanup 130' INT TERM HUP
trap 'release_lock' EXIT
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
    # tCK=1.600ns exercises the native PLL's x8/VCO low-frequency path.
    "ddr4_1250         1600 8   2  0    1  1  0  8  16  DDR4_8G_X8   FIXED_1600  -"
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
    # Data-mask stress test (per-WB-byte writes, 16x burst-write transactions for two lanes)
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

# A regression is a functional check, not a waveform-debug session.  Keep
# dumping opt-in so an inherited DUMP_VCD=1 cannot make every test crawl.
export DUMP_VCD="${REGRESSION_DUMP_VCD:-0}"

# The detailed native BITSLICE model performs the same complete calibration
# sweep as hardware and is substantially slower than component-mode models.
# Give it a safe default timeout while leaving an explicit user override.
if [[ "${PHY_IMPL:-component}" == "native" ]]; then
    SIM_TIMEOUT_MINUTES="${SIM_TIMEOUT_MINUTES:-240}"
else
    SIM_TIMEOUT_MINUTES="${SIM_TIMEOUT_MINUTES:-60}"
fi
if ! [[ "$SIM_TIMEOUT_MINUTES" =~ ^[1-9][0-9]*$ ]]; then
    echo -e "${RED}ERROR: SIM_TIMEOUT_MINUTES must be a positive integer.${RESET}"
    exit 1
fi

if ! acquire_lock; then
    exit 1
fi
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
echo -e "${DIM}Timeout per test: ${SIM_TIMEOUT_MINUTES}m${RESET}"
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
    # Preserve every production calibration operation, but avoid printing a
    # line for every gate/WL tap. Final lane results and all failures remain.
    if [[ "${PHY_IMPL:-component}" == "native" ]]; then
        DEFINES="-d SIM_QUIET_TRAINING_LOG"
    fi
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
    # regression_config.vh is a compilation source and changes per case.
    # Rebuild from a clean XSim library so every test receives its exact
    # DDR4/device/fly-by configuration rather than a stale prior one.
    rm -rf xsim.dir 2>/dev/null

    LOG="$LOG_DIR/${NAME}.log"

    start_time=$(date +%s)
    start_simulation "$LOG"
    show_progress "$LOG"
    wait "$SIM_PID"
    sim_exit=$?
    SIM_PID=""
    if [[ $sim_exit -eq 124 ]] && ! grep -q "TIMEOUT:" "$LOG"; then
        printf 'TIMEOUT: simulation exceeded %s minutes\n' \
            "$SIM_TIMEOUT_MINUTES" >> "$LOG"
    fi
    if [[ $sim_exit -eq 0 ]]; then
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
