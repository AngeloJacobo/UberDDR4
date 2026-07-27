#!/usr/bin/env bash
#
# run_compile.sh — UberDDR4 Build & Verification Suite
#
# Stages:
#   lint      Verilator lint (controller, prober, phy, top)
#   compile   Iverilog parse + Yosys synthesis check
#   formal    SymbiYosys bounded model checking
#   sim       Vivado xsim simulation (single test or regression)
#
# Usage:
#   ./run_compile.sh                     Default (lint+compile+formal+sim baseline)
#   ./run_compile.sh --all               Everything (lint+compile+formal+formal-regr+sim+sim-regr)
#   ./run_compile.sh --lint              Verilator lint only
#   ./run_compile.sh --compile           Iverilog + Yosys only
#   ./run_compile.sh --formal            Formal single config (4 tasks)
#   ./run_compile.sh --formal-regr       Formal regression (28 tasks)
#   ./run_compile.sh --sim [TEST]        Single sim test (default: baseline)
#   ./run_compile.sh --sim-regr          Full sim regression (22 tests)
#   ./run_compile.sh --no-sim            Lint + compile + formal (skip sim)
#
# Sim tests: baseline flyby_50 flyby_100 flyby_200 flyby_300 flyby_400
#   map0 map0_flyby_200 bist_full x16 x16_map0 x16_bist_full x16_flyby_4lane
#   x4 x4_map0 ddr4_1600 ddr4_1600_flyby ddr4_2133 ddr4_2133_flyby
#   density_4g train_fail
#
# Engineer: Angelo C. Jacobo
set -o pipefail

CHILD_PID=""
CHILD_HAS_OWN_PG=false
# Ctrl+C is delivered to this script and, on Git Bash, to each member of the
# foreground pipeline.  Do not let a second delivery re-enter cleanup while
# the first handler is terminating the process tree.
CLEANUP_RUNNING=0

# Linux commonly provides setsid; Git Bash normally does not.  The test
# commands do not require a separate session, so use one only when available.
run_isolated() {
    if command -v setsid >/dev/null 2>&1; then
        setsid "$@"
    else
        "$@"
    fi
}

start_background_log() {
    local log_file="$1"
    shift
    CHILD_HAS_OWN_PG=false
    if command -v setsid >/dev/null 2>&1; then
        setsid "$@" > "$log_file" 2>&1 &
        CHILD_HAS_OWN_PG=true
    else
        "$@" > "$log_file" 2>&1 &
    fi
    CHILD_PID=$!
}

# Terminate every descendant of a background test.  On Linux, setsid gives
# the child a dedicated process group.  Git Bash lacks setsid, so taskkill's
# /T option is required to terminate the Bash/pipeline/XSim process tree.
terminate_child_tree() {
    local pid="$1"
    local own_pg="$2"
    if $own_pg; then
        kill -- -"$pid" 2>/dev/null
    elif command -v taskkill.exe >/dev/null 2>&1; then
        taskkill.exe /PID "$pid" /T /F >/dev/null 2>&1
    else
        # Portable non-Windows fallback for hosts without setsid.
        local child
        for child in $(pgrep -P "$pid" 2>/dev/null); do
            terminate_child_tree "$child" false
        done
        kill "$pid" 2>/dev/null
    fi
}

cleanup() {
    local exit_code="${1:-130}"
    if (( CLEANUP_RUNNING )); then
        # The original handler already owns child termination.  Removing the
        # trap before exiting prevents the repeated "Interrupted" loop seen
        # when Git Bash broadcasts Ctrl+C to nested shell pipelines.
        trap - INT TERM HUP
        exit "$exit_code"
    fi
    CLEANUP_RUNNING=1
    trap - INT TERM HUP
    printf "\n\033[31mInterrupted — killing child processes...\033[0m\n"
    if [[ -n "${CHILD_PID:-}" ]]; then
        terminate_child_tree "$CHILD_PID" "$CHILD_HAS_OWN_PG"
        # Wait only for the process we started.  A bare `wait` can itself be
        # interrupted by the terminal signal and recursively invoke cleanup.
        wait "$CHILD_PID" 2>/dev/null || true
        CHILD_PID=""
    fi
    exit "$exit_code"
}
trap 'cleanup 130' INT TERM HUP

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ═══════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════
VIVADO="${XILINX_VIVADO:-/cad/adi/apps/xilinx/vivado/2023.1/Vivado/2023.1}"
UNISIMS="$VIVADO/data/verilog/src/unisims"

RTL_CORE=(rtl/ddr4_controller.v rtl/ddr4_phy.v rtl/ddr4_prober.v rtl/ddr4_top.v)
RTL_AXI=(rtl/axi/ddr4_top_axi.v rtl/axi/axim2wbsp.v rtl/axi/aximrd2wbsp.v
         rtl/axi/aximwr2wbsp.v rtl/axi/axi_addr.v rtl/axi/skidbuffer.v
         rtl/axi/sfifo.v rtl/axi/wbarbiter.v)
LOGDIR="build_logs"
rm -rf "$LOGDIR"

SIM_TESTS=(
    baseline flyby_50 flyby_100 flyby_200 flyby_300 flyby_400
    map0 map0_flyby_200 bist_full
    x16 x16_map0 x16_bist_full x16_flyby_4lane
    x4 x4_map0
    ddr4_1600 ddr4_1600_flyby ddr4_2133 ddr4_2133_flyby
    density_4g row_bits_14 row_bits_17 train_fail csr_reset dm_stress
)

# ═══════════════════════════════════════════════════════════════════════════
# Colors & symbols (disabled when not a terminal)
# ═══════════════════════════════════════════════════════════════════════════
if [[ -t 1 ]]; then
    RST='\033[0m'  BLD='\033[1m'  DIM='\033[2m'
    RED='\033[1;31m' GRN='\033[1;32m' YLW='\033[1;33m'
    BLU='\033[1;34m' CYN='\033[1;36m' WHT='\033[1;37m'
else
    RST='' BLD='' DIM='' RED='' GRN='' YLW='' BLU='' CYN='' WHT=''
fi
OK="✓"  XF="✗"  SK="○"

# ═══════════════════════════════════════════════════════════════════════════
# State
# ═══════════════════════════════════════════════════════════════════════════
PASS_N=0  FAIL_N=0  SKIP_N=0
SP=0 SF=0 SS=0
declare -a SUMMARY=()
T0=$(date +%s)

# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════
elapsed() {
    local s=$1
    if   (( s >= 3600 )); then printf "%dh%02dm%02ds" $((s/3600)) $((s%3600/60)) $((s%60))
    elif (( s >= 60   )); then printf "%dm%02ds" $((s/60)) $((s%60))
    else                       printf "%ds" "$s"
    fi
}

pass() { printf "  ${GRN}${OK}${RST} %-52s ${GRN}PASS${RST}  ${DIM}%s${RST}\n" "$1" "$2"; ((PASS_N++)); ((SP++)); }
fail() { printf "  ${RED}${XF}${RST} %-52s ${RED}FAIL${RST}  ${DIM}%s${RST}\n" "$1" "$2"; ((FAIL_N++)); ((SF++)); }
skip() { printf "  ${YLW}${SK}${RST} %-52s ${YLW}SKIP${RST}\n" "$1";                      ((SKIP_N++)); ((SS++)); }

show_errors() {
    local f="$1" n="${2:-6}"
    [[ -s "$f" ]] || return 0
    grep -iE "error|fail|warn" "$f" | head -n "$n" | while IFS= read -r l; do
        printf "    ${DIM}%s${RST}\n" "$l"
    done
}

stage_reset() { SP=0; SF=0; SS=0; }

stage_record() {
    local name="$1"
    local st
    if   (( SF > 0 )); then st="${RED}FAIL${RST}"
    elif (( SP > 0 )); then st="${GRN}PASS${RST}"
    else                     st="${YLW}SKIP${RST}"
    fi
    SUMMARY+=("$(printf "  %-14s %2d pass  %2d fail  %2d skip       %b" \
                        "$name" "$SP" "$SF" "$SS" "$st")")
}

banner() {
    local w=66
    local commit branch ts
    commit=$(git rev-parse --short HEAD 2>/dev/null || echo "n/a")
    branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "n/a")
    ts=$(date "+%Y-%m-%d %H:%M")
    echo
    printf "${CYN}╔"; printf '═%.0s' $(seq 1 $w); printf "╗${RST}\n"
    printf "${CYN}║${BLD}${WHT}  %-$((w-2))s  ${CYN}║${RST}\n" "UberDDR4 Build & Verification Suite"
    printf "${CYN}║${DIM}  %-$((w-2))s  ${CYN}║${RST}\n" "$ts  ·  $branch @ $commit"
    printf "${CYN}╚"; printf '═%.0s' $(seq 1 $w); printf "╝${RST}\n"
    echo
}

header() {
    printf "\n${BLU}┌──${BLD} Stage %s/%s: %s${RST}\n" "$1" "$2" "$3"
    printf "${BLU}│${RST}\n"
}

# ═══════════════════════════════════════════════════════════════════════════
# Parse arguments
# ═══════════════════════════════════════════════════════════════════════════
DO_LINT=false  DO_COMPILE=false  DO_FORMAL=false  DO_SIM=false
FORMAL_REGR=false  SIM_REGR=false  SIM_TEST="baseline"
EXPLICIT=false

show_help() {
    sed -n '3,/^set /{ /^set /d; s/^# \?//p; }' "$0"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --lint)        DO_LINT=true;    EXPLICIT=true ;;
        --compile)     DO_COMPILE=true; EXPLICIT=true ;;
        --formal)      DO_FORMAL=true;  EXPLICIT=true ;;
        --all)         DO_LINT=true; DO_COMPILE=true; DO_FORMAL=true; FORMAL_REGR=true
                       DO_SIM=true; SIM_REGR=true; EXPLICIT=true ;;
        --formal-regr) DO_FORMAL=true;  FORMAL_REGR=true; EXPLICIT=true ;;
        --sim)         DO_SIM=true;     EXPLICIT=true
                       if [[ -n "${2:-}" && "${2:0:2}" != "--" ]]; then
                           SIM_TEST="$2"; shift
                       fi ;;
        --sim-regr)    DO_SIM=true;     SIM_REGR=true; EXPLICIT=true ;;
        --no-sim)      DO_LINT=true; DO_COMPILE=true; DO_FORMAL=true; EXPLICIT=true ;;
        --help|-h)     show_help ;;
        *)             printf "${RED}Unknown: %s${RST}\n" "$1"; show_help ;;
    esac
    shift
done

if ! $EXPLICIT; then
    DO_LINT=true; DO_COMPILE=true; DO_FORMAL=true; DO_SIM=true
fi

TOTAL=0
$DO_LINT    && ((TOTAL++))
$DO_COMPILE && ((TOTAL++))
$DO_FORMAL  && ((TOTAL++))
$DO_SIM     && ((TOTAL++))
STAGE=0

# ═══════════════════════════════════════════════════════════════════════════
# Tool check
# ═══════════════════════════════════════════════════════════════════════════
check_tools() {
    local ok=true
    local -a required_tools=()

    $DO_LINT    && required_tools+=(verilator)
    $DO_COMPILE && required_tools+=(iverilog yosys)
    $DO_FORMAL  && required_tools+=(sby)

    printf "${DIM}  Checking tools...${RST}\n"
    for tool in "${required_tools[@]}"; do
        if command -v "$tool" &>/dev/null; then
            printf "  ${GRN}${OK}${RST} %-14s ${DIM}%s${RST}\n" "$tool" "$(command -v "$tool")"
        else
            printf "  ${RED}${XF}${RST} %-14s ${RED}not found${RST}\n" "$tool"
            ok=false
        fi
    done
    if [[ -d "$VIVADO" ]]; then
        printf "  ${GRN}${OK}${RST} %-14s ${DIM}%s${RST}\n" "vivado" "$VIVADO"
    else
        printf "  ${RED}${XF}${RST} %-14s ${RED}XILINX_VIVADO not set${RST}\n" "vivado"
        if $DO_SIM; then ok=false; fi
    fi
    echo
    local plan=""
    $DO_LINT    && plan+="lint → "
    $DO_COMPILE && plan+="compile → "
    $DO_FORMAL  && { $FORMAL_REGR && plan+="formal-regr → " || plan+="formal → "; }
    $DO_SIM     && { $SIM_REGR && plan+="sim-regr (${#SIM_TESTS[@]} tests)" || plan+="sim ($SIM_TEST)"; }
    plan="${plan% → }"
    printf "  ${BLD}Stages:${RST} %s\n\n" "$plan"
    $ok || { printf "${RED}  Missing required tools. Aborting.${RST}\n"; exit 1; }
}

# ═══════════════════════════════════════════════════════════════════════════
# Stage 1: Verilator Lint
# ═══════════════════════════════════════════════════════════════════════════
generate_stubs() {
    mkdir -p "$LOGDIR"
    local stubs="$LOGDIR/.xilinx_stubs.v"
    [[ -f "$stubs" ]] && return
    cat > "$stubs" << 'STUBS'
/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNDRIVEN */
/* verilator lint_off UNUSEDPARAM */
`timescale 1ps / 1ps
module OSERDESE3 #(parameter DATA_WIDTH=8, INIT=0, IS_CLKDIV_INVERTED=0,
    IS_CLK_INVERTED=0, IS_RST_INVERTED=0, ODDR_MODE="FALSE",
    OSERDES_D_BYPASS="FALSE", OSERDES_T_BYPASS="FALSE", SIM_DEVICE="")
    (input CLK, CLKDIV, RST, T,
     input [DATA_WIDTH-1:0] D,
     output OQ, T_OUT);
endmodule
module ISERDESE3 #(parameter DATA_WIDTH=8, FIFO_ENABLE="FALSE",
    FIFO_SYNC_MODE="FALSE", IS_CLK_B_INVERTED=1, IS_CLK_INVERTED=0,
    IS_RST_INVERTED=0, SIM_DEVICE="")
    (input CLK, CLK_B, CLKDIV, D, RST, FIFO_RD_CLK, FIFO_RD_EN,
     output [DATA_WIDTH-1:0] Q, output FIFO_EMPTY, INTERNAL_DIVCLK);
endmodule
module ODELAYE3 #(parameter CASCADE="NONE", DELAY_FORMAT="TIME",
    DELAY_TYPE="FIXED", DELAY_VALUE=0, IS_CLK_INVERTED=0,
    IS_RST_INVERTED=0, REFCLK_FREQUENCY=300.0, SIM_DEVICE="",
    SIM_VERSION=1.0, UPDATE_MODE="ASYNC")
    (input CLK, EN_VTC, INC, CE, LOAD, RST, ODATAIN, CASC_IN, CASC_RETURN,
     input [8:0] CNTVALUEIN,
     output DATAOUT, CASC_OUT,
     output [8:0] CNTVALUEOUT);
endmodule
module IDELAYE3 #(parameter CASCADE="NONE", DELAY_FORMAT="TIME",
    DELAY_TYPE="FIXED", DELAY_VALUE=0, DELAY_SRC="IDATAIN",
    IS_CLK_INVERTED=0, IS_RST_INVERTED=0, REFCLK_FREQUENCY=300.0,
    SIM_DEVICE="", SIM_VERSION=1.0, UPDATE_MODE="ASYNC")
    (input CLK, EN_VTC, INC, CE, LOAD, RST, IDATAIN, DATAIN,
     CASC_IN, CASC_RETURN,
     input [8:0] CNTVALUEIN,
     output DATAOUT, CASC_OUT,
     output [8:0] CNTVALUEOUT);
endmodule
module OBUFDS (input I, output O, OB);
endmodule
module OBUF (input I, output O);
endmodule
module IOBUF (input I, T, output O, inout IO);
endmodule
module IOBUFDS #(parameter DQS_BIAS="FALSE")
    (input I, T, output O, inout IO, IOB);
endmodule
module IDELAYCTRL #(parameter SIM_DEVICE="")
    (input REFCLK, RST, output RDY);
endmodule
STUBS
}

run_lint() {
    ((STAGE++))
    stage_reset
    header "$STAGE" "$TOTAL" "Verilator Lint"

    generate_stubs
    local stubs="$LOGDIR/.xilinx_stubs.v"

    for f in "${RTL_CORE[@]}"; do
        local mod log t0 t1
        mod=$(basename "$f" .v)
        log="$LOGDIR/lint_${mod}.log"
        t0=$(date +%s)
        local lint_files="$f"
        if [[ "$mod" == "ddr4_top" ]]; then
            lint_files="${RTL_CORE[*]}"
        fi
        if verilator --lint-only -Wall \
               --top-module "$mod" "$stubs" $lint_files > "$log" 2>&1; then
            t1=$(date +%s)
            local wc
            wc=$(grep -c "Warning" "$log" 2>/dev/null || true)
            if (( wc > 0 )); then
                pass "$f (${wc} warnings)" "$(elapsed $((t1-t0)))"
            else
                pass "$f" "$(elapsed $((t1-t0)))"
            fi
        else
            t1=$(date +%s)
            fail "$f" "$(elapsed $((t1-t0)))"
            show_errors "$log"
        fi
    done
    stage_record "Lint"
}

# ═══════════════════════════════════════════════════════════════════════════
# Stage 2: Compile Checks
# ═══════════════════════════════════════════════════════════════════════════
run_compile() {
    ((STAGE++))
    stage_reset
    header "$STAGE" "$TOTAL" "Compile Checks"

    generate_stubs
    local stubs="$LOGDIR/.xilinx_stubs.v"
    local log t0 t1

    # ── Iverilog ──
    log="$LOGDIR/compile_iverilog.log"
    t0=$(date +%s)
    if iverilog -g2012 -Wall -t null \
           "$stubs" \
           "${RTL_CORE[@]}" > "$log" 2>&1; then
        t1=$(date +%s)
        pass "iverilog (core RTL)" "$(elapsed $((t1-t0)))"
    else
        t1=$(date +%s)
        fail "iverilog (core RTL)" "$(elapsed $((t1-t0)))"
        show_errors "$log"
    fi

    # ── Yosys ──
    log="$LOGDIR/compile_yosys.log"
    t0=$(date +%s)
    local yosys_script="read_verilog -sv $stubs"
    for f in "${RTL_CORE[@]}"; do yosys_script+=" $f"; done
    yosys_script+="; hierarchy -top ddr4_top -check -purge_lib; proc; opt; check"
    if yosys -q -p "$yosys_script" > "$log" 2>&1; then
        t1=$(date +%s)
        pass "yosys (synthesis check)" "$(elapsed $((t1-t0)))"
    else
        t1=$(date +%s)
        fail "yosys (synthesis check)" "$(elapsed $((t1-t0)))"
        show_errors "$log"
    fi

    stage_record "Compile"
}

# ═══════════════════════════════════════════════════════════════════════════
# Stage 3: Formal Verification
# ═══════════════════════════════════════════════════════════════════════════
run_formal() {
    ((STAGE++))
    stage_reset
    local sby_file label
    if $FORMAL_REGR; then
        sby_file="formal/ddr4_multiconfig.sby"
        label="Formal Verification (regression — 28 tasks)"
    else
        sby_file="formal/ddr4_singleconfig.sby"
        label="Formal Verification (single — 4 tasks)"
    fi
    header "$STAGE" "$TOTAL" "$label"

    rm -rf formal/ddr4_*/
    mkdir -p "$LOGDIR"
    local log t0 t1
    log="$LOGDIR/formal.log"
    t0=$(date +%s)
    start_background_log "$log" sby -f "$sby_file"
    wait "$CHILD_PID"
    local rc=$?
    CHILD_PID=""
    t1=$(date +%s)

    local base
    base=$(basename "$sby_file" .sby)
    for d in ${base}_*/; do
        [[ -d "$d" ]] || continue
        local task
        task=$(basename "$d")
        task=${task#${base}_}
        if [[ -e "${d}PASS" ]]; then
            pass "$task" ""
        else
            fail "$task" ""
        fi
    done

    if (( SF == 0 && SP == 0 )); then
        if (( rc == 0 )); then
            pass "sby $base" "$(elapsed $((t1-t0)))"
        else
            fail "sby $base" "$(elapsed $((t1-t0)))"
            show_errors "$log"
        fi
    else
        printf "  ${DIM}  Total: %s${RST}\n" "$(elapsed $((t1-t0)))"
    fi

    stage_record "Formal"
}

# ═══════════════════════════════════════════════════════════════════════════
# Stage 4: Simulation
# ═══════════════════════════════════════════════════════════════════════════
run_sim() {
    ((STAGE++))
    stage_reset

    if [[ -z "${XILINX_VIVADO:-}" && ! -d "$VIVADO" ]]; then
        header "$STAGE" "$TOTAL" "Simulation"
        fail "XILINX_VIVADO not set" ""
        stage_record "Sim"
        return
    fi
    export XILINX_VIVADO="$VIVADO"

    if $SIM_REGR; then
        header "$STAGE" "$TOTAL" "Simulation Regression (${#SIM_TESTS[@]} tests)"
        mkdir -p "$LOGDIR"
        local log="$LOGDIR/sim_regression.log"
        local st0
        st0=$(date +%s)

        local strip_ansi='s/\x1b\[[0-9;]*m//g'
        local name_re='^\[([0-9]+)/([0-9]+)\] +([^ ]+)'
        local result_re='(PASS|FAIL) +\(([0-9]+)s\)'
        local cur_name=""

        while IFS= read -r line; do
            echo "$line" >> "$log"
            local clean
            clean=$(printf '%s' "$line" | sed "$strip_ansi")
            if [[ $clean =~ $name_re ]]; then
                cur_name="${BASH_REMATCH[3]}"
            fi
            if [[ -n "$cur_name" && $clean =~ $result_re ]]; then
                local result="${BASH_REMATCH[1]}"
                local secs="${BASH_REMATCH[2]}"
                if [[ "$result" == "PASS" ]]; then
                    pass "$cur_name" "$(elapsed "$secs")"
                else
                    fail "$cur_name" "$(elapsed "$secs")"
                fi
                cur_name=""
            fi
        done < <(cd "$SCRIPT_DIR" && run_isolated bash "$SCRIPT_DIR/testbench/regression_test.sh" 2>&1)

        local st1
        st1=$(date +%s)
        printf "  ${DIM}  Total sim time: %s${RST}\n" "$(elapsed $((st1-st0)))"
        printf "  ${DIM}  Logs: testbench/regression_logs/${RST}\n"
    else
        header "$STAGE" "$TOTAL" "Simulation ($SIM_TEST)"

        local idx=-1
        for i in "${!SIM_TESTS[@]}"; do
            if [[ "${SIM_TESTS[$i]}" == "$SIM_TEST" ]]; then
                idx=$((i+1)); break
            fi
        done
        if (( idx < 0 )); then
            fail "$SIM_TEST (unknown test — see --help)" ""
            stage_record "Sim"
            return
        fi

        mkdir -p "$LOGDIR"
        # The simulation launcher runs from the parent directory so the
        # historical UberDDR4/... paths resolve.  Keep its log in this
        # script's build_logs directory with an absolute path.
        local log="$SCRIPT_DIR/$LOGDIR/sim_${SIM_TEST}.log"
        local st0 st1
        st0=$(date +%s)
        cd "$SCRIPT_DIR"
        start_background_log "$log" bash "$SCRIPT_DIR/testbench/regression_test.sh" "$idx"
        wait "$CHILD_PID"
        local rc=$?
        CHILD_PID=""
        st1=$(date +%s)
        cd "$SCRIPT_DIR"

        if (( rc == 0 )); then
            pass "$SIM_TEST" "$(elapsed $((st1-st0)))"
        else
            fail "$SIM_TEST" "$(elapsed $((st1-st0)))"
            grep -iE "FAIL|error|mismatch" "$log" | grep -v "^#" | tail -5 | \
                while IFS= read -r l; do printf "    ${DIM}%s${RST}\n" "$l"; done
        fi
    fi

    stage_record "Sim"
}

# ═══════════════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════════════
print_summary() {
    local t1 total_time total overall w=66
    t1=$(date +%s)
    total_time=$((t1 - T0))
    total=$((PASS_N + FAIL_N + SKIP_N))

    echo
    printf "${WHT}"; printf '═%.0s' $(seq 1 $w); printf "${RST}\n"
    printf "${BLD}${WHT}  RESULTS SUMMARY${RST}\n"
    printf "${WHT}"; printf '═%.0s' $(seq 1 $w); printf "${RST}\n"

    for line in "${SUMMARY[@]}"; do
        printf "%b\n" "$line"
    done

    printf "${DIM}"; printf '─%.0s' $(seq 1 $w); printf "${RST}\n"

    if (( FAIL_N > 0 )); then overall="${RED}FAIL${RST}"
    else                       overall="${GRN}PASS${RST}"
    fi
    printf "  ${BLD}%-14s %2d pass  %2d fail  %2d skip       %b${RST}  ${DIM}%s${RST}\n" \
           "TOTAL" "$PASS_N" "$FAIL_N" "$SKIP_N" "$overall" "$(elapsed $total_time)"

    printf "${WHT}"; printf '═%.0s' $(seq 1 $w); printf "${RST}\n"
    printf "  ${DIM}Logs: %s/${RST}\n" "$LOGDIR"
    echo
}

# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════
banner
check_tools

$DO_LINT    && run_lint
$DO_COMPILE && run_compile
$DO_FORMAL  && run_formal
$DO_SIM     && run_sim

print_summary
exit $FAIL_N
