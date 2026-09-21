#!/usr/bin/env bash
#
# uberddr4.sh - AXKU3 Linux + UberDDR4 workflow, one entry point for every step.
#
# Commands:
#   setup       Download pinned sources/tools into build/, then check the host
#   check       Read-only preflight: pinned revisions, installed tools, imports
#   test        Fast host-only regression tests (no Vivado, no board)
#   build       Generate the SoC and BIOS; optionally synthesize or implement
#   payload     Turn generated csr.json plus cached images into a UART payload
#   implement   Resume implementation from a saved synthesis checkpoint
#   all         setup, test, build, payload, synthesize and implement in order
#   boot        Program the FPGA, then load/boot/check Linux over UART
#   campaign    Repeat the boot batch as a reliability soak and tally the result
#   console     Interactive paced terminal for Linux already loaded by boot
#   clean       Remove this example's local generated files
#
# Usage:
#   ./uberddr4.sh setup
#   ./uberddr4.sh test
#   ./uberddr4.sh build [--synthesize-only | --build] [--data-rate 2400]
#                       [--uart-name serial|jtag_uart] [--uart-baudrate N]
#                       [--build-variant NAME] [--no-linux]
#   ./uberddr4.sh payload [--data-rate 2400] [--build-variant NAME]
#   ./uberddr4.sh implement [--data-rate 2400] [--build-variant NAME]
#   ./uberddr4.sh all [--data-rate 2400] [--uart-name serial|jtag_uart]
#                     [--uart-baudrate N] [--build-variant NAME] [--skip-setup]
#   ./uberddr4.sh boot [--port auto] [--trials 1] [--data-rate 2400]
#                      [--software-megabytes 16] [--sfl-frame-bytes 0]
#                      [--sfl-outstanding 1] [--uart-baudrate 1000000]
#                      [--build-variant NAME]
#   ./uberddr4.sh campaign [--repeat 3] [--trials 10] [--stop-on-failure]
#                          [--port auto] [--data-rate 2400]
#                          [--software-megabytes 16] [--sfl-frame-bytes 0]
#                          [--sfl-outstanding 1] [--uart-baudrate 1000000]
#                          [--build-variant NAME]
#   ./uberddr4.sh console [--port COM6]
#   ./uberddr4.sh clean [--all] [--dry-run] [--yes]
#
# Every command accepts --build-root and --linux-deps-root to relocate outputs.
# Copy local.example.sh to local.sh (ignored by Git) to override tool locations.
#
# One script covers every step, so the whole flow has a single place to read and
# audit. Host-specific details (executable suffixes, Vivado launchers, path
# syntax, serial port naming) are resolved once in "Host platform" below rather
# than spread through the commands.
# The Windows path is the exercised one; see README.md for the Linux caveats.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Re-invoked by the all command so each step runs in its own process.
SCRIPT_SELF="$SCRIPT_DIR/$(basename "${BASH_SOURCE[0]}")"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
note() { printf '%s\n' "$*"; }

# ===========================================================================
# Host platform
# ===========================================================================
# Only this block may inspect the operating system. Everything below uses the
# variables and helpers it defines, so adding a host means editing one place.
case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) HOST_OS=windows ;;
    *)                    HOST_OS=posix   ;;
esac

if [[ $HOST_OS == windows ]]; then
    EXE='.exe'          # Native tool suffix
    BAT='.bat'          # Vivado launcher suffix
    GEN_EXT='.bat'      # Suffix LiteX gives the build script it generates
    PATHSEP=';'         # Separator native Python expects in PYTHONPATH
    DEFAULT_PORT='COM6'
else
    EXE=''
    BAT=''
    # LiteX drops the .bat but still writes an extension: build_<name>.sh.
    # This is not the same suffix as the installed Vivado launchers above.
    GEN_EXT='.sh'
    PATHSEP=':'
    DEFAULT_PORT='/dev/ttyUSB0'
fi

# Windows Python and Vivado cannot read the /c/... form this shell uses, so
# every path handed to them is converted first. On POSIX hosts this is identity.
native() {
    if [[ $HOST_OS == windows ]]; then cygpath -w "$1"; else printf '%s' "$1"; fi
}

# Join arguments with the separator native Python expects, converting as we go.
native_path_list() {
    local joined='' entry
    for entry in "$@"; do
        joined+="$(native "$entry")$PATHSEP"
    done
    printf '%s' "${joined%$PATHSEP}"
}

# ===========================================================================
# Settings
# ===========================================================================
if [[ $HOST_OS == windows ]]; then
    PYTHON='python.exe'
    VIVADO_ROOT='C:\Xilinx\Vivado\2022.2'
    GIT_ROOT='C:\Program Files\Git'
else
    PYTHON='python3'
    VIVADO_ROOT='/opt/Xilinx/Vivado/2022.2'
    GIT_ROOT=''
fi
# Everything downloaded/generated for this example lives in its ignored build
# directory by default, regardless of the caller's working directory.
CACHE_ROOT="$SCRIPT_DIR/build"
DEPENDENCIES_ROOT=''
TOOLS_ROOT=''
LINUX_DEPS_ROOT=''
BUILD_ROOT=''

# ===========================================================================
# Argument validation helpers
# ===========================================================================
require_value() {
    # $1 flag name, $2 supplied value count remaining
    [[ ${2:-0} -gt 0 ]] || die "$1 requires a value"
}

validate_set() {
    # validate_set <flag> <value> <allowed>...
    local flag="$1" value="$2" candidate
    shift 2
    for candidate in "$@"; do
        [[ "$value" == "$candidate" ]] && return 0
    done
    die "$flag must be one of: $*"
}

validate_range() {
    # validate_range <flag> <value> <min> <max>
    local flag="$1" value="$2" min="$3" max="$4"
    [[ "$value" =~ ^-?[0-9]+$ ]] || die "$flag must be an integer"
    (( value >= min && value <= max )) || die "$flag must be between $min and $max"
}

validate_variant() {
    [[ "$1" =~ ^[A-Za-z0-9_-]*$ ]] ||
        die '--build-variant may contain only letters, digits, underscore, and hyphen'
}

DATA_RATES=(1200 1250 1600 1866 2133 2400)

# Root overrides as the user gave them, so the all command can hand the same
# ones to every step it starts.
SHARED_OVERRIDES=()

# ===========================================================================
# Derived environment
# ===========================================================================
# Resolves cache locations, locates Python, and builds the search paths that
# make this project's pinned packages win over any global installation.
init_environment() {
    # Loaded here, not at file scope, so cleanup never executes local settings
    # that could point at an external directory. Explicit command-line roots
    # still win over local.sh, so a one-off override needs no file edit.
    local cli_build_root="$BUILD_ROOT" cli_linux_deps_root="$LINUX_DEPS_ROOT"
    local cli_cache_root="$CACHE_ROOT"
    if [[ -f "$SCRIPT_DIR/local.sh" ]]; then
        # shellcheck source=/dev/null
        . "$SCRIPT_DIR/local.sh"
    fi
    [[ -z "$cli_build_root" ]]      || BUILD_ROOT="$cli_build_root"
    [[ -z "$cli_linux_deps_root" ]] || LINUX_DEPS_ROOT="$cli_linux_deps_root"
    [[ "$cli_cache_root" == "$SCRIPT_DIR/build" ]] || CACHE_ROOT="$cli_cache_root"

    [[ -n "$BUILD_ROOT" ]]       || BUILD_ROOT="$CACHE_ROOT/output"
    [[ -n "$LINUX_DEPS_ROOT" ]]  || LINUX_DEPS_ROOT="$CACHE_ROOT/linux"
    [[ -n "$DEPENDENCIES_ROOT" ]] || DEPENDENCIES_ROOT="$CACHE_ROOT/dependencies"
    [[ -n "$TOOLS_ROOT" ]]       || TOOLS_ROOT="$CACHE_ROOT/tools"

    local cache_path
    for cache_path in "$BUILD_ROOT" "$LINUX_DEPS_ROOT" "$DEPENDENCIES_ROOT" "$TOOLS_ROOT"; do
        [[ "$cache_path" =~ [[:space:]] ]] &&
            die 'Use cache/build paths without spaces; set CACHE_ROOT in local.sh.'
    done

    # PYTHON may be a bare command to look up on PATH, or an explicit location.
    # An explicit Windows path contains no forward slash, so it must be converted
    # and tested directly rather than handed to a PATH search.
    if [[ "$PYTHON" == */* || "$PYTHON" == *\\* ]]; then
        PYTHON_BIN="$(to_shell_path "$PYTHON")"
        [[ -x "$PYTHON_BIN" ]] ||
            die "Python not executable: $PYTHON. Set PYTHON in local.sh."
    else
        PYTHON_BIN="$(command -v "$PYTHON" 2>/dev/null)" ||
            die "Python not found: $PYTHON. Set PYTHON in local.sh."
        [[ -n "$PYTHON_BIN" ]] || die "Python not found: $PYTHON. Set PYTHON in local.sh."
    fi

    VIVADO_BIN="$(join_path "$(to_shell_path "$VIVADO_ROOT")" bin)"
    if [[ $HOST_OS == windows ]]; then
        MAKE_BIN="$(join_path "$(to_shell_path "$VIVADO_ROOT")" gnuwin/bin)"
        GIT_UNIX_BIN="$(join_path "$(to_shell_path "$GIT_ROOT")" usr/bin)"
        SOFTWARE_BIN="$TOOLS_ROOT/litex-software/Scripts"
        SOFTWARE_LIB="$TOOLS_ROOT/litex-software/Lib/site-packages"
        # Vivado's launcher exits 1 without any message when this is unset,
        # which a stripped environment can do. Never override a real value.
        export PROCESSOR_ARCHITECTURE="${PROCESSOR_ARCHITECTURE:-AMD64}"
    else
        # Vivado ships GNU make only for Windows; elsewhere use the system one.
        MAKE_BIN="$(dirname "$(command -v make 2>/dev/null || echo /usr/bin/make)")"
        GIT_UNIX_BIN="$(dirname "$(command -v sh 2>/dev/null || echo /bin/sh)")"
        SOFTWARE_BIN="$TOOLS_ROOT/litex-software/bin"
        SOFTWARE_LIB="$(printf '%s\n' "$TOOLS_ROOT"/litex-software/lib/python*/site-packages | head -1)"
    fi
    GCC_BIN="$TOOLS_ROOT/riscv-gcc/xpack-riscv-none-elf-gcc-15.2.0-1/bin"

    # Prefer this project's helpers and pinned packages over global installs.
    # Both CPU packages are present because the target retains a small test mode.
    PYTHON_PATHS=(
        "$SCRIPT_DIR"
        "$DEPENDENCIES_ROOT/python-packages-windows"
        "$DEPENDENCIES_ROOT/python-packages"
        "$SOFTWARE_LIB"
        "$DEPENDENCIES_ROOT/migen"
        "$DEPENDENCIES_ROOT/litex"
        "$DEPENDENCIES_ROOT/pythondata-cpu-vexriscv"
        "$DEPENDENCIES_ROOT/pythondata-software-picolibc"
        "$DEPENDENCIES_ROOT/pythondata-software-compiler_rt"
        "$LINUX_DEPS_ROOT/pythondata-cpu-vexriscv-smp"
        "$LINUX_DEPS_ROOT/python-packages"
    )
    export PYTHONPATH; PYTHONPATH="$(native_path_list "${PYTHON_PATHS[@]}")"
    export PYTHONIOENCODING='utf-8'
    export PYTHONUNBUFFERED='1'
    # When Windows Python finds a console on stdout it writes with
    # WriteConsoleW instead of WriteFile. Under the VS Code terminal's
    # pseudo-console that call failed with "[WinError 1] Incorrect function" on
    # the SoC generator's first print, after minutes of logging had already
    # reached stderr on the same console; only that one process's stdout was
    # affected, and the same command through a pipe or a file always worked.
    # This selects the plain WriteFile path, which behaves the same on a
    # console, a pipe and a file. Output stays UTF-8: the encoding above
    # applies to this path too.
    if [[ $HOST_OS == windows ]]; then
        export PYTHONLEGACYWINDOWSSTDIO='1'
    fi
}

# Accept a native (C:\...) or shell (/c/...) path and return the shell form.
to_shell_path() {
    if [[ $HOST_OS == windows ]]; then cygpath -u "$1"; else printf '%s' "$1"; fi
}

# Append to a directory without doubling the separator. Git for Windows installs
# at the MSYS root, so converting its location yields a bare "/".
join_path() {
    printf '%s/%s' "${1%/}" "$2"
}

# Run a project Python command, converting path arguments for the interpreter.
# No output redirection is needed here: this
# shell already hands the child ordinary pipes, and stderr stays separate so
# routine logging cannot be mistaken for a failure.
run_python() {
    "$PYTHON_BIN" "$@"
    local status=$?
    (( status == 0 )) || die "Python command failed with exit code $status: $1"
}

# ===========================================================================
# check - read-only preflight
# ===========================================================================
# A failure stops the build early; this command does not repair or install tools.
cmd_check() {
    local entries name commit is_linux root path actual dirty allowed
    entries="$("$PYTHON_BIN" -c '
import json, sys
# Native Windows Python would end each line with CRLF, leaving a stray carriage
# return in the last field the shell reads back.
sys.stdout.reconfigure(newline="\n")
lock = json.load(open(sys.argv[1], encoding="utf-8"))
for name, spec in lock["repositories"].items():
    print("\t".join([name, spec["commit"], "1" if spec.get("linux") else "0"]))
' "$(native "$SCRIPT_DIR/dependencies.json")")" || die 'Cannot read dependencies.json'

    while IFS=$'\t' read -r name commit is_linux; do
        [[ -n "$name" ]] || continue
        if [[ "$is_linux" == 1 ]]; then root="$LINUX_DEPS_ROOT"; else root="$DEPENDENCIES_ROOT"; fi
        path="$root/$name"
        [[ -d "$path" ]] || die "Missing dependency: $path. Run './uberddr4.sh setup'."
        actual="$(git -c "safe.directory=$(native "$path")" -C "$path" rev-parse HEAD)" ||
            die "Cannot inspect dependency: $path"
        [[ "$actual" == "$commit" ]] || die "Wrong revision for $name; expected $commit."
        dirty="$(git -c "safe.directory=$(native "$path")" -C "$path" status --porcelain --untracked-files=no)" ||
            die "Cannot inspect dependency: $path"
        # The upstream LiteX flow can add this one CPU prefix. Accept only its
        # exact documented contents; arbitrary local edits are not a
        # reproducible input.
        if [[ -n "$dirty" ]]; then
            allowed=' M pythondata_cpu_vexriscv_smp/verilog/VexRiscvLitexSmpCluster_Cc1_Iw32Is4096Iy1_Dw32Ds4096Dy1_ITs4DTs4_Ood_Wm.v'
            if [[ "$name" != 'pythondata-cpu-vexriscv-smp' || "$(printf '%s\n' "$dirty" | wc -l)" -ne 1 || "$dirty" != "$allowed" ]]; then
                die "Dependency has modified tracked files: $path"
            fi
            run_python "$(native "$SCRIPT_DIR/prepare_cpu.py")" --repository "$(native "$path")"
        fi
    done <<< "$entries"

    local tool
    for tool in "$VIVADO_BIN/vivado$BAT" "$MAKE_BIN/make$EXE" "$GIT_UNIX_BIN/sh$EXE" \
                "$GCC_BIN/riscv-none-elf-gcc$EXE" "$SOFTWARE_BIN/meson$EXE" \
                "$SOFTWARE_BIN/ninja$EXE"; do
        [[ -e "$tool" ]] || die "Missing tool: $tool"
    done

    run_python -c "import sys, migen, litex, serial, colorama, requests, yaml, fdt; assert sys.version_info[:2] == (3,12), 'Use Python 3.12'; print('Python dependencies OK')"
    note "Environment OK; output: $BUILD_ROOT"
}

# ===========================================================================
# setup - one-time network setup
# ===========================================================================
cmd_setup() {
    note 'Fetching pinned source/tools into the project cache. No drivers or global settings are changed.'
    run_python "$(native "$SCRIPT_DIR/setup_dependencies.py")" \
        --deps-root "$(native "$DEPENDENCIES_ROOT")" \
        --tools-root "$(native "$TOOLS_ROOT")" \
        --linux-root "$(native "$LINUX_DEPS_ROOT")"
    cmd_check
}

# ===========================================================================
# test - fast host-only tests
# ===========================================================================
# Simulated bus transactions, mocked UART, and cache checks. Requires the
# pinned environment, but does not run Vivado or access the board.
cmd_test() {
    cmd_check
    ( cd "$REPO_ROOT" && "$PYTHON_BIN" "$(native "$SCRIPT_DIR/test_target.py")" ) ||
        die 'test_target.py failed'
    ( cd "$REPO_ROOT" && "$PYTHON_BIN" "$(native "$SCRIPT_DIR/test_setup.py")" ) ||
        die 'test_setup.py failed'
}

# ===========================================================================
# build - generate the SoC and BIOS
# ===========================================================================
# Vivado runs only with --synthesize-only or --build.
cmd_build() {
    local do_build=false do_synth=false linux=true
    local data_rate=0 uart_name='serial' uart_baudrate=0 build_variant=''
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --build)            do_build=true ;;
            --synthesize-only)  do_synth=true ;;
            --no-linux)         linux=false ;;
            --data-rate)        require_value "$1" $(($#-1)); data_rate="$2"; shift ;;
            --uart-name)        require_value "$1" $(($#-1)); uart_name="$2"; shift ;;
            --uart-baudrate)    require_value "$1" $(($#-1)); uart_baudrate="$2"; shift ;;
            --build-variant)    require_value "$1" $(($#-1)); build_variant="$2"; shift ;;
            *) die "Unknown build option: $1" ;;
        esac
        shift
    done

    $do_build && $do_synth && die '--build and --synthesize-only are mutually exclusive'
    [[ "$data_rate" == 0 ]] || validate_set '--data-rate' "$data_rate" "${DATA_RATES[@]}"
    validate_set '--uart-name' "$uart_name" serial jtag_uart
    validate_variant "$build_variant"

    local effective_rate=2400
    [[ "$data_rate" == 0 ]] || effective_rate="$data_rate"

    local snapshot_leaf output_leaf
    if $linux; then
        snapshot_leaf="source-uberddr4-linux-$effective_rate"
        output_leaf="axku3_vexriscv_linux_uberddr4_$effective_rate"
    else
        if [[ "$effective_rate" == 2400 ]]; then
            snapshot_leaf='source-uberddr4'
        else
            snapshot_leaf="source-uberddr4-$effective_rate"
        fi
        if [[ "$uart_name" == serial ]]; then
            output_leaf='axku3_vexriscv_uberddr4'
        else
            output_leaf='axku3_vexriscv_uberddr4_jtag'
        fi
        [[ "$effective_rate" == 2400 ]] || output_leaf+="_$effective_rate"
    fi
    [[ -z "$build_variant" ]] || { snapshot_leaf+="-$build_variant"; output_leaf+="_$build_variant"; }

    local snapshot_root="$BUILD_ROOT/$snapshot_leaf"
    local rtl_snapshot="$snapshot_root/rtl"
    local output_dir="$BUILD_ROOT/build/$output_leaf"
    local build_name='axku3_vexriscv_uberddr4'

    cmd_check

    if $linux; then
        [[ "$uart_name" == serial ]] || die 'Linux hardware boot requires the physical serial UART'
        local smp_data="$LINUX_DEPS_ROOT/pythondata-cpu-vexriscv-smp/pythondata_cpu_vexriscv_smp/__init__.py"
        [[ -f "$smp_data" ]] || die "Missing pinned VexRiscv-SMP data package: $smp_data"
    fi
    if [[ "$uart_baudrate" == 0 ]]; then
        if $linux; then uart_baudrate=1000000; else uart_baudrate=115200; fi
    fi
    validate_range '--uart-baudrate' "$uart_baudrate" 9600 3000000

    # Work from an isolated snapshot so file sync and source edits cannot
    # interrupt a long Vivado run. Only debug attributes are stripped below.
    mkdir -p "$snapshot_root/constraints" "$rtl_snapshot/phy"
    local name
    for name in axku3_platform.py axku3_uberddr4.py validate_generated.py; do
        cp -f "$SCRIPT_DIR/$name" "$snapshot_root/"
    done
    cp -f "$SCRIPT_DIR/constraints/axku3_uberddr4.xdc" "$snapshot_root/constraints/"
    for name in ddr4_top.v ddr4_controller.v ddr4_prober.v ddr4_phy.v; do
        cp -f "$REPO_ROOT/rtl/$name" "$rtl_snapshot/"
    done
    for name in ddr4_phy_native.v ddr4_phy_native_adapter.v ddr4_phy_native_byte.v ddr4_phy_native_reset.v; do
        cp -f "$REPO_ROOT/rtl/phy/$name" "$rtl_snapshot/phy/"
    done

    # The reusable controller RTL deliberately marks extensive bring-up probes.
    # This Linux example has no ILA, so preserving those otherwise-dead nets
    # blocks physical optimization and can create dangling-debug DRC warnings.
    # Strip only MARK_DEBUG attributes from the isolated build snapshot; all
    # RTL, CDC attributes, and the repository sources remain unchanged.
    local verilog
    while IFS= read -r verilog; do
        "$PYTHON_BIN" -c '
import sys
path, standalone, trailing = sys.argv[1:4]
with open(path, "r", encoding="utf-8", newline="") as stream:
    text = stream.read()
text = text.replace(standalone, "").replace(trailing, "")
with open(path, "w", encoding="utf-8", newline="") as stream:
    stream.write(text)
' "$(native "$verilog")" '(* mark_debug = "true" *)' ', mark_debug = "true"' ||
            die "Cannot strip debug attributes from $verilog"
    done < <(find "$rtl_snapshot" -name '*.v' -type f)

    local extra_python_paths=()
    if $linux; then
        local cpu_snapshot="$snapshot_root/cpu-data"
        run_python "$(native "$SCRIPT_DIR/prepare_cpu.py")" \
            --repository "$(native "$LINUX_DEPS_ROOT/pythondata-cpu-vexriscv-smp")" \
            --output "$(native "$cpu_snapshot")"
        extra_python_paths=("$cpu_snapshot")
    fi
    export PYTHONPATH; PYTHONPATH="$(native_path_list "${extra_python_paths[@]}" "${PYTHON_PATHS[@]}")"
    export PATH="$SOFTWARE_BIN:$GCC_BIN:$GIT_UNIX_BIN:$MAKE_BIN:$VIVADO_BIN:$PATH"
    export LITEX_ENV_CC_TRIPLE='riscv-none-elf'
    export PYTHONIOENCODING='utf-8'
    # LiteX's generated Makefiles substitute these verbatim. The interpreter is
    # quoted with forward slashes so a Windows path survives the shell they use.
    export PYTHON; PYTHON="\"$(native "$PYTHON_BIN" | tr '\\' '/')\""
    if [[ $HOST_OS == windows ]]; then
        # Vivado's bundled GNU make must not invoke cmd.exe as its shell.
        export SHELL='sh.exe'
        export MAKEFLAGS='SHELL=sh.exe'
    fi

    local generate_args=(
        "$(native "$snapshot_root/axku3_uberddr4.py")"
        --output-dir "$(native "$output_dir")"
        --uberddr4-rtl-dir "$(native "$rtl_snapshot")"
        --uart-name "$uart_name"
        --uart-baudrate "$uart_baudrate"
        --data-rate "$effective_rate"
    )
    $linux && generate_args+=(--linux)

    pushd "$snapshot_root" >/dev/null || die "Cannot enter $snapshot_root"

    run_python "${generate_args[@]}"

    local validator='validate_generated.py'
    $linux && validator='validate_linux_generated.py'
    $linux && cp -f "$SCRIPT_DIR/$validator" "$snapshot_root/"
    local validator_args=(
        "$(native "$snapshot_root/$validator")"
        --output-dir "$(native "$output_dir")"
        --uart-name "$uart_name"
        --reference-xdc "$(native "$REPO_ROOT/example_demo/axku3/axku3_uberddr4.xdc")"
    )
    $linux && validator_args+=(--uart-baudrate "$uart_baudrate" --data-rate "$effective_rate")
    run_python "${validator_args[@]}"

    # Stop here for the default command. The generated-file checks above catch
    # missing sources and wrong configuration before spending time on Vivado.
    if $do_build || $do_synth; then
        local gateware_dir="$output_dir/gateware"
        local vivado_script="$gateware_dir/build_$build_name$GEN_EXT"
        [[ -f "$vivado_script" ]] || die "Missing generated Vivado launcher: $vivado_script"
        pushd "$gateware_dir" >/dev/null || die "Cannot enter $gateware_dir"
        if $do_synth; then
            local full_tcl="$gateware_dir/$build_name.tcl"
            local synth_tcl="$gateware_dir/${build_name}_synth_only.tcl"
            write_synthesis_tcl "$full_tcl" "$synth_tcl" "$build_name"
            "$VIVADO_BIN/vivado$BAT" -mode batch -source "$(native "$synth_tcl")" ||
                die "Vivado build failed with exit code $?"
            run_python "$(native "$SCRIPT_DIR/validate_synthesis.py")" \
                --gateware-dir "$(native "$gateware_dir")" --data-rate "$effective_rate"
        else
            run_generated_launcher "$vivado_script" || die "Vivado build failed with exit code $?"
            local implementation_args=(
                "$(native "$SCRIPT_DIR/validate_implementation.py")"
                --gateware-dir "$(native "$gateware_dir")"
                --uart-name "$uart_name"
            )
            $linux && implementation_args+=(--linux)
            implementation_args+=(--data-rate "$effective_rate")
            run_python "${implementation_args[@]}"
        fi
        popd >/dev/null || true
    fi

    popd >/dev/null || true
}

# The generated launcher is a .bat on Windows and a shell script elsewhere.
run_generated_launcher() {
    if [[ $HOST_OS == windows ]]; then
        cmd //c "$(native "$1")"
    else
        sh "$1"
    fi
}

# Truncate the generated Tcl at the implementation boundary and stop after
# synthesis reports. Refuse an unfamiliar layout rather than run the full flow.
write_synthesis_tcl() {
    local full_tcl="$1" synth_tcl="$2" build_name="$3"
    [[ -f "$full_tcl" ]] || die "Cannot locate synthesis boundary in $full_tcl"
    "$PYTHON_BIN" -c '
import sys
full_tcl, synth_tcl, build_name = sys.argv[1:4]
with open(full_tcl, "r", encoding="utf-8", newline="") as stream:
    text = stream.read()
marker = "# Add pre-optimize commands"
index = text.find(marker)
if index < 0:
    raise SystemExit("Cannot locate synthesis boundary in " + full_tcl)
tail = (
    "report_clock_utilization -file {0}_clock_utilization_synth.rpt\n"
    "report_timing_summary -report_unconstrained -file {0}_timing_unconstrained_synth.rpt\n"
    "check_timing -verbose -file {0}_check_timing_synth.rpt\n"
    "quit\n"
).format(build_name)
with open(synth_tcl, "w", encoding="utf-8", newline="") as stream:
    stream.write(text[:index] + tail)
' "$(native "$full_tcl")" "$(native "$synth_tcl")" "$build_name" ||
        die "Cannot locate synthesis boundary in $full_tcl"
}

# ===========================================================================
# payload - build the UART boot payload
# ===========================================================================
# Run after build; output goes under BUILD_ROOT/payload-<configuration>.
# This does not program the FPGA or open a serial port.
cmd_payload() {
    local data_rate=2400 build_variant=''
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --data-rate)     require_value "$1" $(($#-1)); data_rate="$2"; shift ;;
            --build-variant) require_value "$1" $(($#-1)); build_variant="$2"; shift ;;
            *) die "Unknown payload option: $1" ;;
        esac
        shift
    done
    validate_set '--data-rate' "$data_rate" "${DATA_RATES[@]}"
    validate_variant "$build_variant"
    local suffix=''; [[ -z "$build_variant" ]] || suffix="_$build_variant"

    local csr_json="$BUILD_ROOT/build/axku3_vexriscv_linux_uberddr4_${data_rate}${suffix}/csr.json"
    local image_dir="$LINUX_DEPS_ROOT/images-2022"
    local output_dir="$BUILD_ROOT/payload-axku3-uberddr4-${data_rate}${suffix}"
    local fdt_package="$LINUX_DEPS_ROOT/python-packages"

    local path
    for path in "$csr_json" "$image_dir" "$fdt_package"; do
        [[ -e "$path" ]] || die "Missing Linux payload dependency: $path"
    done

    run_python "$(native "$SCRIPT_DIR/prepare_linux_payload.py")" \
        --csr-json "$(native "$csr_json")" \
        --image-dir "$(native "$image_dir")" \
        --output-dir "$(native "$output_dir")"
}

# ===========================================================================
# implement - resume from a synthesis checkpoint
# ===========================================================================
# Reuse the generated Tcl's implementation steps; do not synthesize again.
# Produces routed reports and a bitstream, then runs the hardware-use validator.
cmd_implement() {
    local data_rate=2400 build_variant=''
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --data-rate)     require_value "$1" $(($#-1)); data_rate="$2"; shift ;;
            --build-variant) require_value "$1" $(($#-1)); build_variant="$2"; shift ;;
            *) die "Unknown implement option: $1" ;;
        esac
        shift
    done
    validate_set '--data-rate' "$data_rate" "${DATA_RATES[@]}"
    validate_variant "$build_variant"

    local output_leaf="axku3_vexriscv_linux_uberddr4_$data_rate"
    [[ -z "$build_variant" ]] || output_leaf+="_$build_variant"
    local gateware="$BUILD_ROOT/build/$output_leaf/gateware"
    local stem='axku3_vexriscv_uberddr4'
    local full_tcl="$gateware/$stem.tcl"
    local synth_dcp="$gateware/${stem}_synth.dcp"
    local resume_tcl="$gateware/${stem}_resume_implementation.tcl"

    local path
    for path in "$gateware" "$full_tcl" "$synth_dcp" "$VIVADO_BIN"; do
        [[ -e "$path" ]] || die "Required implementation input is absent: $path"
    done

    run_python "$(native "$SCRIPT_DIR/validate_synthesis.py")" \
        --gateware-dir "$(native "$gateware")" --data-rate "$data_rate"

    # This boundary is supplied by the pinned LiteX Vivado backend. Refuse an
    # unfamiliar script layout rather than accidentally rerunning synthesis.
    "$PYTHON_BIN" -c '
import sys
full_tcl, resume_tcl, dcp = sys.argv[1:4]
with open(full_tcl, "r", encoding="utf-8", newline="") as stream:
    text = stream.read()
marker = "# Add pre-optimize commands"
index = text.find(marker)
if index < 0:
    raise SystemExit("Cannot locate implementation boundary in " + full_tcl)
body = "open_checkpoint {" + dcp.replace("\\", "/") + "}\n" + text[index:]
with open(resume_tcl, "w", encoding="utf-8", newline="") as stream:
    stream.write(body)
' "$(native "$full_tcl")" "$(native "$resume_tcl")" "$(native "$synth_dcp")" ||
        die "Cannot locate implementation boundary in $full_tcl"

    export PATH="$VIVADO_BIN:$PATH"
    pushd "$gateware" >/dev/null || die "Cannot enter $gateware"
    "$VIVADO_BIN/vivado$BAT" -mode batch -source "$(native "$resume_tcl")" ||
        die "Vivado implementation failed with exit code $?"
    popd >/dev/null || true

    run_python "$(native "$SCRIPT_DIR/validate_implementation.py")" \
        --gateware-dir "$(native "$gateware")" \
        --uart-name serial --linux --data-rate "$data_rate"
}

# ===========================================================================
# all - setup through implement in one command
# ===========================================================================
# The documented sequence run end to end: setup, test, generate, payload,
# synthesize, implement. Each step runs as a separate process, exactly as
# chaining the commands with && does, so nothing a long build exports can leak
# into a later step, and any failing step stops the run. This is the Linux
# flow; boot and campaign need the board and stay separate commands.
cmd_all() {
    local data_rate=2400 uart_name='serial' uart_baudrate=0 build_variant=''
    local skip_setup=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --data-rate)     require_value "$1" $(($#-1)); data_rate="$2"; shift ;;
            --uart-name)     require_value "$1" $(($#-1)); uart_name="$2"; shift ;;
            --uart-baudrate) require_value "$1" $(($#-1)); uart_baudrate="$2"; shift ;;
            --build-variant) require_value "$1" $(($#-1)); build_variant="$2"; shift ;;
            --skip-setup)    skip_setup=true ;;
            *) die "Unknown all option: $1" ;;
        esac
        shift
    done
    # Check every value before the first download or Vivado run: a typo meant
    # for the last step must not surface an hour into the sequence.
    validate_set '--data-rate' "$data_rate" "${DATA_RATES[@]}"
    validate_set '--uart-name' "$uart_name" serial jtag_uart
    validate_variant "$build_variant"
    [[ "$uart_baudrate" == 0 ]] ||
        validate_range '--uart-baudrate' "$uart_baudrate" 9600 3000000

    local build_options=(--data-rate "$data_rate" --uart-name "$uart_name")
    [[ "$uart_baudrate" == 0 ]] || build_options+=(--uart-baudrate "$uart_baudrate")
    local target_options=(--data-rate "$data_rate")
    if [[ -n "$build_variant" ]]; then
        build_options+=(--build-variant "$build_variant")
        target_options+=(--build-variant "$build_variant")
    fi

    local started; started="$(date +%Y-%m-%dT%H:%M:%S)"
    note "ALL_BEGIN data_rate=$data_rate started=$started"
    $skip_setup || run_step setup
    run_step test
    # Generate and prepare the payload before synthesizing. Both take seconds,
    # so a bad configuration surfaces ahead of the long Vivado runs, not after.
    run_step build "${build_options[@]}"
    run_step payload "${target_options[@]}"
    run_step build --synthesize-only "${build_options[@]}"
    run_step implement "${target_options[@]}"

    note ''
    note "ALL_PASS started=$started finished=$(date +%Y-%m-%dT%H:%M:%S)"
    note "Next, with the board connected: ./uberddr4.sh boot --port $DEFAULT_PORT"
}

# One step, in its own process, with the root overrides the user supplied.
run_step() {
    note ''
    note "ALL_STEP: uberddr4.sh $*"
    "$BASH" "$SCRIPT_SELF" "${SHARED_OVERRIDES[@]+"${SHARED_OVERRIDES[@]}"}" "$@" ||
        die "Step failed: uberddr4.sh $*"
}

# ===========================================================================
# boot - program, load and check Linux
# ===========================================================================
# Close other serial terminals first. Default is one trial; --trials 10 repeats
# the entire cycle. No synthesis or boot-flash programming occurs here.
cmd_boot() {
    local trials=1 software_megabytes=16 sfl_frame_bytes=0 sfl_outstanding=1
    local uart_baudrate=1000000 data_rate=2400 build_variant='' port='auto'
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --trials)             require_value "$1" $(($#-1)); trials="$2"; shift ;;
            --software-megabytes) require_value "$1" $(($#-1)); software_megabytes="$2"; shift ;;
            --sfl-frame-bytes)    require_value "$1" $(($#-1)); sfl_frame_bytes="$2"; shift ;;
            --sfl-outstanding)    require_value "$1" $(($#-1)); sfl_outstanding="$2"; shift ;;
            --uart-baudrate)      require_value "$1" $(($#-1)); uart_baudrate="$2"; shift ;;
            --data-rate)          require_value "$1" $(($#-1)); data_rate="$2"; shift ;;
            --build-variant)      require_value "$1" $(($#-1)); build_variant="$2"; shift ;;
            --port)               require_value "$1" $(($#-1)); port="$2"; shift ;;
            *) die "Unknown boot option: $1" ;;
        esac
        shift
    done
    validate_range '--trials' "$trials" 1 100
    validate_range '--software-megabytes' "$software_megabytes" 1 256
    validate_range '--sfl-frame-bytes' "$sfl_frame_bytes" 0 251
    validate_range '--sfl-outstanding' "$sfl_outstanding" 1 8
    validate_range '--uart-baudrate' "$uart_baudrate" 9600 3000000
    validate_set '--data-rate' "$data_rate" "${DATA_RATES[@]}"
    validate_variant "$build_variant"

    [[ "$sfl_frame_bytes" == 0 ]] && sfl_frame_bytes=251

    # Give each campaign a separate evidence directory; keep failed trial logs too.
    local stamp; stamp="$(date +%Y%m%d_%H%M%S)"
    run_trials "$data_rate" "$build_variant" "$port" "$uart_baudrate" "$trials" \
        "$software_megabytes" "$sfl_frame_bytes" "$sfl_outstanding" \
        "linux_uberddr4_${data_rate}_$stamp" ||
        die "Hardware trials failed with exit code $?"
}

# ===========================================================================
# Shared trial runner
# ===========================================================================
# boot and campaign differ only in how many batches they run and whether one
# failed trial ends the batch, so both resolve artifacts, refuse an unchecked
# build and reach the board through this one function. It returns the runner's
# exit status instead of exiting, which lets a campaign record a failed round
# and continue. Arguments are positional and fixed:
#   $1 data rate   $2 build variant   $3 port          $4 UART baud rate
#   $5 trials      $6 software MiB    $7 SFL frame     $8 SFL outstanding
#   $9 evidence directory name, then any extra runner options.
run_trials() {
    local data_rate="$1" build_variant="$2" port="$3" uart_baudrate="$4"
    local trials="$5" software_megabytes="$6" sfl_frame_bytes="$7"
    local sfl_outstanding="$8" label="$9"
    shift 9

    local suffix=''; [[ -z "$build_variant" ]] || suffix="_$build_variant"

    local gateware="$BUILD_ROOT/build/axku3_vexriscv_linux_uberddr4_${data_rate}${suffix}/gateware"
    local bitstream="$gateware/axku3_vexriscv_uberddr4.bit"
    local payload="$BUILD_ROOT/payload-axku3-uberddr4-${data_rate}${suffix}"
    local boot_json="$payload/boot.json"
    local xsdb="$VIVADO_BIN/xsdb$BAT"
    local hw_server="$VIVADO_BIN/hw_server$BAT"
    local program_tcl="$SCRIPT_DIR/linux_hardware_trials.tcl"

    local path
    for path in "$gateware" "$bitstream" "$boot_json" "$PYTHON_BIN" "$xsdb" "$hw_server" "$program_tcl"; do
        [[ -e "$path" ]] || die "Required path is absent: $path"
    done

    # Do not program an image with missing reports or unreviewed timing/DRC issues.
    run_python "$(native "$SCRIPT_DIR/validate_implementation.py")" \
        --gateware-dir "$(native "$gateware")" \
        --uart-name serial --linux --data-rate "$data_rate"

    start_hardware_server "$hw_server"

    "$PYTHON_BIN" "$(native "$SCRIPT_DIR/linux_hardware_trials.py")" \
        --port "$port" --baudrate "$uart_baudrate" --trials "$trials" \
        --software-megabytes "$software_megabytes" --sfl-frame-bytes "$sfl_frame_bytes" \
        --sfl-outstanding "$sfl_outstanding" --data-rate "$data_rate" \
        --bitstream "$(native "$bitstream")" --boot-json "$(native "$boot_json")" \
        --xsdb "$(native "$xsdb")" --program-tcl "$(native "$program_tcl")" \
        --output-dir "$(native "$BUILD_ROOT/hardware/$label")" "$@"
}

# ===========================================================================
# campaign - repeat the trial batch to measure reliability
# ===========================================================================
# A single passing boot shows the flow works once. Reliability is a rate, so
# this repeats the whole program/upload/boot/test batch and reports how many
# rounds and trials passed. By default a failure is recorded and the campaign
# continues, because stopping at the first one only proves that one happened;
# --stop-on-failure keeps the board in its failed state for inspection instead.
cmd_campaign() {
    local repeat=3 trials=10 software_megabytes=16 sfl_frame_bytes=0 sfl_outstanding=1
    local uart_baudrate=1000000 data_rate=2400 build_variant='' port='auto'
    local stop_on_failure=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --repeat)             require_value "$1" $(($#-1)); repeat="$2"; shift ;;
            --trials)             require_value "$1" $(($#-1)); trials="$2"; shift ;;
            --software-megabytes) require_value "$1" $(($#-1)); software_megabytes="$2"; shift ;;
            --sfl-frame-bytes)    require_value "$1" $(($#-1)); sfl_frame_bytes="$2"; shift ;;
            --sfl-outstanding)    require_value "$1" $(($#-1)); sfl_outstanding="$2"; shift ;;
            --uart-baudrate)      require_value "$1" $(($#-1)); uart_baudrate="$2"; shift ;;
            --data-rate)          require_value "$1" $(($#-1)); data_rate="$2"; shift ;;
            --build-variant)      require_value "$1" $(($#-1)); build_variant="$2"; shift ;;
            --port)               require_value "$1" $(($#-1)); port="$2"; shift ;;
            --stop-on-failure)    stop_on_failure=true ;;
            *) die "Unknown campaign option: $1" ;;
        esac
        shift
    done
    validate_range '--repeat' "$repeat" 1 100
    validate_range '--trials' "$trials" 1 100
    validate_range '--software-megabytes' "$software_megabytes" 1 256
    validate_range '--sfl-frame-bytes' "$sfl_frame_bytes" 0 251
    validate_range '--sfl-outstanding' "$sfl_outstanding" 1 8
    validate_range '--uart-baudrate' "$uart_baudrate" 9600 3000000
    validate_set '--data-rate' "$data_rate" "${DATA_RATES[@]}"
    validate_variant "$build_variant"

    [[ "$sfl_frame_bytes" == 0 ]] && sfl_frame_bytes=251

    local extra=()
    $stop_on_failure || extra=(--continue-on-failure)

    local stamp; stamp="$(date +%Y%m%d_%H%M%S)"
    local campaign_dir="$BUILD_ROOT/hardware/campaign_${data_rate}_$stamp"
    mkdir -p "$campaign_dir" || die "Cannot create campaign directory: $campaign_dir"
    local log="$campaign_dir/campaign.tsv"
    printf 'round\tstarted\tfinished\tevidence\tresult\n' >"$log" ||
        die "Cannot write campaign log: $log"

    note "CAMPAIGN_BEGIN rounds=$repeat trials_per_round=$trials data_rate=$data_rate"
    note "CAMPAIGN_LOG=$log"

    local round passed=0 failed=0 label started finished verdict
    local failed_rounds=()
    for ((round = 1; round <= repeat; round++)); do
        label="linux_uberddr4_${data_rate}_${stamp}_round$(printf '%02d' "$round")"
        note ''
        note "CAMPAIGN_ROUND_${round}_BEGIN of $repeat -> $label"
        started="$(date +%Y-%m-%dT%H:%M:%S)"
        # A failed round leaves its transcripts on disk and the next round
        # reprograms the FPGA, which recovers the board from whatever state it
        # was left in. Only an absent artifact or a broken host aborts outright.
        if run_trials "$data_rate" "$build_variant" "$port" "$uart_baudrate" "$trials" \
            "$software_megabytes" "$sfl_frame_bytes" "$sfl_outstanding" \
            "$label" "${extra[@]+"${extra[@]}"}"; then
            verdict=PASS; passed=$((passed + 1))
        else
            verdict=FAIL; failed=$((failed + 1)); failed_rounds+=("$round")
        fi
        finished="$(date +%Y-%m-%dT%H:%M:%S)"
        printf '%s\t%s\t%s\t%s\t%s\n' "$round" "$started" "$finished" "$label" "$verdict" >>"$log"
        note "CAMPAIGN_ROUND_${round}_$verdict"
        if [[ "$verdict" == FAIL ]] && $stop_on_failure; then
            note 'Stopping: --stop-on-failure was given.'
            break
        fi
    done

    # Trial-level totals read back from the per-round records, not assumed from
    # the requested count: a round that stopped early contributed fewer trials.
    local trial_pass=0 trial_fail=0 summary
    for summary in "$BUILD_ROOT/hardware/linux_uberddr4_${data_rate}_${stamp}_round"*/summary.tsv; do
        [[ -f "$summary" ]] || continue
        trial_pass=$((trial_pass + $(awk -F'\t' 'NR > 1 && $NF == "PASS"' "$summary" | wc -l)))
        trial_fail=$((trial_fail + $(awk -F'\t' 'NR > 1 && $NF == "FAIL"' "$summary" | wc -l)))
    done

    note ''
    note "CAMPAIGN_ROUNDS_PASS=$passed"
    note "CAMPAIGN_ROUNDS_FAIL=$failed"
    note "CAMPAIGN_TRIALS_PASS=$trial_pass"
    note "CAMPAIGN_TRIALS_FAIL=$trial_fail"
    note "CAMPAIGN_LOG=$log"
    (( failed == 0 )) ||
        die "Failed campaign rounds: ${failed_rounds[*]}"
    note 'CAMPAIGN_PASS'
}

# Reuse an already running hw_server; otherwise start one without a visible
# window and give it a moment to open its port.
start_hardware_server() {
    local hw_server="$1"
    if hardware_server_running; then return 0; fi
    if [[ $HOST_OS == windows ]]; then
        cmd //c "start /b \"\" \"$(native "$hw_server")\" -s tcp::3121" >/dev/null 2>&1
    else
        nohup "$hw_server" -s tcp::3121 >/dev/null 2>&1 &
        disown 2>/dev/null || true
    fi
    sleep 2
}

hardware_server_running() {
    if [[ $HOST_OS == windows ]]; then
        tasklist //FI "IMAGENAME eq hw_server.exe" 2>/dev/null | grep -qi 'hw_server.exe'
    else
        pgrep -x hw_server >/dev/null 2>&1
    fi
}

# ===========================================================================
# console - interactive terminal
# ===========================================================================
# For Linux already loaded by boot. It does not boot, reset or test the board.
# Ctrl+] releases the serial port before another boot.
cmd_console() {
    local port="$DEFAULT_PORT"
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --port) require_value "$1" $(($#-1)); port="$2"; shift ;;
            *) die "Unknown console option: $1" ;;
        esac
        shift
    done

    note 'Press Enter for the Linux prompt. Ctrl+] closes the console.'
    note 'Input is paced at up to 200 characters/second for reliable command pasting.'
    # Pace every transmitted byte, including separate writes from pasted
    # keystrokes. Linux 5.14 LiteUART polls a small RX FIFO; reads and output
    # remain unrestricted.
    local console_code='
import sys
import time
import serial
from serial.tools import miniterm

original_write = serial.Serial.write

def paced_write(port, data):
    for value in data:
        if original_write(port, bytes([value])) != 1:
            raise serial.SerialException("Short console write")
        time.sleep(0.005)
    return len(data)

def enable_console_ansi():
    """Let a Windows console interpret the escapes the board sends.

    miniterm asks for virtual-terminal processing only when
    platform.release() reads exactly "10"; on Windows 11 it reads "11", so
    the shell prompt arrives as literal text like ESC[01;32m. Returns what
    is needed to undo the change, or None when there was nothing to do.
    """
    if sys.platform != "win32":
        return None
    import ctypes
    from ctypes import wintypes
    # A private handle keeps these prototypes off the shared windll object.
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetStdHandle.restype = wintypes.HANDLE
    kernel32.GetStdHandle.argtypes = [wintypes.DWORD]
    kernel32.GetConsoleMode.argtypes = [wintypes.HANDLE,
                                        ctypes.POINTER(wintypes.DWORD)]
    kernel32.SetConsoleMode.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    handle = kernel32.GetStdHandle(-11)
    mode = wintypes.DWORD()
    if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
        return None
    virtual_terminal = 0x0004
    if mode.value & virtual_terminal:
        return None
    if not kernel32.SetConsoleMode(handle, mode.value | virtual_terminal):
        return None
    return kernel32, handle, mode.value

def restore_console_mode(saved):
    if saved is None:
        return
    kernel32, handle, mode = saved
    kernel32.SetConsoleMode(handle, mode)

serial.Serial.write = paced_write
saved_console_mode = enable_console_ansi()
try:
    miniterm.main()
finally:
    restore_console_mode(saved_console_mode)
'
    # miniterm reads keys straight from the console on Windows, which a MinTTY
    # pseudo-terminal does not provide. winpty ships with Git for Windows and
    # bridges the two; without a terminal at all, run the child directly.
    if [[ $HOST_OS == windows && -t 0 ]] && command -v winpty >/dev/null 2>&1; then
        winpty "$PYTHON_BIN" -c "$console_code" "$port" 1000000 --raw --eol LF ||
            die 'Console failed. Check the serial port and close other serial programs.'
    else
        "$PYTHON_BIN" -c "$console_code" "$port" 1000000 --raw --eol LF ||
            die 'Console failed. Check the serial port and close other serial programs.'
    fi
}

# ===========================================================================
# clean - remove local generated files
# ===========================================================================
# Default: build/output. --all: build, including downloaded dependencies/tools.
# Deliberately ignore local.sh and external cache overrides: never recursively
# delete a user-configured external path. Stop builds and save wanted logs first.
cmd_clean() {
    local all=false dry_run=false assume_yes=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --all)     all=true ;;
            --dry-run) dry_run=true ;;
            --yes|-y)  assume_yes=true ;;
            *) die "Unknown clean option: $1" ;;
        esac
        shift
    done

    # Resolve nothing here: a linked build directory must be refused below
    # rather than silently followed to wherever it points.
    local project_root="$SCRIPT_DIR" target
    if $all; then target="$project_root/build"; else target="$project_root/build/output"; fi

    # Reject junctions/symlinks in the path before accessing the target's
    # contents. A linked build directory must never turn cleanup into deletion
    # elsewhere. Walk up by string so a link cannot be resolved away first.
    local ancestor="$target"
    while [[ -n "$ancestor" && "$ancestor" != '/' ]]; do
        if [[ -L "$ancestor" ]]; then
            die "Refusing linked cleanup path: $ancestor"
        fi
        ancestor="$(dirname "$ancestor")"
    done

    # Containment is checked against the resolved project root so a relative or
    # symlinked invocation cannot move the target outside this project.
    case "$target" in
        "$project_root"/*) ;;
        *) die 'Cleanup target is outside this project.' ;;
    esac

    if [[ ! -e "$target" ]]; then
        note "Nothing to clean: $target"
        return 0
    fi
    [[ -d "$target" ]] || die "Expected a build directory, not a file: $target"

    # Refuse any nested link before deleting anything, including under --dry-run.
    local entry
    while IFS= read -r entry; do
        [[ -n "$entry" ]] || continue
        die "Refusing linked cleanup entry: $entry"
    done < <(find "$target" -type l 2>/dev/null)

    local action
    if $all; then
        action='Permanently delete ALL local dependencies, tools, build outputs and test logs'
    else
        action='Permanently delete generated outputs, including bitstreams and test logs'
    fi

    if $dry_run; then
        note "What if: Performing the operation \"$action\" on target \"$target\"."
        return 0
    fi

    if ! $assume_yes; then
        [[ -t 0 ]] || die "Refusing to delete without confirmation; pass --yes to proceed: $target"
        local reply
        printf '%s\n' "$action"
        read -r -p "Delete \"$target\" permanently? [y/N] " reply
        case "$reply" in
            [Yy]|[Yy][Ee][Ss]) ;;
            *) note 'Cancelled.'; return 0 ;;
        esac
    fi

    rm -rf "$target" || die "Cannot remove: $target"
    note "Removed permanently: $target"
}

# ===========================================================================
# Main
# ===========================================================================
show_help() {
    sed -n '3,/^set -o pipefail/{ /^set -o pipefail/d; s/^# \?//p; }' "$0"
}

main() {
    [[ $# -gt 0 ]] || { show_help; exit 1; }
    local command="$1"; shift

    case "$command" in
        -h|--help|help) show_help; exit 0 ;;
        setup|check|test|build|payload|implement|all|boot|campaign|console|clean) ;;
        # Reject an unknown command before resolving tools or reading settings,
        # so a typo reports itself instead of an unrelated environment problem.
        *) printf 'Unknown command: %s\n\n' "$command" >&2; show_help >&2; exit 1 ;;
    esac

    # Shared options are accepted before or after the command's own flags.
    local passthrough=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --build-root)      require_value "$1" $(($#-1)); BUILD_ROOT="$2"; SHARED_OVERRIDES+=("$1" "$2"); shift ;;
            --linux-deps-root) require_value "$1" $(($#-1)); LINUX_DEPS_ROOT="$2"; SHARED_OVERRIDES+=("$1" "$2"); shift ;;
            --cache-root)      require_value "$1" $(($#-1)); CACHE_ROOT="$2"; SHARED_OVERRIDES+=("$1" "$2"); shift ;;
            *) passthrough+=("$1") ;;
        esac
        shift
    done

    # Cleanup deliberately runs without the environment so it cannot be pointed
    # at an external cache by local settings or command-line overrides.
    if [[ "$command" == clean ]]; then
        cmd_clean "${passthrough[@]+"${passthrough[@]}"}"
        return
    fi

    init_environment

    case "$command" in
        setup)     cmd_setup     "${passthrough[@]+"${passthrough[@]}"}" ;;
        check)     cmd_check     "${passthrough[@]+"${passthrough[@]}"}" ;;
        test)      cmd_test      "${passthrough[@]+"${passthrough[@]}"}" ;;
        build)     cmd_build     "${passthrough[@]+"${passthrough[@]}"}" ;;
        payload)   cmd_payload   "${passthrough[@]+"${passthrough[@]}"}" ;;
        implement) cmd_implement "${passthrough[@]+"${passthrough[@]}"}" ;;
        all)       cmd_all       "${passthrough[@]+"${passthrough[@]}"}" ;;
        boot)      cmd_boot      "${passthrough[@]+"${passthrough[@]}"}" ;;
        campaign)  cmd_campaign  "${passthrough[@]+"${passthrough[@]}"}" ;;
        console)   cmd_console   "${passthrough[@]+"${passthrough[@]}"}" ;;
    esac
}

# Sourcing with UBERDDR4_SH_NO_MAIN=1 defines the functions without running a
# command, which lets the offline tests exercise the launcher directly.
[[ "${UBERDDR4_SH_NO_MAIN:-}" == 1 ]] || main "$@"
