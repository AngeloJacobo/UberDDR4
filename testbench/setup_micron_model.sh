#!/usr/bin/env bash
#
# setup_micron_model.sh  -  Create symlinks for the Micron DDR4 behavioral model
#
# The Micron DDR4 model ships with Vivado and cannot be redistributed.
# This script creates symlinks in testbench/micron/ pointing to the model
# files in your Vivado installation, so both the CLI flow (run_xsim.sh)
# and the Vivado GUI can compile them directly.
#
# Prerequisites:
#   source /path/to/Vivado/2023.1/settings64.sh   # sets $XILINX_VIVADO
#
# Usage (from repo root):
#   ./testbench/setup_micron_model.sh
#
# Safe to re-run: generated links or copies are overwritten in place.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MICRON_DIR="$SCRIPT_DIR/micron"

if [[ -z "${XILINX_VIVADO:-}" ]]; then
    echo "ERROR: XILINX_VIVADO is not set. Source Vivado settings64.sh first." >&2
    exit 1
fi

SRC="$XILINX_VIVADO/data/ip/xilinx/ddr4_v2_2/data/dlib/ultrascale/ddr4_sdram/tb/ddr4_model"

if [[ ! -d "$SRC" ]]; then
    echo "ERROR: Micron DDR4 model not found at:" >&2
    echo "  $SRC" >&2
    echo "Check your Vivado installation (need ddr4_v2_2 IP)." >&2
    exit 1
fi

MICRON_FILES=(
    arch_defines.v
    arch_package.sv
    proj_package.sv
    interface.sv
    StateTable.sv
    StateTableCore.sv
    MemoryArray.sv
    ddr4_model.sv
    timing_tasks.sv
)

mkdir -p "$MICRON_DIR"

for f in "${MICRON_FILES[@]}"; do
    if [[ ! -f "$SRC/$f" ]]; then
        echo "ERROR: Required Micron model file is missing:" >&2
        echo "  $SRC/$f" >&2
        exit 1
    fi
done

echo "Installing Micron model files in $MICRON_DIR ..."

for f in "${MICRON_FILES[@]}"; do
    # Git Bash may be unable to create native Windows symlinks unless
    # Developer Mode or elevated privileges are enabled.  Prefer links, but
    # fall back to copies so the same setup command works on Windows and Linux.
    if ln -sf "$SRC/$f" "$MICRON_DIR/$f" 2>/dev/null; then
        echo "  linked: $f"
    else
        cp -f "$SRC/$f" "$MICRON_DIR/$f"
        echo "  copied: $f"
    fi
done

echo ""
echo "Done. Installed ${#MICRON_FILES[@]} Micron DDR4 model files."
echo "You can now run:  ./testbench/run_xsim.sh"
