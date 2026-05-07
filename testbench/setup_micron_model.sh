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
#   ./UberDDR4/testbench/setup_micron_model.sh
#
# Safe to re-run: ln -sf overwrites existing symlinks without error.
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
    arch_package.sv
    proj_package.sv
    interface.sv
    StateTable.sv
    StateTableCore.sv
    MemoryArray.sv
    ddr4_model.sv
)

echo "Creating symlinks in $MICRON_DIR ..."

for f in "${MICRON_FILES[@]}"; do
    ln -sf "$SRC/$f" "$MICRON_DIR/$f"
    echo "  $f -> $SRC/$f"
done

echo ""
echo "Done. Symlinks created for ${#MICRON_FILES[@]} Micron DDR4 model files."
echo "You can now run:  ./UberDDR4/testbench/run_xsim.sh"
