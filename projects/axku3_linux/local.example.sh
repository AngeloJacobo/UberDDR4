# Copy to local.sh (ignored by Git) only to override installed-tool locations.
# Dependencies and outputs otherwise stay in this project's build/ directory.
# This file is sourced by uberddr4.sh, so use plain shell assignments.

PYTHON='C:\Python312\python.exe'
VIVADO_ROOT='C:\Xilinx\Vivado\2022.2'
GIT_ROOT='C:\Program Files\Git'

# Optional: use an external cache if your checkout is synced or has spaces.
# CACHE_ROOT='/c/fpga-build/uberddr4-linux'
