# Copy to local.ps1 (ignored by Git) only to override installed-tool locations.
# Dependencies and outputs otherwise stay in this project's build/ directory.
@{
    Python = 'C:\Python312\python.exe'
    VivadoRoot = 'C:\Xilinx\Vivado\2022.2'
    GitRoot = 'C:\Program Files\Git'
    # Optional: use an external cache if your checkout is synced or has spaces.
    # CacheRoot = 'C:\fpga-build\uberddr4-linux'
}
