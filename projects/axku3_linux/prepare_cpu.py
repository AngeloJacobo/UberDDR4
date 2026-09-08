"""Validate the upstream CPU source and prepare its isolated build snapshot.

With --output, copy the package marker and RAM helper, then prepend SYNTHESIS
to the selected CPU Verilog. Without --output, only validate the source cache.
The upstream checkout is never edited; unexpected cached modifications fail.
"""

import argparse
from pathlib import Path
import shutil
import subprocess

CPU = 'VexRiscvLitexSmpCluster_Cc1_Iw32Is4096Iy1_Dw32Ds4096Dy1_ITs4DTs4_Ood_Wm.v'
RELATIVE = 'pythondata_cpu_vexriscv_smp/verilog/' + CPU
PREFIX = b'`define SYNTHESIS\n'


def checked_source(repository):
    """Accept upstream bytes or the one known LiteX-added SYNTHESIS prefix."""
    expected = subprocess.check_output(['git', '-C', str(repository), 'show', 'HEAD:' + RELATIVE])
    actual = (repository / RELATIVE).read_bytes().replace(b'\r\n', b'\n')
    expected = expected.replace(b'\r\n', b'\n')
    if actual not in (expected, PREFIX + expected):
        raise RuntimeError('CPU RTL differs from the pinned source beyond the documented SYNTHESIS prefix')
    return expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = checked_source(args.repository)
    if args.output:
        target = args.output / 'pythondata_cpu_vexriscv_smp'
        (target / 'verilog').mkdir(parents=True, exist_ok=True)
        shutil.copyfile(args.repository / 'pythondata_cpu_vexriscv_smp/__init__.py', target / '__init__.py')
        shutil.copyfile(args.repository / 'pythondata_cpu_vexriscv_smp/verilog/Ram_1w_1rs_Generic.v',
                        target / 'verilog/Ram_1w_1rs_Generic.v')
        (target / 'verilog' / CPU).write_bytes(PREFIX + source)
    print('CPU_SOURCE_PASS: pinned RTL with explicit SYNTHESIS prefix')


if __name__ == '__main__':
    main()
