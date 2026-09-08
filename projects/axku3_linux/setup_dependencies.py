"""Download the host dependencies listed in dependencies.json; called by setup.ps1.

Git sources, Python packages, compiler and Linux images stay in the given cache
directories. No drivers, registry values or global Python installs are changed.
Network access is required; existing source edits and bad archive hashes fail
instead of being silently replaced.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request
import zipfile

from prepare_cpu import checked_source, RELATIVE


def run(*args):
    subprocess.run([str(arg) for arg in args], check=True)


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def download(url, path, digest):
    """Verify both cached and freshly downloaded bytes before trusting an archive."""
    if not path.exists():
        partial = path.with_suffix(path.suffix + '.partial')
        request = urllib.request.Request(url, headers={'User-Agent': 'UberDDR4-example'})
        with urllib.request.urlopen(request, timeout=120) as response, partial.open('wb') as output:
            shutil.copyfileobj(response, output)
        if sha256(partial).lower() != digest.lower():
            raise RuntimeError(f'Download checksum mismatch: {partial}')
        partial.replace(path)
    if sha256(path).lower() != digest.lower():
        raise RuntimeError(f'Cached archive checksum mismatch: {path}')


def extract(archive, destination):
    """Reject escaping paths and symlinks before extracting any ZIP member."""
    destination.mkdir(parents=True, exist_ok=True)
    root = destination.resolve()
    with zipfile.ZipFile(archive) as package:
        for member in package.infolist():
            target = (root / member.filename).resolve()
            if not target.is_relative_to(root) or (member.external_attr >> 16) & 0o170000 == 0o120000:
                raise RuntimeError(f'Unsafe archive member: {member.filename}')
        package.extractall(root)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deps-root', type=Path, required=True)
    parser.add_argument('--tools-root', type=Path, required=True)
    parser.add_argument('--linux-root', type=Path, required=True)
    args = parser.parse_args()
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError('Use Python 3.12 for the recorded toolchain')
    lock = json.loads(Path(__file__).with_name('dependencies.json').read_text())
    for root in (args.deps_root, args.tools_root, args.linux_root):
        root.mkdir(parents=True, exist_ok=True)
    # Source revisions are detached and fixed. Reusing a cache must not quietly
    # pull a newer branch or discard someone's dependency changes.
    for name, spec in lock['repositories'].items():
        target = (args.linux_root if spec.get('linux') else args.deps_root) / name
        if not target.exists():
            run('git', 'clone', '--no-checkout', spec['url'], target)
            run('git', '-C', target, 'checkout', '--detach', spec['commit'])
            run('git', '-C', target, 'submodule', 'update', '--init', '--recursive')
        actual = subprocess.check_output(['git', '-C', str(target), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = subprocess.check_output(['git', '-C', str(target), 'status', '--porcelain', '--untracked-files=no'], text=True)
        if name == 'pythondata-cpu-vexriscv-smp' and dirty.rstrip('\r\n') == ' M ' + RELATIVE:
            checked_source(target)
            dirty = ''
        if actual != spec['commit'] or dirty:
            raise RuntimeError(f'Existing dependency is not the clean pinned revision: {target}')

    # --target supplies importable modules; --prefix supplies meson/ninja
    # launchers as well. Both destinations are project-cache directories.
    # Keep pip's reusable download cache local too, not just installed packages.
    pip = [sys.executable, '-m', 'pip', '--disable-pip-version-check',
           '--cache-dir', args.tools_root / 'pip-cache']
    run(*pip, 'install', '--upgrade', '--target', args.deps_root / 'python-packages-windows',
        *lock['python_packages'])
    run(*pip, 'install', '--ignore-installed', '--prefix', args.tools_root / 'litex-software',
        *lock['build_packages'])

    downloads = args.tools_root / 'downloads'
    downloads.mkdir(exist_ok=True)
    wheel_name = f"fdt-{lock['fdt']['version']}-py3-none-any.whl"
    wheel = downloads / wheel_name
    if not wheel.exists():
        run(*pip, 'download', '--only-binary=:all:', '--no-deps', '--dest', downloads,
            f"fdt=={lock['fdt']['version']}")
    if sha256(wheel) != lock['fdt']['sha256']:
        raise RuntimeError('Unexpected FDT wheel hash')
    run(*pip, 'install', '--no-deps', '--upgrade', '--target', args.linux_root / 'python-packages', wheel)

    gcc_archive = downloads / 'riscv-gcc.zip'
    download(lock['gcc']['url'], gcc_archive, lock['gcc']['sha256'])
    gcc_root = args.tools_root / 'riscv-gcc'
    gcc_exe = gcc_root / 'xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe'
    if not gcc_exe.exists():
        extract(gcc_archive, gcc_root)
    if not gcc_exe.is_file():
        raise RuntimeError(f'Expected compiler absent after extraction: {gcc_exe}')

    archive = downloads / 'linux_2022_03_23.zip'
    download(lock['linux']['url'], archive, lock['linux']['sha256'])
    unpacked = args.linux_root / 'unpacked-2022'
    extract(archive, unpacked)
    images = args.linux_root / 'images-2022'
    images.mkdir(exist_ok=True)
    for name in ('Image', 'opensbi.bin', 'rootfs.cpio'):
        candidates = [path for path in unpacked.rglob(name) if path.is_file()]
        if len(candidates) != 1:
            raise RuntimeError(f'Expected one {name} in Linux archive, found {len(candidates)}')
        shutil.copyfile(candidates[0], images / name)
    print('SETUP_PASS: sources, toolchain and Linux images are cached')


if __name__ == '__main__':
    main()
