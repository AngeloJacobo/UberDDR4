"""Offline checks for cache handling; never downloads or accesses the board."""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import zipfile

from prepare_cpu import checked_source, RELATIVE, PREFIX, main as prepare_cpu_main
from setup_dependencies import download, extract
from validate_linux_generated import validate_source_paths


class SetupTests(unittest.TestCase):
    """Use temporary files and mocks to exercise safe failure paths offline."""
    def test_cpu_snapshot_includes_ram_helper(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source'
            output = root / 'output'
            cpu = source / RELATIVE
            cpu.parent.mkdir(parents=True)
            cpu.write_bytes(b'module cpu; endmodule\n')
            (cpu.parent / 'Ram_1w_1rs_Generic.v').write_bytes(b'module ram; endmodule\n')
            (cpu.parent.parent / '__init__.py').write_bytes(b'data_location = "verilog"\n')
            with patch('prepare_cpu.subprocess.check_output', return_value=cpu.read_bytes()), \
                 patch('sys.argv', ['prepare_cpu.py', '--repository', str(source), '--output', str(output)]):
                prepare_cpu_main()
            self.assertEqual((output / RELATIVE).read_bytes(), PREFIX + cpu.read_bytes())
            self.assertEqual((output / RELATIVE).with_name('Ram_1w_1rs_Generic.v').read_bytes(),
                             (cpu.parent / 'Ram_1w_1rs_Generic.v').read_bytes())

    def test_synthesis_inputs_must_exist(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'top.v').write_text('module top; endmodule')
            validate_source_paths('read_verilog {top.v}\n', root)
            with self.assertRaisesRegex(RuntimeError, 'Missing synthesis input'):
                validate_source_paths('read_verilog {top.v}\nread_verilog {missing.v}\n', root)

    def test_cached_archive_checksum_is_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'archive.zip'
            path.write_bytes(b'cached')
            with patch('urllib.request.urlopen') as network:
                download('https://invalid.example', path, hashlib.sha256(b'cached').hexdigest())
                with self.assertRaisesRegex(RuntimeError, 'checksum mismatch'):
                    download('https://invalid.example', path, '0' * 64)
                network.assert_not_called()

    def test_extraction_rejects_parent_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / 'bad.zip'
            with zipfile.ZipFile(archive, 'w') as output:
                output.writestr('../outside', b'bad')
            with self.assertRaisesRegex(RuntimeError, 'Unsafe archive'):
                extract(archive, root / 'destination')
            self.assertFalse((root / 'outside').exists())

    def test_extracts_normal_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / 'good.zip'
            with zipfile.ZipFile(archive, 'w') as output:
                output.writestr('nested/file', b'good')
            extract(archive, root / 'destination')
            self.assertEqual((root / 'destination/nested/file').read_bytes(), b'good')

    def test_cpu_accepts_only_exact_documented_prefix(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / RELATIVE
            path.parent.mkdir(parents=True)
            source = b'module cpu; endmodule\n'
            with patch('prepare_cpu.subprocess.check_output', return_value=source):
                for value in (source, PREFIX + source):
                    path.write_bytes(value)
                    self.assertEqual(checked_source(root), source)
                path.write_bytes(PREFIX + source + b'// unexpected edit\n')
                with self.assertRaisesRegex(RuntimeError, 'differs'):
                    checked_source(root)


@unittest.skipUnless(os.name == 'nt', 'Windows PowerShell cleanup script')
class CleanupTests(unittest.TestCase):
    """Run the real cleanup script only against disposable project fixtures."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='uberddr4-clean-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.project = self.root / 'project'
        self.output = self.project / 'build/output'
        self.output.mkdir(parents=True)
        (self.output / 'dummy.bit').write_bytes(b'fixture only')
        self.dependency = self.project / 'build/dependencies/keep.txt'
        self.dependency.parent.mkdir()
        self.dependency.write_text('dependency fixture')
        self.source = self.project / 'source.v'
        self.source.write_text('source fixture')
        # Cleanup must work without importing the environment or executing
        # arbitrary local settings that might point at an external directory.
        (self.project / 'local.ps1').write_text("throw 'Do not load local settings'\n")
        self.script = self.project / 'clean.ps1'
        shutil.copyfile(Path(__file__).with_name('clean.ps1'), self.script)

    def run_cleanup(self, *flags):
        command = "& '" + str(self.script).replace("'", "''") + "' -Confirm:$false " + ' '.join(flags)
        return subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive',
                               '-Command', command], capture_output=True, text=True, timeout=30)

    def test_default_keeps_dependencies_and_sources(self):
        result = self.run_cleanup()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.output.exists())
        self.assertTrue(self.dependency.exists())
        self.assertTrue(self.source.exists())
        self.assertTrue((self.project / 'local.ps1').exists())

    def test_all_keeps_project_sources(self):
        result = self.run_cleanup('-All')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((self.project / 'build').exists())
        self.assertTrue(self.source.exists())
        self.assertTrue(self.script.exists())

    def test_whatif_never_deletes(self):
        for flags in (('-WhatIf',), ('-All', '-WhatIf')):
            result = self.run_cleanup(*flags)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((self.output / 'dummy.bit').exists())
            self.assertTrue(self.dependency.exists())

    def test_missing_output_is_harmless(self):
        self.output.rename(self.project / 'build/other-output')
        result = self.run_cleanup()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Nothing to clean', result.stdout)
        self.assertTrue(self.dependency.exists())

    def test_nested_junction_is_refused_before_deletion(self):
        outside = self.root / 'outside-project'
        outside.mkdir()
        sentinel = outside / 'keep.txt'
        sentinel.write_text('must survive')
        link = self.output / 'linked'
        quote = lambda p: "'" + str(p).replace("'", "''") + "'"
        result = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command',
                                 f"New-Item -ItemType Junction -Path {quote(link)} -Target {quote(outside)}"],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = self.run_cleanup()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Refusing linked cleanup entry', result.stderr)
        self.assertTrue(sentinel.exists())
        self.assertTrue((self.output / 'dummy.bit').exists())


@unittest.skipUnless(os.name == 'nt', 'Windows PowerShell launchers')
class LauncherTests(unittest.TestCase):
    """Exercise the real batch launcher in each installed PowerShell version."""

    def run_launcher(self, source, *arguments):
        quote = lambda value: "'" + str(value).replace("'", "''") + "'"
        environment = Path(__file__).with_name('environment.ps1').resolve()
        shells = [shutil.which(name) for name in ('powershell.exe', 'pwsh.exe')]
        self.assertIsNotNone(shells[0], 'Windows PowerShell must be available')
        with tempfile.TemporaryDirectory(prefix='uberddr4-launch-test-') as directory:
            script = Path(directory) / 'child with spaces.py'
            script.write_text(source, encoding='utf-8')
            values = ','.join(quote(v) for v in (script, *arguments))
            command = ("$ErrorActionPreference='Stop'; $BuildRoot=''; $LinuxDepsRoot=''; "
                       f". {quote(environment)}; Invoke-ProjectPython -ArgumentList @({values}); "
                       "Write-Output 'LAUNCHER_COMPLETE'")
            return [(shell, subprocess.run([shell, '-NoProfile', '-NonInteractive', '-Command', command],
                                          capture_output=True, text=True, timeout=30))
                    for shell in shells if shell]

    def test_stdout_and_descendant_use_pipes(self):
        source = ("import subprocess, sys\n"
                  "print('STDOUT_PIPED=' + str(not sys.stdout.isatty()))\n"
                  "subprocess.run([sys.executable, '-c', "
                  "\"import sys; print('CHILD_PIPED=' + str(not sys.stdout.isatty()))\"], check=True)\n")
        for shell, result in self.run_launcher(source):
            with self.subTest(shell=shell):
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('STDOUT_PIPED=True', result.stdout)
                self.assertIn('CHILD_PIPED=True', result.stdout)
                self.assertIn('LAUNCHER_COMPLETE', result.stdout)

    def test_stderr_logging_and_argument_boundaries(self):
        argument = "a path with spaces & an apostrophe's value"
        source = ("import sys\nprint(sys.argv[1])\n"
                  "print('ORDINARY_STDERR_LOG', file=sys.stderr)\n")
        for shell, result in self.run_launcher(source, argument):
            with self.subTest(shell=shell):
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(argument, result.stdout)
                self.assertIn('ORDINARY_STDERR_LOG', result.stderr)
                self.assertIn('LAUNCHER_COMPLETE', result.stdout)

    def test_nonzero_exit_stops_the_caller(self):
        for shell, result in self.run_launcher("import sys\nprint('BEFORE_FAILURE')\nsys.exit(7)\n"):
            with self.subTest(shell=shell):
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('BEFORE_FAILURE', result.stdout)
                self.assertIn('exit code 7', result.stderr)
                self.assertNotIn('LAUNCHER_COMPLETE', result.stdout)


if __name__ == '__main__':
    unittest.main(verbosity=2)
