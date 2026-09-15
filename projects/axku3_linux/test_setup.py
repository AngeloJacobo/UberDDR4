"""Offline checks for cache handling; never downloads or accesses the board."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import zipfile

from prepare_cpu import checked_source, RELATIVE, PREFIX, main as prepare_cpu_main
from setup_dependencies import download, extract, submodule_command
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

    def test_only_the_needed_submodules_are_requested(self):
        """A blanket recursive init drags in VexRiscv test data no build uses.

        Its path nests deep enough to fail every Git for Windows clone, so the
        lock file must keep naming just the picolibc sources the BIOS needs.
        """
        lock = json.loads(Path(__file__).with_name('dependencies.json').read_text())
        declared = {name for name, spec in lock['repositories'].items() if spec.get('submodules')}
        self.assertEqual(declared, {'pythondata-software-picolibc'})
        command = submodule_command(Path('cache'), lock['repositories']['pythondata-software-picolibc'])
        self.assertIn('-c', command)
        self.assertIn('core.longpaths=true', command)
        self.assertEqual(command[-2:], ['--', 'pythondata_software_picolibc/data'])
        self.assertIsNone(submodule_command(Path('cache'), lock['repositories']['litex']))


BASH = shutil.which('bash')
ENTRY_POINT = Path(__file__).with_name('uberddr4.sh')


@unittest.skipUnless(BASH, 'Shell entry point requires bash')
class CleanupTests(unittest.TestCase):
    """Run the real cleanup command only against disposable project fixtures."""

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
        # Cleanup must work without loading the environment or executing
        # arbitrary local settings that might point at an external directory.
        (self.project / 'local.sh').write_text("echo 'Do not load local settings' >&2\nexit 1\n")
        self.script = self.project / 'uberddr4.sh'
        shutil.copyfile(ENTRY_POINT, self.script)

    def run_cleanup(self, *flags):
        return subprocess.run([BASH, str(self.script), 'clean', '--yes', *flags],
                              capture_output=True, text=True, timeout=60)

    def test_default_keeps_dependencies_and_sources(self):
        result = self.run_cleanup()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.output.exists())
        self.assertTrue(self.dependency.exists())
        self.assertTrue(self.source.exists())
        self.assertTrue((self.project / 'local.sh').exists())

    def test_all_keeps_project_sources(self):
        result = self.run_cleanup('--all')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((self.project / 'build').exists())
        self.assertTrue(self.source.exists())
        self.assertTrue(self.script.exists())

    def test_dry_run_never_deletes(self):
        for flags in (('--dry-run',), ('--all', '--dry-run')):
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

    def test_confirmation_is_required_without_a_terminal(self):
        """Without --yes and without a terminal, refuse rather than assume yes."""
        result = subprocess.run([BASH, str(self.script), 'clean'],
                                capture_output=True, text=True, timeout=60,
                                stdin=subprocess.DEVNULL)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Refusing to delete without confirmation', result.stderr)
        self.assertTrue((self.output / 'dummy.bit').exists())

    def test_nested_link_is_refused_before_deletion(self):
        outside = self.root / 'outside-project'
        outside.mkdir()
        sentinel = outside / 'keep.txt'
        sentinel.write_text('must survive')
        link = self.output / 'linked'
        if os.name == 'nt':
            # A junction needs no elevation, unlike a Windows directory symlink.
            created = subprocess.run(['cmd', '/c', 'mklink', '/J', str(link), str(outside)],
                                     capture_output=True, text=True, timeout=30)
            self.assertEqual(created.returncode, 0, created.stderr)
        else:
            link.symlink_to(outside, target_is_directory=True)
        result = self.run_cleanup()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Refusing linked cleanup entry', result.stderr)
        self.assertTrue(sentinel.exists())
        self.assertTrue((self.output / 'dummy.bit').exists())


@unittest.skipUnless(BASH, 'Shell entry point requires bash')
class LauncherTests(unittest.TestCase):
    """Exercise the real Python launcher inside the shell entry point.

    Sourcing with UBERDDR4_SH_NO_MAIN keeps the script from running a command,
    so run_python can be called directly with a disposable child program.
    """

    BODY = ('UBERDDR4_SH_NO_MAIN=1 . "$1"\n'
            'PYTHON_BIN="$2"\n'
            'shift 2\n'
            'run_python "$@"\n'
            'echo LAUNCHER_COMPLETE\n')

    def run_launcher(self, source, *arguments):
        with tempfile.TemporaryDirectory(prefix='uberddr4-launch-test-') as directory:
            script = Path(directory) / 'child with spaces.py'
            script.write_text(source, encoding='utf-8')
            # Paths travel as positional arguments so no shell quoting rule has
            # to cope with spaces, apostrophes or Windows separators.
            return subprocess.run(
                [BASH, '-c', self.BODY, 'uberddr4-launcher-test', str(ENTRY_POINT),
                 sys.executable, str(script), *(str(value) for value in arguments)],
                capture_output=True, text=True, timeout=60)

    def test_stdout_and_descendant_use_pipes(self):
        source = ("import subprocess, sys\n"
                  "print('STDOUT_PIPED=' + str(not sys.stdout.isatty()))\n"
                  "subprocess.run([sys.executable, '-c', "
                  "\"import sys; print('CHILD_PIPED=' + str(not sys.stdout.isatty()))\"], check=True)\n")
        result = self.run_launcher(source)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('STDOUT_PIPED=True', result.stdout)
        self.assertIn('CHILD_PIPED=True', result.stdout)
        self.assertIn('LAUNCHER_COMPLETE', result.stdout)

    def test_stderr_logging_and_argument_boundaries(self):
        argument = "a path with spaces & an apostrophe's value"
        source = ("import sys\nprint(sys.argv[1])\n"
                  "print('ORDINARY_STDERR_LOG', file=sys.stderr)\n")
        result = self.run_launcher(source, argument)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(argument, result.stdout)
        self.assertIn('ORDINARY_STDERR_LOG', result.stderr)
        self.assertIn('LAUNCHER_COMPLETE', result.stdout)

    def test_nonzero_exit_stops_the_caller(self):
        result = self.run_launcher("import sys\nprint('BEFORE_FAILURE')\nsys.exit(7)\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('BEFORE_FAILURE', result.stdout)
        self.assertIn('exit code 7', result.stderr)
        self.assertNotIn('LAUNCHER_COMPLETE', result.stdout)


@unittest.skipUnless(BASH, 'Shell entry point requires bash')
class PythonEnvironmentTests(unittest.TestCase):
    """Check the interpreter settings every command inherits.

    Windows Python writes to a console with WriteConsoleW, which failed with
    WinError 1 on the SoC generator's stdout while stderr on the same console
    kept working. The legacy setting keeps every step on the plain WriteFile
    path, which behaves the same on a console, a pipe and a file.
    """

    BODY = ('UBERDDR4_SH_NO_MAIN=1 . "$1"\n'
            'PYTHON="$2"\n'
            'CACHE_ROOT="$3"\n'
            'init_environment >/dev/null\n'
            'printf "HOST_OS=%s LEGACY=%s ENCODING=%s\\n" "$HOST_OS" '
            '"${PYTHONLEGACYWINDOWSSTDIO:-unset}" "${PYTHONIOENCODING:-unset}"\n')

    def test_stdout_avoids_the_windows_console_api(self):
        with tempfile.TemporaryDirectory(prefix='uberddr4-env-test-') as directory:
            if ' ' in directory:
                self.skipTest('Cache paths with spaces are rejected by design')
            result = subprocess.run(
                [BASH, '-c', self.BODY, 'uberddr4-environment-test', str(ENTRY_POINT),
                 sys.executable, directory],
                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('ENCODING=utf-8', result.stdout)
        # Only Windows has the console API this avoids; elsewhere the variable
        # would be meaningless and must stay out of the child's environment.
        if 'HOST_OS=windows' in result.stdout:
            self.assertIn('LEGACY=1', result.stdout)
        else:
            self.assertIn('LEGACY=unset', result.stdout)


class ConsoleProgramTests(unittest.TestCase):
    """Run the console program the shell embeds, with the serial port stubbed.

    It is carried as a single-quoted shell string, so it must contain no
    single quote, and it has to ask for the Windows console escape handling
    itself: miniterm requests that only when platform.release() reads exactly
    "10", and Windows 11 reads "11", which leaves the Linux prompt arriving as
    literal text such as ESC[01;32m.
    """

    @staticmethod
    def program():
        lines = ENTRY_POINT.read_text(encoding='utf-8').splitlines()
        start = next(index for index, line in enumerate(lines)
                     if line.strip() == "local console_code='")
        end = next(index for index in range(start + 1, len(lines))
                   if lines[index] == "'")
        return '\n'.join(lines[start + 1:end])

    def load(self):
        """Execute the program with the serial package replaced by stubs."""
        writes = []
        started = []

        class Serial:
            def write(self, data):
                writes.append(bytes(data))
                return len(data)

        serial = types.ModuleType('serial')
        serial.Serial = Serial
        serial.SerialException = type('SerialException', (Exception,), {})
        tools = types.ModuleType('serial.tools')
        miniterm = types.ModuleType('serial.tools.miniterm')
        miniterm.main = lambda: started.append('main')
        tools.miniterm = miniterm
        serial.tools = tools
        namespace = {'__name__': 'uberddr4_console'}
        stubs = {'serial': serial, 'serial.tools': tools,
                 'serial.tools.miniterm': miniterm}
        with patch.dict(sys.modules, stubs):
            exec(compile(self.program(), 'console_code', 'exec'), namespace)
        return namespace, Serial, writes, started

    def test_the_program_survives_the_shell_quoting_that_carries_it(self):
        self.assertNotIn("'", self.program())

    def test_pacing_transmits_one_byte_per_write(self):
        _, serial_class, writes, _ = self.load()
        self.assertEqual(serial_class().write(b'abc'), 3)
        self.assertEqual(writes, [b'a', b'b', b'c'])

    def test_console_escape_handling_wraps_the_terminal(self):
        namespace, _, _, started = self.load()
        self.assertEqual(started, ['main'])
        source = self.program()
        self.assertLess(source.index('saved_console_mode = enable_console_ansi()'),
                        source.index('miniterm.main()'))
        # The mode has to be put back even when the terminal raises.
        self.assertIn('finally:\n    restore_console_mode(saved_console_mode)', source)
        # Asking twice reports no second change, whether this test runs on a
        # console that already had the flag, one that did not, or no console.
        saved = namespace['enable_console_ansi']()
        try:
            self.assertIsNone(namespace['enable_console_ansi']())
        finally:
            namespace['restore_console_mode'](saved)


@unittest.skipUnless(BASH, 'Shell entry point requires bash')
class OptionValidationTests(unittest.TestCase):
    """Reject bad option values before any download, Vivado run or deletion."""

    def check(self, snippet):
        body = f'UBERDDR4_SH_NO_MAIN=1 . "$1"\n{snippet}\n'
        return subprocess.run([BASH, '-c', body, 'uberddr4-option-test', str(ENTRY_POINT)],
                              capture_output=True, text=True, timeout=60)

    def test_data_rate_outside_the_supported_set_is_rejected(self):
        self.assertEqual(self.check('validate_set --data-rate 2400 "${DATA_RATES[@]}"').returncode, 0)
        rejected = self.check('validate_set --data-rate 9999 "${DATA_RATES[@]}"')
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn('--data-rate must be one of', rejected.stderr)

    def test_build_variant_rejects_path_and_shell_characters(self):
        self.assertEqual(self.check('validate_variant ok-name_1').returncode, 0)
        for value in ('../escape', 'has space', 'semi;colon'):
            with self.subTest(value=value):
                rejected = self.check(f'validate_variant "{value}"')
                self.assertNotEqual(rejected.returncode, 0)
                self.assertIn('--build-variant may contain only', rejected.stderr)

    def test_numeric_ranges_are_enforced(self):
        self.assertEqual(self.check('validate_range --trials 10 1 100').returncode, 0)
        for value in ('0', '101', 'ten'):
            with self.subTest(value=value):
                self.assertNotEqual(self.check(f'validate_range --trials {value} 1 100').returncode, 0)

    def test_unknown_command_is_reported(self):
        result = subprocess.run([BASH, str(ENTRY_POINT), 'not-a-command'],
                                capture_output=True, text=True, timeout=60)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Unknown command', result.stderr)


@unittest.skipUnless(BASH, 'Shell entry point requires bash')
class AllCommandTests(unittest.TestCase):
    """Check the whole-flow command's step order and options without running one.

    run_step is replaced, so no download, no Vivado run and no board access
    happens; only the sequence the command would start is recorded.
    """

    STUB = r'''
UBERDDR4_SH_NO_MAIN=1 . "$1"
RECORD="$2"
FAIL_STEP="$3"
shift 3

run_step() {
    printf '%s\n' "$*" >>"$RECORD"
    [[ "$1" != "$FAIL_STEP" ]] || die "Step failed: uberddr4.sh $*"
}

cmd_all "$@"
'''

    def run_all(self, *arguments, fail_step=''):
        directory = tempfile.mkdtemp(prefix='uberddr4-all-test-')
        self.addCleanup(shutil.rmtree, directory, True)
        record = Path(directory) / 'steps.txt'
        record.touch()
        result = subprocess.run(
            [BASH, '-c', self.STUB, 'uberddr4-all-test', str(ENTRY_POINT),
             str(record), fail_step, *arguments],
            capture_output=True, text=True, timeout=60)
        return result, record.read_text().splitlines()

    def test_default_run_covers_setup_through_implement_in_order(self):
        result, steps = self.run_all()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(steps, [
            'setup',
            'test',
            'build --data-rate 2400 --uart-name serial',
            'payload --data-rate 2400',
            'build --synthesize-only --data-rate 2400 --uart-name serial',
            'implement --data-rate 2400',
        ])
        self.assertIn('ALL_PASS', result.stdout)

    def test_options_reach_the_steps_that_take_them(self):
        _, steps = self.run_all('--data-rate', '2133', '--build-variant', 'trial_a',
                                '--uart-baudrate', '115200', '--skip-setup')
        self.assertEqual(steps, [
            'test',
            'build --data-rate 2133 --uart-name serial --uart-baudrate 115200 '
            '--build-variant trial_a',
            'payload --data-rate 2133 --build-variant trial_a',
            'build --synthesize-only --data-rate 2133 --uart-name serial '
            '--uart-baudrate 115200 --build-variant trial_a',
            'implement --data-rate 2133 --build-variant trial_a',
        ])

    def test_a_failing_step_stops_the_run(self):
        result, steps = self.run_all(fail_step='payload')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Step failed: uberddr4.sh payload', result.stderr)
        # Synthesis and implementation must not start after a failure.
        self.assertEqual(steps[-1].split()[0], 'payload')
        self.assertNotIn('ALL_PASS', result.stdout)

    def test_bad_options_are_rejected_before_the_first_step(self):
        for arguments in (('--data-rate', '9999'), ('--uart-name', 'spi'),
                          ('--build-variant', '../escape'), ('--uart-baudrate', '10'),
                          ('--data-rate',), ('--unknown-option',)):
            with self.subTest(arguments=arguments):
                result, steps = self.run_all(*arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(steps, [])


@unittest.skipUnless(BASH, 'Shell entry point requires bash')
class CampaignTests(unittest.TestCase):
    """Drive the repeat loop with a stubbed trial runner, no Vivado and no board.

    The stub stands in for one program/upload/boot/test batch: it writes the
    per-round summary the real runner writes, and fails the round the test asks
    it to fail, so the loop, the campaign log and the tally are exercised.
    """

    STUB = r'''
UBERDDR4_SH_NO_MAIN=1 . "$1"
BUILD_ROOT="$2"
FAIL_ROUND="$3"
shift 3

run_trials() {
    local label="$9"
    shift 9
    printf '%s\n' "$*" >>"$BUILD_ROOT/extra-options.txt"
    local results="$BUILD_ROOT/hardware/$label"
    mkdir -p "$results"
    printf 'trial\tresult\n' >"$results/summary.tsv"
    if [[ "$label" == *"round0$FAIL_ROUND" ]]; then
        printf '1\tPASS\n2\tFAIL\n' >>"$results/summary.tsv"
        return 1
    fi
    printf '1\tPASS\n2\tPASS\n' >>"$results/summary.tsv"
}

cmd_campaign "$@"
'''

    def run_campaign(self, fail_round, *arguments):
        directory = tempfile.mkdtemp(prefix='uberddr4-campaign-test-')
        self.addCleanup(shutil.rmtree, directory, True)
        result = subprocess.run(
            [BASH, '-c', self.STUB, 'uberddr4-campaign-test', str(ENTRY_POINT),
             directory, str(fail_round), *arguments],
            capture_output=True, text=True, timeout=60)
        logs = sorted(Path(directory).glob('hardware/campaign_*/campaign.tsv'))
        return result, Path(directory), logs

    def test_a_failed_round_is_recorded_and_the_soak_continues(self):
        result, root, logs = self.run_campaign(2, '--repeat', '3', '--trials', '2')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Failed campaign rounds: 2', result.stderr)
        self.assertNotIn('CAMPAIGN_PASS', result.stdout)
        self.assertEqual(len(logs), 1)
        rows = logs[0].read_text().splitlines()
        self.assertEqual(len(rows), 4)
        self.assertEqual([row.split('\t')[-1] for row in rows[1:]], ['PASS', 'FAIL', 'PASS'])
        # Five of six trials passed; a stopped soak could not have said that.
        self.assertIn('CAMPAIGN_ROUNDS_PASS=2', result.stdout)
        self.assertIn('CAMPAIGN_ROUNDS_FAIL=1', result.stdout)
        self.assertIn('CAMPAIGN_TRIALS_PASS=5', result.stdout)
        self.assertIn('CAMPAIGN_TRIALS_FAIL=1', result.stdout)
        # Failures inside a batch must not end it either.
        self.assertEqual((root / 'extra-options.txt').read_text().split('\n')[0],
                         '--continue-on-failure')

    def test_stop_on_failure_ends_the_soak_at_the_first_failed_round(self):
        result, root, logs = self.run_campaign(1, '--repeat', '4', '--trials', '2',
                                               '--stop-on-failure')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(len(logs[0].read_text().splitlines()), 2)
        self.assertIn('CAMPAIGN_ROUNDS_FAIL=1', result.stdout)
        self.assertEqual((root / 'extra-options.txt').read_text().strip(), '')

    def test_every_round_passing_reports_a_passing_campaign(self):
        result, _, logs = self.run_campaign(0, '--repeat', '2', '--trials', '2')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('CAMPAIGN_PASS', result.stdout)
        self.assertIn('CAMPAIGN_TRIALS_PASS=4', result.stdout)
        self.assertIn('CAMPAIGN_TRIALS_FAIL=0', result.stdout)
        self.assertEqual(len(logs[0].read_text().splitlines()), 3)

    def test_bad_campaign_options_are_rejected_before_the_board_is_touched(self):
        for arguments in (('--repeat', '0'), ('--repeat', 'many'), ('--repeat', '101'),
                          ('--trials', '0'), ('--data-rate', '9999'),
                          ('--build-variant', '../escape'), ('--repeat',),
                          ('--unknown-option',)):
            with self.subTest(arguments=arguments):
                result, root, logs = self.run_campaign(0, *arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(logs, [])
                self.assertFalse((root / 'hardware').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
