#!/usr/bin/env python3
"""Offline tests for the SoC wiring and the host's boot/check protocol.

Migen simulation checks bus handshakes and byte lanes; mocked serial replies
check failure handling. These tests do not model physical DDR4 or boot Linux.
Run through test.ps1 so the pinned LiteX/Migen packages are on PYTHONPATH.
"""

import os
import tempfile
import unittest
from unittest.mock import Mock, patch
import io
import base64
import gzip
import hashlib
import json
from pathlib import Path

from migen import Instance, Module
from migen.fhdl import verilog
from migen.sim import run_simulation
from litex.soc.interconnect import wishbone

from axku3_uberddr4 import (
    BaseSoC,
    DATA_RATE_CONFIGS,
    MAIN_RAM_SIZE,
    SYS_CLK_FREQ,
    UBERDDR4_DEBUG_BASE,
    WB_ADDR_BITS,
    WB_DATA_BITS,
    _ClassicToPipelined,
    _configure_windows_make_paths,
    litex_builder,
)
from linux_hardware_trials import PersistentTranscript, read_until, main as trials_main, value as transcript_value
import fdt
from prepare_linux_payload import (LOAD_ADDRESSES, adapt_board_dts,
                                   reserve_rv32_last_page, validate_rv32_reservation)
from serial_trace import TracedLiteXTerm, SFLUploadError
from linux_hardware_trials import ROOT_PROMPTS, write_console, test_userspace, capture_zero_failure


RTL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "rtl"))


class AXKU3UberDDR4TargetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.soc = BaseSoC(RTL_DIR)

    def test_memory_map_and_width(self):
        self.assertEqual(SYS_CLK_FREQ, 300_000_000)
        self.assertEqual(MAIN_RAM_SIZE, 0x40000000)
        self.assertEqual(self.soc.bus.regions["main_ram"].origin, 0x40000000)
        self.assertEqual(
            self.soc.bus.regions["uberddr4_debug"].origin,
            UBERDDR4_DEBUG_BASE,
        )
        self.assertEqual(len(self.soc.uberddr4_main_wb.dat_w), WB_DATA_BITS)
        self.assertEqual(len(self.soc.uberddr4_main_wb.adr), WB_ADDR_BITS)
        self.assertEqual(len(self.soc.uberddr4_main_wb.sel), 32)
        self.assertEqual(self.soc.data_rate, 2400)
        self.assertEqual(self.soc.sys_clk_freq, 300_000_000)
        self.assertEqual(self.soc.controller_clk_period, 3332)
        self.assertEqual(self.soc.ddr4_clk_period, 833)

    def test_matched_rate_configuration(self):
        self.assertEqual(DATA_RATE_CONFIGS[1200], (150_000_000, 6668, 1667))

    @unittest.skipUnless(os.name == 'nt', 'Windows make path adapter')
    def test_make_include_path_does_not_expand_object_names(self):
        output = os.path.abspath('build/output/build/linux')
        include = os.path.join(output, 'software', 'include')
        # Restore the global LiteX hook after checking this target-local adapter.
        with patch.object(litex_builder, '_makefile_escape', litex_builder._makefile_escape):
            _configure_windows_make_paths(output)
            escaped = litex_builder._makefile_escape(include)
            self.assertEqual(escaped, '../include')
            self.assertEqual(os.path.normpath(os.path.join(output, 'software', 'libc', escaped)), include)
            self.assertEqual(litex_builder._makefile_escape(' -march=rv32i2p0_ma -mabi=ilp32'),
                             ' -march=rv32i2p0_ma -mabi=ilp32')

    def test_controller_and_cpu_reset_domains_are_distinct(self):
        self.assertTrue(hasattr(self.soc.crg, "cd_uber"))
        self.assertTrue(hasattr(self.soc.crg, "cd_sys"))
        self.assertTrue(hasattr(self.soc.crg, "cd_ref"))
        self.assertFalse(hasattr(self.soc.crg, "rst"))
        self.assertFalse(hasattr(self.soc.ctrl, "soc_rst"))
        self.assertFalse(hasattr(self.soc.ctrl, "cpu_rst"))
        self.assertTrue(hasattr(self.soc.ctrl, "bus_error"))
        self.assertEqual(len(self.soc.uberddr4_init_done), 1)
        self.assertEqual(len(self.soc.uberddr4_init_failed), 1)

    def test_byte_lane_parameter_preserves_existing_rtl_width(self):
        # Fix constant sizing at the SoC boundary, without patching ddr4_prober.
        ddr = next(s for s in self.soc._fragment.specials
                   if isinstance(s, Instance) and s.of == 'ddr4_top')
        parameters = {p.name: p.value for p in ddr.items if isinstance(p, Instance.Parameter)}
        self.assertEqual(parameters['BYTE_LANES'].value, 4)
        self.assertEqual(len(parameters['BYTE_LANES']), 32)
        self.assertEqual(parameters['BIST_MODE'].value, 0)
        self.assertNotIn('BIST_ADDR_BITS_OVERRIDE', parameters)
        # Check emitted Verilog too: a correctly sized Python object is useful
        # only if the generated parameter keeps its width at the RTL boundary.
        wrapper = Module()
        wrapper.specials += Instance('ddr4_prober', p_BYTE_LANES=parameters['BYTE_LANES'])
        self.assertIn("32'd4", str(verilog.convert(wrapper)))

    def test_width_converter_preserves_word_lane_and_byte_selects(self):
        dut = Module()
        narrow = wishbone.Interface(data_width=32, adr_width=30)
        wide = wishbone.Interface(data_width=256, adr_width=26)
        dut.submodules.converter = wishbone.Converter(narrow, wide)

        def stimulus():
            yield narrow.adr.eq(7)
            yield narrow.dat_w.eq(0xDEADBEEF)
            yield narrow.sel.eq(0b0101)
            yield
            self.assertEqual((yield wide.adr), 0)
            self.assertEqual((yield wide.sel), 0b0101 << 28)
            self.assertEqual((yield wide.dat_w), 0xDEADBEEF << 224)

            yield narrow.adr.eq(8)
            yield narrow.dat_w.eq(0x12345678)
            yield narrow.sel.eq(0b1010)
            yield wide.dat_r.eq(0x89ABCDEF)
            yield
            self.assertEqual((yield wide.adr), 1)
            self.assertEqual((yield wide.sel), 0b1010)
            self.assertEqual((yield wide.dat_w), 0x12345678)
            self.assertEqual((yield narrow.dat_r), 0x89ABCDEF)

        run_simulation(dut, stimulus())

    def test_classic_bridge_issues_one_request(self):
        dut = Module()
        classic = wishbone.Interface(data_width=32, adr_width=8)
        dut.submodules.bridge = bridge = _ClassicToPipelined(classic)
        observed = []

        def stimulus():
            yield classic.cyc.eq(1)
            yield classic.stb.eq(1)
            yield bridge.stall.eq(1)
            for _ in range(2):
                observed.append((yield bridge.stb))
                yield
            yield bridge.stall.eq(0)
            observed.append((yield bridge.stb))
            yield
            for _ in range(2):
                observed.append((yield bridge.stb))
                yield
            yield bridge.ack.eq(1)
            yield
            self.assertEqual((yield classic.ack), 1)
            yield bridge.ack.eq(0)
            yield classic.cyc.eq(0)
            yield classic.stb.eq(0)
            yield

        run_simulation(dut, stimulus())
        # The first sample precedes simulator application of the driven bus
        # values. While stalled, STB remains asserted; after the single
        # accepted cycle it remains suppressed until ACK.
        self.assertEqual(observed, [0, 1, 1, 1, 0])


class AXKU3LinuxTargetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.soc = BaseSoC(
            RTL_DIR,
            uart_name="serial",
            data_rate=2400,
            cpu_type="vexriscv_smp",
            cpu_variant="linux",
            uart_baudrate=1_000_000,
            linux=True,
        )

    def test_linux_cpu_and_platform_devices(self):
        self.assertEqual(self.soc.cpu.__class__.__name__, "VexRiscvSMP")
        self.assertEqual(self.soc.cpu.variant, "linux")
        self.assertEqual(self.soc.cpu_count if hasattr(self.soc, "cpu_count") else 1, 1)
        self.assertEqual(self.soc.csr.locs["uart"], 2)
        self.assertEqual(self.soc.csr.locs["timer0"], 3)
        self.assertEqual(self.soc.bus.regions["clint"].origin, 0xF0010000)
        self.assertEqual(self.soc.bus.regions["plic"].origin, 0xF0C00000)
        self.assertEqual(self.soc.bus.regions["opensbi"].origin, 0x40F00000)

    def test_linux_uses_uberddr4_as_main_ram(self):
        self.assertEqual(self.soc.bus.regions["main_ram"].origin, 0x40000000)
        self.assertEqual(self.soc.bus.regions["main_ram"].size, 0x40000000)
        self.assertEqual(len(self.soc.uberddr4_main_wb.dat_w), 256)
        self.assertEqual(self.soc.data_rate, 2400)
        self.assertEqual(self.soc.sys_clk_freq, 300_000_000)
        self.assertEqual(self.soc.controller_clk_period, 3332)
        self.assertEqual(self.soc.ddr4_clk_period, 833)

    def test_serial_loader_jumps_to_opensbi(self):
        self.assertEqual(list(LOAD_ADDRESSES)[-1], "opensbi.bin")
        self.assertEqual(LOAD_ADDRESSES["opensbi.bin"], 0x40F00000)

    def test_dts_does_not_advertise_incompatible_reset_controller(self):
        csr = dict(csr_bases=dict(ctrl=0xf0000000), csr_registers=dict(
            ctrl_scratch=dict(addr=0xf0000000, type='rw'),
            ctrl_bus_errors=dict(addr=0xf0000004, type='ro')))
        dts = 'soc { soc_ctrl0: soc_controller@f0000000 { compatible = "litex,soc-controller"; }; serial@f0001000 {}; };'
        adapted = adapt_board_dts(dts, csr)
        self.assertNotIn('soc-controller', adapted)
        self.assertIn('serial@f0001000', adapted)
        csr['csr_registers']['ctrl_reset'] = dict(addr=0xf0000000, type='rw')
        with self.assertRaisesRegex(RuntimeError, 'Unexpected software-reset'):
            adapt_board_dts(dts, csr)

    @staticmethod
    def reserved_tree():
        dts = '/dts-v1/; / { #address-cells = <1>; #size-cells = <1>; reserved-memory { #address-cells = <1>; #size-cells = <1>; ranges; opensbi@40f00000 { reg = <0x40f00000 0x80000>; }; }; };'
        csr = {'memories': {'main_ram': {'base': 0x40000000, 'size': 0x40000000}}}
        dts = dts.replace("{", "{\n").replace(";", ";\n")
        return fdt.parse_dts(reserve_rv32_last_page(dts, csr))

    def test_last_page_reserved_in_flattened_payload(self):
        tree = fdt.parse_dtb(self.reserved_tree().to_dtb(version=17))
        validate_rv32_reservation(tree)
        node = tree.get_node('/reserved-memory/opensbi@40f00000')
        self.assertEqual(list(node.get_property('reg').data), [0x40f00000, 0x80000])

    def test_unsafe_or_incomplete_last_page_reservation_is_rejected(self):
        tree = self.reserved_tree()
        node = tree.get_node('/reserved-memory/rv32-last-page@7ffff000')
        node.get_property('reg').data[0] = 0x7fffe000
        with self.assertRaisesRegex(RuntimeError, 'reservation range'):
            validate_rv32_reservation(tree)
        node.get_property('reg').data[0] = 0x7ffff000
        node.remove_property('no-map')
        with self.assertRaisesRegex(RuntimeError, 'no-map'):
            validate_rv32_reservation(tree)
        with self.assertRaisesRegex(RuntimeError, 'reservation'):
            validate_rv32_reservation(fdt.parse_dts('/dts-v1/;\n/ {\n};\n'))

    def test_hardware_transcript_csr_parser(self):
        transcript = bytearray(b"noise\r\nUBER_STATUS=0x000010d0\r\n")
        self.assertEqual(transcript_value(transcript, "UBER_STATUS"), 0x10D0)

    def test_fixed_serial_io_does_not_reconfigure_port(self):
        class FixedPort:
            def __setattr__(self, name, value):
                raise AssertionError(f'Unexpected port reconfiguration: {name}')
            def write(self, data):
                return len(data)
            def read(self, size):
                return b'K'
        with tempfile.TemporaryDirectory() as directory, \
             patch('serial_trace.LiteXTerm.__init__', return_value=None):
            terminal = TracedLiteXTerm(trace_path=Path(directory) / 'trace.json',
                                      stable_timeouts=True)
            terminal.port = FixedPort()
            self.assertEqual(terminal.write_sfl_data(b'frame'), 5)
            self.assertEqual(terminal.read_sfl_reply(), b'K')
            self.assertEqual(terminal.tx_count, 1)

    def test_fixed_serial_io_rejects_short_write_and_empty_reply(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch('serial_trace.LiteXTerm.__init__', return_value=None):
            terminal = TracedLiteXTerm(trace_path=Path(directory) / 'trace.json',
                                      stable_timeouts=True)
            terminal.port = Mock()
            terminal.port.write.return_value = 2
            terminal.port.read.return_value = b''
            with self.assertRaisesRegex(SFLUploadError, 'Short serial write'):
                terminal.write_sfl_data(b'frame')
            with self.assertRaisesRegex(SFLUploadError, 'No serial reply'):
                terminal.read_sfl_reply()
            self.assertTrue(terminal.trace_path.is_file())

    def test_idle_bios_timeout_requires_following_positive_ack(self):
        with tempfile.TemporaryDirectory() as directory:
            terminal = TracedLiteXTerm(True, None, "0x40000000", None, False,
                trace_path=Path(directory) / 'sfl.json', stable_timeouts=True)
            terminal.outstanding = 1
            terminal.port = Mock()
            terminal.port.read.side_effect = [b'E', b'E', b'K']
            self.assertTrue(terminal.receive_upload_response())
            self.assertEqual(terminal.timeout_reply_recoveries, 1)
            self.assertEqual(terminal.recovered_timeout_replies, 2)
            for replies in ([b'E', b''], [b'E', b'C'], [b'E', b'U']):
                terminal.port.read.side_effect = replies
                with self.assertRaises(SFLUploadError):
                    terminal.receive_upload_response()
            self.assertEqual(terminal.timeout_reply_recoveries, 1)

    def test_timeout_reply_recovery_is_bounded_and_not_pipelined(self):
        with tempfile.TemporaryDirectory() as directory:
            terminal = TracedLiteXTerm(True, None, "0x40000000", None, False,
                trace_path=Path(directory) / 'sfl.json', stable_timeouts=True)
            terminal.outstanding = 1
            terminal.port = Mock()
            terminal.port.read.return_value = b'E'
            with patch('serial_trace.time.monotonic', side_effect=range(100)):
                with self.assertRaises(SFLUploadError):
                    terminal.receive_upload_response(timeout=10)
            terminal.outstanding = 2
            terminal.port.read.side_effect = [b'E', b'K']
            with self.assertRaises(SFLUploadError):
                terminal.receive_upload_response()
            self.assertEqual(terminal.timeout_reply_recoveries, 0)

    def test_hardware_transcript_is_persisted_incrementally(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial_uart.bin"
            transcript = PersistentTranscript(path)
            transcript.extend(b"partial")
            transcript.extend(b" evidence")
            self.assertEqual(path.read_bytes(), b"partial evidence")

    def test_serial_markers_in_one_read_preserve_tail(self):
        port = Mock()
        port.in_waiting = 12
        port.read.return_value = b"first second"
        transcript = bytearray()
        with patch('linux_hardware_trials.sys.stdout') as output:
            output.buffer = io.BytesIO()
            _, cursor = read_until(port, transcript, b"first", 1, "first")
            read_until(port, transcript, b"second", 1, "second", cursor)
        self.assertEqual(port.read.call_count, 1)

    def test_serial_marker_does_not_reuse_old_prompt(self):
        port = Mock()
        port.in_waiting = 8
        port.read.return_value = b"new done"
        transcript = bytearray(b"old done")
        with patch('linux_hardware_trials.sys.stdout') as output:
            output.buffer = io.BytesIO()
            _, cursor = read_until(port, transcript, b"done", 1, "fresh prompt")
        self.assertEqual(cursor, 16)
        self.assertEqual(port.read.call_count, 1)

    def test_kernel_panic_fails_without_waiting_for_login_timeout(self):
        transcript = bytearray(b'Kernel panic - not syncing: diagnostic')
        with self.assertRaisesRegex(RuntimeError, 'Kernel panic while waiting'):
            read_until(Mock(), transcript, b'login:', 180, 'login', 0)

    def test_colored_buildroot_prompt_is_recognized(self):
        transcript = bytearray(b'\x1b[01;32mroot@buildroot\x1b[00m:\x1b[01;34m~\x1b[00m# ')
        _, cursor = read_until(Mock(), transcript, ROOT_PROMPTS, 1, 'root shell', 0)
        self.assertEqual(cursor, len(transcript))

    def test_console_pacing_preserves_all_bytes_with_small_chunks(self):
        port = Mock()
        port.write.side_effect = len
        data = b'echo a command longer than the RX FIFO\n'
        with patch('linux_hardware_trials.time.sleep') as sleep:
            write_console(port, data)
        chunks = [call.args[0] for call in port.write.call_args_list]
        self.assertEqual(b''.join(chunks), data)
        self.assertTrue(all(len(chunk) <= 8 for chunk in chunks))
        self.assertEqual(sleep.call_count, len(chunks))

    def test_bad_zero_hash_stops_before_random_overwrite_or_delete(self):
        transcript = bytearray()
        sent = []
        def send(port, data):
            sent.append(data)
        def receive(port, received, *args, **kwargs):
            if sent[-1].startswith(b'echo UBER_COMMAND_RC='):
                received.extend(b'UBER_COMMAND_RC=0\n')
            elif sent[-1].startswith(b'echo UBER_ZERO_SIZE='):
                received.extend(b'UBER_ZERO_SIZE=16777216\n')
            elif sent[-1].startswith(b'echo UBER_ZERO_HASH='):
                received.extend(b'UBER_ZERO_HASH=' + b'0' * 64 + b'\n')
        with patch('linux_hardware_trials.write_console', side_effect=send), \
             patch('linux_hardware_trials.read_until', side_effect=receive):
            with self.assertRaisesRegex(RuntimeError, 'file preserved'):
                test_userspace(Mock(), transcript, 16)
        self.assertFalse(any(b'/dev/urandom' in data or b'rm -f' in data for data in sent))

    def test_failed_zero_file_is_captured_and_checked_on_host(self):
        contents = bytearray(1024 * 1024)
        contents[0x4fe0:0x5000] = bytes(range(32))
        digest = hashlib.sha256(contents).hexdigest()
        encoded = base64.b64encode(gzip.compress(contents))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial_uart.bin"
            transcript = PersistentTranscript(path)
            transcript.extend(b"UBER_ZERO_HASH=" + digest.encode() + b"\n")
            def receive(*args, **kwargs):
                transcript.extend(b"gzip -c /tmp/uberddr4_test.bin | base64\n" + encoded + b"\nroot@buildroot:~# ")
            with patch('linux_hardware_trials.write_console'), \
                 patch('linux_hardware_trials.read_until', side_effect=receive), \
                 patch('linux_hardware_trials.sys.stdout', new_callable=io.StringIO):
                capture_zero_failure(Mock(), transcript, 1)
            self.assertEqual((Path(directory) / "trial_uart_failed_file.bin").read_bytes(), contents)
            report = json.loads((Path(directory) / "trial_uart_failed_file.json").read_text())
            self.assertTrue(report["matches_reported_hash"])
            self.assertEqual(report["nonzero_bytes"], 31)

    def test_capture_error_does_not_replace_original_zero_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            transcript = PersistentTranscript(Path(directory) / "trial_uart.bin")
            with patch('linux_hardware_trials._test_userspace',
                       side_effect=RuntimeError("Zero file hash mismatch; file preserved")), \
                 patch('linux_hardware_trials.write_console', side_effect=RuntimeError("UART unavailable")), \
                 patch('linux_hardware_trials.sys.stdout', new_callable=io.StringIO):
                with self.assertRaisesRegex(RuntimeError, "Zero file hash mismatch"):
                    test_userspace(Mock(), transcript, 1)
            report = json.loads((Path(directory) / "trial_uart_failed_file.json").read_text())
            self.assertEqual(report["capture_error"], "UART unavailable")

    def test_bad_random_hash_stops_before_delete(self):
        transcript = bytearray(b"OpenSBI\nLinux version\n32-bit RISC-V Linux running on LiteX / VexRiscv-SMP.\n")
        sent = []
        values = dict(UBER_STATUS="0x000000d0", UBER_TRAIN_FAIL="0x0",
                      UBER_CORRECT="0x0", UBER_ERROR="0x0", UBER_BIST_STATUS="0x40",
                      UBER_CONFIG="0x40", UBER_VERSION="0x1", UBER_INIT_PROGRESS="0x80",
                      UBER_ZERO_SIZE="1048576", UBER_FILE_SIZE="1048576",
                      UBER_ZERO_HASH=hashlib.sha256(bytes(1048576)).hexdigest(),
                      UBER_HASH1="1"*64, UBER_HASH2="2"*64, UBER_COMMAND_RC="0")
        def send(port, data):
            sent.append(data)
        def receive(port, received, *args, **kwargs):
            command = sent[-1].decode()
            if command.startswith("echo UBERDDR4_SWTEST_BEGIN"):
                received.extend(b"UBERDDR4_SWTEST_BEGIN\n")
            for name, value in values.items():
                if command.startswith("echo " + name + "="):
                    received.extend((name + "=" + value + "\n").encode())
        with patch('linux_hardware_trials.write_console', side_effect=send), \
             patch('linux_hardware_trials.read_until', side_effect=receive):
            with self.assertRaisesRegex(RuntimeError, "RAM hashes are absent or unequal"):
                test_userspace(Mock(), transcript, 1)
        self.assertFalse(any(b"rm -f" in command for command in sent))

    def test_stale_payload_is_rejected_before_board_access(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('design.bit', 'program.tcl', 'xsdb', 'opensbi.bin'):
                (root / name).write_bytes(b'test fixture')
            boot = root / 'boot.json'
            boot.write_text(json.dumps({'rv32.dtb': '0x40ef0000', 'opensbi.bin': '0x40f00000'}))
            tree = self.reserved_tree()
            tree.get_node('/reserved-memory').remove_subnode('rv32-last-page@7ffff000')
            (root / 'rv32.dtb').write_bytes(tree.to_dtb(version=17))
            argv = ['trials', '--data-rate', '2400', '--bitstream', str(root / 'design.bit'),
                    '--boot-json', str(boot), '--xsdb', str(root / 'xsdb'),
                    '--program-tcl', str(root / 'program.tcl'),
                    '--output-dir', str(root / 'results')]
            with patch('sys.argv', argv), patch('linux_hardware_trials.select_port') as select, \
                 patch('linux_hardware_trials.upload_and_test') as upload:
                with self.assertRaisesRegex(RuntimeError, 'prepare_linux_payload.ps1'):
                    trials_main()
                select.assert_not_called()
                upload.assert_not_called()
            self.assertFalse((root / 'results').exists())

    def test_completed_trial_survives_later_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('design.bit', 'program.tcl', 'xsdb', 'opensbi.bin'):
                (root / name).write_bytes(b'test fixture')
            boot = root / 'boot.json'
            boot.write_text(json.dumps({'rv32.dtb': '0x40ef0000', 'opensbi.bin': '0x40f00000'}))
            (root / 'rv32.dtb').write_bytes(self.reserved_tree().to_dtb(version=17))
            output = root / 'results'
            argv = ['trials', '--trials', '2', '--data-rate', '2400',
                    '--bitstream', str(root / 'design.bit'), '--boot-json', str(boot),
                    '--xsdb', str(root / 'xsdb'), '--program-tcl', str(root / 'program.tcl'),
                    '--output-dir', str(output)]
            result = dict(bytes=100, status=0xd0, train_fail=0, correct=0,
                          error=0, bist_status=0x40, config=0x40, version=1,
                          init_progress=0x80, ram_sha256='a' * 64)
            with patch('sys.argv', argv), patch('linux_hardware_trials.select_port', return_value='TEST'), \
                 patch('linux_hardware_trials.upload_and_test', side_effect=[result, RuntimeError('interrupted')]), \
                 patch('linux_hardware_trials.sys.stdout', new_callable=io.StringIO):
                with self.assertRaisesRegex(RuntimeError, 'interrupted'):
                    trials_main()
            self.assertEqual(len((output / 'summary.tsv').read_text().splitlines()), 2)
            self.assertIn('PASS', (output / 'summary.tsv').read_text())
            self.assertEqual(json.loads((output / 'trial_02_failure.json').read_text())['trial'], 2)
            self.assertEqual(json.loads((output / 'manifest.json').read_text())['requested_trials'], 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
