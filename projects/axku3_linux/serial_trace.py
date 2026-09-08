"""Instrument LiteX's Serial Firmware Loader (SFL) without changing its frames.

Keep the last 24 transport events and the largest host reply-to-send gap.
Fixed timeout mode avoids Windows serial reconfiguration during each transfer;
it still rejects short writes and missing acknowledgements. Used by the runner,
not by the interactive console, and never executed on the RISC-V CPU.
"""

from collections import deque
import json
import time

from litex.tools.litex_term import LiteXTerm, SFLUploadError


class TracedLiteXTerm(LiteXTerm):
    """Add a small persistent diagnostic trail to the upstream serial loader."""
    def __init__(self, *args, trace_path, stable_timeouts=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.trace_path = trace_path
        self.events = deque(maxlen=24)
        self.tx_count = 0
        self.last_reply = time.monotonic()
        self.max_host_gap = 0.0
        self.stable_timeouts = stable_timeouts
        self.timeout_reply_recoveries = 0
        self.recovered_timeout_replies = 0

    def save_trace(self):
        self.trace_path.write_text(json.dumps(dict(
            tx_count=self.tx_count, max_host_gap=self.max_host_gap,
            timeout_reply_recoveries=self.timeout_reply_recoveries,
            recovered_timeout_replies=self.recovered_timeout_replies,
            events=list(self.events)), indent=2))

    def write_sfl_data(self, data, timeout=1.0):
        started = time.monotonic()
        gap = started - self.last_reply
        self.max_host_gap = max(self.max_host_gap, gap)
        self.tx_count += 1
        self.events.append(dict(event='tx', time=started, frame=self.tx_count,
                                bytes=len(data), header=data[:8].hex(), host_gap=gap))
        if self.stable_timeouts:
            written = self.port.write(data)
            if written != len(data):
                raise SFLUploadError('Short serial write with fixed timeout')
            return written
        return super().write_sfl_data(data, timeout)

    def read_sfl_reply(self, timeout=1.0):
        started = time.monotonic()
        try:
            if self.stable_timeouts:
                reply = self.port.read(1)
                if not reply:
                    raise SFLUploadError('No serial reply with fixed timeout')
            else:
                reply = super().read_sfl_reply(timeout)
        except Exception as exc:
            self.events.append(dict(event='read_error', time=time.monotonic(), error=str(exc)))
            self.save_trace()
            raise
        self.last_reply = time.monotonic()
        self.events.append(dict(event='rx', time=self.last_reply, reply=reply.hex(),
                                wait=self.last_reply-started))
        if reply != b'K':
            self.save_trace()
        return reply

    def receive_upload_response(self, timeout=1.0):
        # The BIOS emits E after 250 ms even with no frame in progress. A host
        # scheduling/terminal stall can queue this idle reply ahead of the next
        # frame's K. With exactly one outstanding frame, require that frame's
        # positive ACK within the original deadline; E alone is never success.
        # Keep upstream behavior for pipelined transfers and all CRC/command
        # errors. Whole-image DDR CRC readback remains mandatory in the runner.
        if not self.stable_timeouts or self.outstanding != 1:
            return super().receive_upload_response(timeout)
        deadline = time.monotonic() + timeout
        timeouts = 0
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SFLUploadError('No frame acknowledgement after BIOS timeout replies', reply=b'E')
            try:
                accepted = super().receive_upload_response(remaining)
            except SFLUploadError as error:
                if error.reply != b'E':
                    raise
                timeouts += 1
                continue
            if timeouts:
                self.timeout_reply_recoveries += 1
                self.recovered_timeout_replies += timeouts
                self.events.append(dict(event='timeout_replies_then_ack',
                                        time=time.monotonic(), count=timeouts))
                self.save_trace()
            return accepted
