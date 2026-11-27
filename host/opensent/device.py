"""Talk to an OpenSENT instrument (USB CDC) or its host simulator (pty).

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterator, List, Optional

import serial
import serial.tools.list_ports

USB_PRODUCT = "OpenSENT"


class DeviceError(RuntimeError):
    pass


def find_ports() -> List[str]:
    """Serial ports whose USB product string is OpenSENT."""
    out = []
    for p in serial.tools.list_ports.comports():
        if (p.product or "") == USB_PRODUCT or (p.manufacturer or "") == USB_PRODUCT:
            out.append(p.device)
    return sorted(out)


@dataclass
class StreamFrame:
    ch: int
    ts_us: float
    err: str
    status: int
    data: str
    crc: int
    fc1: int
    fc2: int
    tick_ns: float
    flags: int
    pause: int

    @property
    def ok(self) -> bool:
        return self.err == "OK"

    @classmethod
    def parse(cls, line: str) -> "StreamFrame":
        # "#F ch,ts_us,err,status,data,crc,fc1,fc2,tick_ns,flags,pause"
        p = line[3:].split(",")
        return cls(int(p[0]), float(p[1]), p[2], int(p[3], 16), p[4], int(p[5], 16), int(p[6]), int(p[7]),
                   float(p[8]), int(p[9]), int(p[10]))


def parse_kv(s: str) -> Dict[str, object]:
    """'a=1 b=2.5 c=X' -> {'a': 1, 'b': 2.5, 'c': 'X'}"""
    out: Dict[str, object] = {}
    for tok in s.split():
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        try:
            out[k] = int(v)
        except ValueError:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


class Device:
    def __init__(self, port: Optional[str] = None, timeout: float = 2.0) -> None:
        if port is None:
            ports = find_ports()
            if not ports:
                raise DeviceError("no OpenSENT device found (use --port)")
            port = ports[0]
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=timeout)
        self.ser.reset_input_buffer()
        self.frames: List[StreamFrame] = []  # async frames seen while waiting for replies
        self.on_frame: Optional[Callable[[StreamFrame], None]] = None

    def close(self) -> None:
        self.ser.close()

    def __enter__(self) -> "Device":
        return self

    def __exit__(self, *a) -> None:
        self.close()

    # ------------------------------------------------------------- raw
    def _readline(self) -> str:
        raw = self.ser.readline()
        if not raw:
            raise DeviceError("timeout waiting for reply")
        return raw.decode(errors="replace").strip()

    def _async(self, line: str) -> None:
        if line.startswith("#F "):
            f = StreamFrame.parse(line)
            if self.on_frame:
                self.on_frame(f)
            else:
                self.frames.append(f)

    def query(self, cmd: str) -> str:
        """Send one line, return its reply line (async records are kept aside)."""
        self.ser.write((cmd.strip() + "\n").encode())
        while True:
            line = self._readline()
            if line.startswith("#"):
                self._async(line)
                continue
            if line.startswith("ERR"):
                raise DeviceError("%s -> %s" % (cmd, line))
            if ";ERR" in line:
                raise DeviceError("%s -> %s" % (cmd, line))
            return line

    def cmd(self, cmd: str) -> None:
        r = self.query(cmd)
        if any(x != "OK" for x in r.split(";")):
            raise DeviceError("%s -> %s" % (cmd, r))

    # ---------------------------------------------------------- typed
    def idn(self) -> str:
        return self.query("*IDN?")

    def reset(self) -> None:
        self.query("*RST")

    def rx_stat(self, ch: int = 0) -> Dict[str, object]:
        return parse_kv(self.query("RX%d:STAT?" % ch))

    def rx_last(self, ch: int = 0) -> Dict[str, object]:
        r = self.query("RX%d:LAST?" % ch)
        return {} if r == "NONE" else parse_kv(r)

    def rx_slow(self, ch: int = 0) -> List[tuple]:
        r = self.query("RX%d:SLOW?" % ch)
        if r == "NONE":
            return []
        out = []
        for item in r.split(","):
            fmt, rest = item.split(":")
            i, v = rest.split("=")
            out.append((fmt, int(i), int(v)))
        return out

    def tx_info(self) -> Dict[str, object]:
        return parse_kv(self.query("TX:INFO?"))

    def stream(self, mask: int, seconds: float, every: int = 1) -> List[StreamFrame]:
        """Collect streamed frames for a while."""
        got: List[StreamFrame] = []
        self.on_frame = got.append
        try:
            self.cmd("STREAM %d,%d" % (mask, every))
            end = time.monotonic() + seconds
            self.ser.timeout = 0.05
            while time.monotonic() < end:
                raw = self.ser.readline()
                if raw:
                    line = raw.decode(errors="replace").strip()
                    if line.startswith("#"):
                        self._async(line)
        finally:
            self.ser.timeout = 2.0
            self.query("STREAM 0")
            # drain the tail of the stream
            self.ser.timeout = 0.2
            while True:
                raw = self.ser.readline()
                if not raw:
                    break
                line = raw.decode(errors="replace").strip()
                if line.startswith("#"):
                    self._async(line)
            self.ser.timeout = 2.0
            self.on_frame = None
        return got

    def iter_stream(self, mask: int, every: int = 1) -> Iterator[StreamFrame]:
        """Endless stream iterator (Ctrl-C to stop)."""
        self.cmd("STREAM %d,%d" % (mask, every))
        self.ser.timeout = 0.5
        try:
            while True:
                raw = self.ser.readline()
                if raw:
                    line = raw.decode(errors="replace").strip()
                    if line.startswith("#F "):
                        yield StreamFrame.parse(line)
        finally:
            self.ser.timeout = 2.0
            self.ser.write(b"STREAM 0\n")
            time.sleep(0.1)
            self.ser.reset_input_buffer()
