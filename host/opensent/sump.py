"""Minimal SUMP / Openbench Logic Sniffer client for the OpenSENT logic
analyzer port (PulseView/sigrok use their own "ols" driver; this is for
scripts and the self test).

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import struct
import time
from typing import Dict, List, Optional

import serial

BASE_CLOCK = 100_000_000


class Sump:
    def __init__(self, port: str, timeout: float = 2.0) -> None:
        self.ser = serial.Serial(port, 115200, timeout=timeout)
        self.reset()

    def close(self) -> None:
        self.ser.close()

    def _short(self, cmd: int) -> None:
        self.ser.write(bytes([cmd]))

    def _long(self, cmd: int, value: int) -> None:
        self.ser.write(bytes([cmd]) + struct.pack("<I", value & 0xFFFFFFFF))

    def reset(self) -> None:
        self.ser.write(b"\x00" * 5)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def ident(self) -> bytes:
        self._short(0x02)
        return self.ser.read(4)

    def metadata(self) -> Dict[str, object]:
        self._short(0x04)
        out: Dict[str, object] = {}
        names = {0x20: "probes", 0x21: "sample_memory", 0x23: "max_rate", 0x24: "protocol"}
        while True:
            k = self.ser.read(1)
            if not k or k[0] == 0:
                return out
            key = k[0]
            if key >> 5 == 0:
                s = b""
                while True:
                    c = self.ser.read(1)
                    if not c or c == b"\x00":
                        break
                    s += c
                out[{1: "name", 2: "version"}.get(key, hex(key))] = s.decode()
            elif key >> 5 == 1:
                out[names.get(key, hex(key))] = struct.unpack(">I", self.ser.read(4))[0]
            else:
                out[hex(key)] = self.ser.read(1)[0]

    def capture(self, rate: int, samples: int, pre: int = 0, mask: int = 0, value: int = 0,
                groups: int = 0b11) -> List[int]:
        """Returns `samples` 16-bit samples, oldest first. With a trigger,
        the trigger sample is at index `pre`."""
        samples = (samples + 3) // 4 * 4
        pre = (pre + 3) // 4 * 4
        self._long(0xC0, mask)
        self._long(0xC1, value)
        self._long(0xC2, 1 << 27)
        self._long(0x80, BASE_CLOCK // rate - 1)
        rc, dc = samples // 4 - 1, (samples - pre) // 4 - 1
        self._long(0x81, (rc & 0xFFFF) | ((dc & 0xFFFF) << 16))
        self._long(0x82, (~(groups << 2)) & 0x3C)
        self._short(0x01)
        per = bin(groups & 3).count("1")
        raw = self.ser.read(samples * per)
        if len(raw) != samples * per:
            raise TimeoutError("got %d of %d bytes" % (len(raw), samples * per))
        out = []
        for i in range(samples):
            b = raw[i * per:(i + 1) * per]
            if groups == 0b11:
                out.append(b[0] | b[1] << 8)
            elif groups == 0b01:
                out.append(b[0])
            else:
                out.append(b[0] << 8)
        return out[::-1]  # the device sends the newest sample first


def runs(bits: List[int]) -> List[int]:
    r, c = [], 1
    for a, b in zip(bits, bits[1:]):
        if a == b:
            c += 1
        else:
            r.append(c)
            c = 1
    return r[1:]
