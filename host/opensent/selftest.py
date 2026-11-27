"""End-to-end instrument self test.

The transmitter drives the TX pin and the instrument decodes the *real pin*
through its monitor channel (RX2). With ``jumper=True`` (wire TX->RX0) the
independent input-capture receiver RX0 is checked too. Every check uses
counter deltas, so it runs on hardware and on the simulator alike.

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from .device import Device


@dataclass
class Result:
    name: str
    ok: bool
    detail: str


@dataclass
class Report:
    results: List[Result] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(r.ok for r in self.results)

    def add(self, name: str, ok: bool, detail: str) -> Result:
        r = Result(name, ok, detail)
        self.results.append(r)
        return r


ERR_KEYS = ("crc", "nibble", "length", "short", "sync")


def _delta(a: dict, b: dict) -> dict:
    return {k: b[k] - a[k] for k in b if isinstance(b.get(k), int) and isinstance(a.get(k), int)}


class SelfTest:
    def __init__(self, dev: Device, jumper: bool = False, log: Callable[[str], None] = print,
                 quick: bool = False) -> None:
        self.dev = dev
        self.chans = [2, 0] if jumper else [2]
        self.log = log
        self.quick = quick
        self.rep = Report()

    # ------------------------------------------------------------ helpers
    def _setup(self, *cmds: str) -> None:
        d = self.dev
        d.reset()
        for ch in self.chans:
            d.cmd("RX%d:STATE ON" % ch)
        for c in cmds:
            d.cmd(c)
        d.cmd("TX:STATE ON")

    def _window(self, seconds: float) -> List[dict]:
        """settle, then counter deltas over `seconds` for every channel"""
        time.sleep(0.15)
        a = [self.dev.rx_stat(ch) for ch in self.chans]
        time.sleep(seconds)
        b = [self.dev.rx_stat(ch) for ch in self.chans]
        return [dict(_delta(x, y), tick_ns=y["tick_ns"], n=y["n"], crc_mode=y["crc_mode"], lock=y["lock"])
                for x, y in zip(a, b)]

    def _check(self, name: str, cond: bool, detail: str) -> bool:
        r = self.rep.add(name, cond, detail)
        self.log("  %s  %-34s %s" % ("PASS" if r.ok else "FAIL", name, detail))
        return cond

    def _clean(self, name: str, d: dict, min_frames: int, tick_us: Optional[float] = None,
               extra: str = "") -> bool:
        errs = sum(d.get(k, 0) for k in ERR_KEYS)
        ok = d["frames"] >= min_frames and errs == 0 and d["ok"] == d["frames"]
        det = "frames=%d ok=%d errs=%d" % (d["frames"], d["ok"], errs)
        if tick_us:
            dev = abs(d["tick_ns"] / (tick_us * 1000.0) - 1.0)
            ok = ok and dev < 0.002
            det += " tick=%.1fns (%.3f%%)" % (d["tick_ns"], dev * 100)
        return self._check(name, ok, det + extra)

    # -------------------------------------------------------------- tests
    def test_counter(self) -> None:
        """Every frame decoded, in order, none lost: FC1 counts 0..4095."""
        self._setup("TX:WAVE 1,COUNT,0,4095", "TX:WAVE 2,SINE,0,4095,0.25")
        time.sleep(0.2)
        for ch in self.chans:
            frames = self.dev.stream(1 << ch, 0.6 if self.quick else 1.5)
            good = [f for f in frames if f.ok]
            gaps = sum(1 for a, b in zip(good, good[1:]) if (a.fc1 + 1) % 4096 != b.fc1)
            bad = len(frames) - len(good)
            self._check("RX%d sequence (no lost frames)" % ch, len(good) > 500 and gaps == 0 and bad == 0,
                        "frames=%d bad=%d gaps=%d" % (len(frames), bad, gaps))

    def test_ticks(self) -> None:
        ticks = [3.0, 10.0, 90.0] if self.quick else [3.0, 4.5, 6.0, 10.0, 25.0, 50.0, 90.0]
        for t in ticks:
            self._setup("TX:TICK %g" % t, "TX:WAVE 1,RAMP,0,4095,0.2", "TX:WAVE 2,TRI,0,4095,0.3")
            secs = max(0.4, 60 * 200 * t * 1e-6)  # >= ~60 frames
            for ch, d in zip(self.chans, self._window(secs)):
                self._clean("RX%d tick %g us" % (ch, t), d, 40, t)

    def test_clock_error(self) -> None:
        """Sensor clock off by +/-18 %: still decoded (J2716 allows +/-20 %)."""
        for t in (3.0 * 1.18, 3.0 * 0.82):
            self._setup("TX:TICK %g" % t, "TX:WAVE 1,SINE,0,4095,0.1")
            for ch, d in zip(self.chans, self._window(0.4)):
                self._clean("RX%d sensor clock %+d %%" % (ch, round((t / 3 - 1) * 100)), d, 500, t)

    def test_crc_modes(self) -> None:
        for mode in ("2010", "LEGACY", "STATUS"):
            # status bits != 0: with a zero status nibble the STATUS variant equals 2010
            self._setup("TX:CRC %s" % mode, "TX:STATUS 1", "TX:WAVE 1,RAMP,0,4095,0.1", "TX:WAVE 2,COUNT,0,4095")
            for ch, d in zip(self.chans, self._window(0.4)):
                d = dict(d)
                if str(d["crc_mode"]) != mode:
                    d["ok"] = -1
                self._clean("RX%d CRC %s auto-detect" % (ch, mode), d, 500, extra=" detected=%s" % d["crc_mode"])

    def test_formats(self) -> None:
        # varying data: with a constant signal and no RX:NIB configured, an
        # N-nibble frame and an (N-1)-nibble frame + short pause can both pass
        # the CRC; the decoder then prefers the J2716-typical length
        for nib, fmt in ((3, "12"), (4, "16"), (8, "16_8")):
            self._setup("TX:FMT %s" % fmt, "TX:NIB %d" % nib, "TX:WAVE 1,RAMP,0,4095,0.05", "TX:WAVE 2,COUNT,0,255")
            for ch, d in zip(self.chans, self._window(0.3)):
                d = dict(d)
                if d["n"] != nib:
                    d["ok"] = -1
                self._clean("RX%d %d data nibbles (learned %s)" % (ch, nib, d["n"]), d, 300)

    def test_pauses(self) -> None:
        # constant data on purpose: a 12-tick pause is then ambiguous with a 7th nibble
        cases = [("12 (nibble-like)", "TX:PAUSE 12"), ("56 (sync-like)", "TX:PAUSE 56"), ("300", "TX:PAUSE 300"),
                 ("768", "TX:PAUSE 768"), ("const 282", "TX:PAUSE CONST,282")]
        if self.quick:
            cases = cases[1:3] + cases[4:]
        for name, cmd in cases:
            self._setup(cmd)
            for ch, d in zip(self.chans, self._window(0.4)):
                d = dict(d)
                if d["pauses"] < d["frames"] - 1 or d["n"] != 6:
                    d["ok"] = -1
                self._clean("RX%d pause %s" % (ch, name), d, 100, extra=" pauses=%d n=%d" % (d["pauses"], d["n"]))

    def test_slow(self) -> None:
        sets = {
            "SHORT": [(1, 0x00), (3, 0xFF), (7, 0x5A), (15, 0x81)],
            "ENH12": [(0x01, 0x000), (0x23, 0x312), (0x90, 0xA5A), (0xFF, 0xFFF)],
            "ENH16": [(1, 0xFFFF), (2, 0x1234), (15, 0x8001)],
        }
        for fmt, msgs in sets.items():
            cmds = ["TX:SLOW %s" % fmt] + ["TX:SLOW:ADD %d,%d" % m for m in msgs]
            self._setup(*cmds, "TX:WAVE 1,SINE,0,4095,0.1")
            n_frames = (16 if fmt == "SHORT" else 18) * len(msgs) * 3
            time.sleep(n_frames * 300e-6 + 0.2)
            for ch in self.chans:
                got = {(i, v) for f, i, v in self.dev.rx_slow(ch) if f == fmt}
                want = set(msgs)
                st = self.dev.rx_stat(ch)
                self._check("RX%d slow %s" % (ch, fmt), got == want and st["slow_err"] == 0,
                            "msgs=%d/%d slow_ok=%d slow_err=%d" % (len(got & want), len(want), st["slow_ok"],
                                                                     st["slow_err"]))

    def test_faults(self) -> None:
        cases = [("CRC", "crc", 1), ("NIBBLE", "nibble", 1), ("SHORT", "length", 1), ("SYNC", "sync", 1),
                 ("CAL", "cal", 2), ("SCRC", "slow_err", 1)]
        n = 8
        for fault, key, per in cases:
            extra = ["TX:SLOW ENH12", "TX:SLOW:ADD 1,100", "TX:SLOW:ADD 2,200"] if fault == "SCRC" else []
            self._setup(*extra, "TX:WAVE 1,RAMP,0,4095,0.1")
            time.sleep(0.2)
            for ch in self.chans:
                self.dev.cmd("RX%d:NIB 6" % ch)
            time.sleep(0.15)
            a = [self.dev.rx_stat(ch) for ch in self.chans]
            every = 40 if fault == "SCRC" else 10
            self.dev.cmd("TX:FAULT %s,%d,%d" % (fault, n, every))
            time.sleep(n * every * 300e-6 + 0.3)
            b = [self.dev.rx_stat(ch) for ch in self.chans]
            inj = self.dev.query("TX:FAULT?")
            for ch, x, y in zip(self.chans, a, b):
                d = _delta(x, y)
                others = sum(d.get(k, 0) for k in ERR_KEYS if k != key)
                got = d.get(key, 0)
                ok = got == n * per and (others == 0 or fault in ("SCRC", "CAL"))
                if fault == "CAL":
                    ok = n <= got <= n * per and sum(d.get(k, 0) for k in ERR_KEYS) == 0
                self._check("RX%d fault %s detected" % (ch, fault), ok,
                            "%s=%d (expected %d) other_errs=%d [%s]" % (key, got, n * per, others, inj))

    def test_rx_tick_config(self) -> None:
        """Receiver locked to a nominal tick rejects a 40 % off sensor."""
        self._setup("TX:TICK 4.2", "TX:WAVE 1,RAMP,0,4095,0.1")
        for ch in self.chans:
            self.dev.cmd("RX%d:TICK 3" % ch)
        for ch, d in zip(self.chans, self._window(0.3)):
            self._check("RX%d rejects tick outside +/-25 %%" % ch, d["ok"] == 0, "ok=%d frames=%d" % (d["ok"], d["frames"]))

    def test_bridge(self) -> None:
        """SENT -> analog bridge: decoded FC1 drives DAC1, read back on chip."""
        self._setup("TX:FC1 0")
        self.dev.cmd("BRIDGE1:MAP RX2,FC1,0,4095,0.5,3.0")
        self.dev.cmd("BRIDGE1:FAIL 0.25,100")
        self.dev.cmd("BRIDGE1:STATE ON")
        for val in (0, 1024, 2048, 3071, 4095):
            self.dev.cmd("TX:FC1 %d" % val)
            time.sleep(0.1)
            cmd_v, state, meas = self.dev.query("BRIDGE1:OUT?").split(",")
            want = 0.5 + 2.5 * val / 4095.0
            m = float(meas)
            ok = state == "OK" and abs(float(cmd_v) - want) < 0.002 and (m < 0 or abs(m - want) < 0.02)
            self._check("bridge FC1=%d -> %.3f V" % (val, want), ok,
                        "commanded=%s measured=%s state=%s" % (cmd_v, meas, state))
        self.dev.cmd("TX:STATE OFF")  # sensor lost -> fail-safe level after the timeout
        time.sleep(0.3)
        cmd_v, state, meas = self.dev.query("BRIDGE1:OUT?").split(",")
        m = float(meas)
        self._check("bridge fail-safe on sensor loss", state == "FAIL" and abs(float(cmd_v) - 0.25) < 0.002
                    and (m < 0 or abs(m - 0.25) < 0.02), "commanded=%s measured=%s state=%s" % (cmd_v, meas, state))

    def test_capture(self) -> None:
        """Raw two-edge capture of the pin, decoded on the PC."""
        from . import codec
        self._setup("TX:WAVE 1,COUNT,0,4095", "TX:SLOW ENH12", "TX:SLOW:ADD 0x23,786")
        time.sleep(0.1)
        for ch in self.chans:
            level0, clk, times = self.dev.capture(ch, 4000)
            fall = times[0::2] if level0 else times[1::2]
            rise = times[1::2] if level0 else times[0::2]
            lows = [r - f for f, r in zip(fall, rise)]
            frames = list(codec.Decoder(clk_hz=1.0, nibbles=6).decode_edges(fall))
            good = [f for f in frames if f.ok]
            gaps = sum(1 for a, b in zip(good, good[1:]) if (a.fc1 + 1) % 4096 != b.fc1)
            low_ticks = sorted(lows)[len(lows) // 2] / 3e-6 if lows else 0
            self._check("RX%d capture -> PC decode" % ch, len(good) >= len(frames) - 1 and len(good) > 150
                        and gaps == 0 and abs(low_ticks - 5) < 0.2,
                        "edges=%d frames=%d ok=%d gaps=%d low=%.2f ticks" % (len(times), len(frames), len(good),
                                                                          gaps, low_ticks))

    def test_pulltest(self) -> None:
        """Real RX0/RX1 input path without wires: internal pull-up/down bit-bang."""
        self.dev.reset()
        for ch in (0, 1):
            self.dev.cmd("RX%d:STATE ON" % ch)
            self.dev.cmd("RX%d:NIB 6" % ch)
            got = []
            self.dev.on_frame = got.append
            try:
                self.dev.cmd("STREAM %d,1" % (1 << ch))
                self.dev.query("RX%d:PULLTEST 300" % ch)
                time.sleep(0.2)
                self.dev.query("STREAM 0")
            finally:
                self.dev.on_frame = None
            st = self.dev.rx_stat(ch)
            good = [f for f in got if f.ok]
            seq_ok = all(f.fc1 == i + (good[0].fc1 if good else 0) for i, f in enumerate(good))
            ok = len(good) >= 298 and seq_ok and all(f.fc2 == 0x5A5 for f in good) and \
                abs(st["tick_ns"] / 5000.0 - 1) < 0.002 and st["frames"] == st["ok"]
            self._check("RX%d capture path (pull bit-bang)" % ch, ok,
                        "frames=%d ok=%d in-order=%s tick=%.1fns" % (st["frames"], st["ok"], seq_ok, st["tick_ns"]))

    def run(self) -> Report:
        idn = self.dev.idn()
        self.log("OpenSENT self test on %s (%s), channels %s" % (self.dev.port, idn.split(",")[1],
                                                                ",".join("RX%d" % c for c in self.chans)))
        for t in (self.test_counter, self.test_ticks, self.test_clock_error, self.test_crc_modes,
                  self.test_formats, self.test_pauses, self.test_slow, self.test_faults, self.test_rx_tick_config, self.test_bridge, self.test_capture, self.test_pulltest):
            self.log("%s: %s" % (t.__name__[5:], (t.__doc__ or "").strip().splitlines()[0] if t.__doc__ else ""))
            t()
        self.dev.reset()
        passed = sum(r.ok for r in self.rep.results)
        for r in self.rep.results:
            if not r.ok:
                self.log("  failed: %s (%s)" % (r.name, r.detail))
        self.log("%s: %d/%d checks passed" % ("PASS" if self.rep.ok else "FAIL", passed, len(self.rep.results)))
        return self.rep
