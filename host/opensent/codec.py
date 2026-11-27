"""Pure-Python SAE J2716 SENT codec.

A line-by-line port of the C core in ``core/`` (same algorithm, same
results), so captures can be decoded on a PC and inside the sigrok/PulseView
protocol decoder. No dependencies.

The decoder consumes falling-edge timestamps (any unit; give ``clk_hz`` so
ticks can be reported in nanoseconds) and yields :class:`Frame` objects.
Each frame carries the boundaries of every pulse so tools can annotate it.

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Iterator, List, Optional, Sequence, Tuple

SYNC_TICKS = 56
NIBBLE_OFFSET = 12
MAX_DATA = 8
PAUSE_MIN, PAUSE_MAX = 12, 768

CRC_2010, CRC_LEGACY, CRC_STATUS, CRC_AUTO = 0, 1, 2, 3
CRC_NAMES = {CRC_2010: "2010", CRC_LEGACY: "LEGACY", CRC_STATUS: "STATUS", CRC_AUTO: "AUTO"}

OK, E_CRC, E_NIBBLE, E_LENGTH, E_SHORT, E_SYNC = range(6)
ERR_NAMES = ["OK", "CRC", "NIBBLE", "LENGTH", "SHORT", "SYNC"]

F_PAUSE, F_CAL, F_MARGIN, F_TICK = 0x01, 0x02, 0x04, 0x08

SLOW_NONE, SLOW_SHORT, SLOW_ENH12, SLOW_ENH16 = range(4)
SLOW_NAMES = ["OFF", "SHORT", "ENH12", "ENH16"]

# name -> (fc1 nibbles, fc2 nibbles, fc2 reversed)
FORMATS = {
    "12_12": (3, 3, True),
    "12_12N": (3, 3, False),
    "16_8": (4, 2, False),
    "12": (3, 0, False),
    "16": (4, 0, False),
    "RAW": (0, 0, False),
}

# --------------------------------------------------------------------- CRC


def _mul4(c: int) -> int:
    for _ in range(4):
        c <<= 1
        if c & 0x10:
            c ^= 0x1D
    return c & 0xF


def _mul6(c: int) -> int:
    for _ in range(6):
        c <<= 1
        if c & 0x40:
            c ^= 0x59
    return c & 0x3F


_T4 = [_mul4(i) for i in range(16)]
_T6 = [_mul6(i) for i in range(64)]


def crc4(mode: int, status: int, data: Sequence[int]) -> int:
    """4-bit SENT CRC, seed 0b0101, polynomial x^4+x^3+x^2+1."""
    c = 5
    if mode == CRC_LEGACY:
        for d in data:
            c = _T4[c] ^ (d & 0xF)
        return c
    if mode == CRC_STATUS:
        c = _T4[c ^ (status & 0xF)]
        for d in data:
            c = _T4[c ^ (d & 0xF)]
        return c
    for d in data:
        c = _T4[c] ^ (d & 0xF)
    return _T4[c]


def crc6(bits24: int) -> int:
    """6-bit CRC of an enhanced serial message, seed 0b010101, poly 0x59."""
    c = 0x15
    for i in (3, 2, 1, 0):
        c = _T6[c] ^ ((bits24 >> (6 * i)) & 0x3F)
    return _T6[c]


def _enh_crc_input(b2_12: int, b3_12: int) -> int:
    x = 0
    for i in range(11, -1, -1):
        x = (x << 2) | (((b2_12 >> i) & 1) << 1) | ((b3_12 >> i) & 1)
    return x


# ------------------------------------------------------------ fast channel


def unpack(fmt: str, d: Sequence[int]) -> Tuple[int, int]:
    n1, n2, rev = FORMATS[fmt]
    if n1 + n2 > len(d):
        return 0, 0
    a = 0
    for i in range(n1):
        a = (a << 4) | d[i]
    b = 0
    for i in range(n2):
        v = d[n1 + i]
        if rev:
            b |= v << (4 * i)
        else:
            b = (b << 4) | v
    return a, b


def pack(fmt: str, fc1: int, fc2: int) -> List[int]:
    n1, n2, rev = FORMATS[fmt]
    out = [(fc1 >> (4 * i)) & 0xF for i in range(n1 - 1, -1, -1)]
    for i in range(n2):
        sh = i if rev else n2 - 1 - i
        out.append((fc2 >> (4 * sh)) & 0xF)
    return out


# ------------------------------------------------------------ slow channel


def slow_bits(fmt: int, msg_id: int, value: int, idx: int, bad_crc: bool = False) -> Tuple[int, int, int]:
    """(bit3, bit2, frames_per_message) for frame ``idx`` of a serial message."""
    if fmt == SLOW_SHORT:
        nib = [msg_id & 0xF, (value >> 4) & 0xF, value & 0xF]
        c = crc4(CRC_2010, 0, nib) ^ (1 if bad_crc else 0)
        word = ((msg_id & 0xF) << 12) | ((value & 0xFF) << 4) | c
        return (1 if idx == 0 else 0), (word >> (15 - idx)) & 1, 16
    if fmt in (SLOW_ENH12, SLOW_ENH16):
        c16 = fmt == SLOW_ENH16
        hi4 = (msg_id & 0xF) if c16 else ((msg_id >> 4) & 0xF)
        lo4 = ((value >> 12) & 0xF) if c16 else (msg_id & 0xF)
        b3w = (int(c16) << 10) | (hi4 << 6) | (lo4 << 1)
        b2w = value & 0xFFF
        c = crc6(_enh_crc_input(b2w, b3w)) ^ (1 if bad_crc else 0)
        if idx < 6:
            return 1, (c >> (5 - idx)) & 1, 18
        k = 17 - idx
        return (b3w >> k) & 1, (b2w >> k) & 1, 18
    return 0, 0, 0


@dataclass
class SlowMessage:
    fmt: int
    id: int
    value: int
    ts: float = 0.0
    count: int = 1

    @property
    def fmt_name(self) -> str:
        return SLOW_NAMES[self.fmt]


class SlowDecoder:
    def __init__(self) -> None:
        self.sh2 = self.sh3 = self.nbits = 0
        self.table: dict = {}
        self.ok = self.crc_err = 0

    def reset(self, clear_table: bool = False) -> None:
        self.sh2 = self.sh3 = self.nbits = 0
        if clear_table:
            self.table.clear()
            self.ok = self.crc_err = 0

    def _store(self, fmt: int, msg_id: int, value: int, ts: float) -> SlowMessage:
        key = (fmt != SLOW_SHORT, msg_id)
        m = self.table.get(key)
        if m:
            m.fmt, m.value, m.ts = fmt, value, ts
            m.count += 1
        else:
            m = self.table[key] = SlowMessage(fmt, msg_id, value, ts)
        return m

    def feed(self, status: int, ts: float = 0.0) -> Optional[SlowMessage]:
        self.sh2 = ((self.sh2 << 1) | ((status >> 2) & 1)) & 0xFFFFFFFF
        self.sh3 = ((self.sh3 << 1) | ((status >> 3) & 1)) & 0xFFFFFFFF
        self.nbits = min(self.nbits + 1, 32)
        if self.nbits >= 16 and (self.sh3 & 0xFFFF) == 0x8000:
            w = self.sh2 & 0xFFFF
            mid, val, c = w >> 12, (w >> 4) & 0xFF, w & 0xF
            nib = [mid, val >> 4, val & 0xF]
            if c in (crc4(CRC_2010, 0, nib), crc4(CRC_LEGACY, 0, nib)):
                self.ok += 1
                return self._store(SLOW_SHORT, mid, val, ts)
            self.crc_err += 1
            return None
        if self.nbits >= 18 and (self.sh3 & 0x3F821) == 0x3F000:
            b3w, b2w = self.sh3 & 0xFFF, self.sh2 & 0xFFF
            c = (self.sh2 >> 12) & 0x3F
            if c != crc6(_enh_crc_input(b2w, b3w)):
                self.crc_err += 1
                return None
            c16 = (b3w >> 10) & 1
            hi4, lo4 = (b3w >> 6) & 0xF, (b3w >> 1) & 0xF
            self.ok += 1
            if c16:
                return self._store(SLOW_ENH16, hi4, (lo4 << 12) | b2w, ts)
            return self._store(SLOW_ENH12, (hi4 << 4) | lo4, b2w, ts)
        return None


# ----------------------------------------------------------------- decoder


@dataclass
class Frame:
    ts: float                     # start of the sync pulse (input time unit)
    err: int = OK
    status: int = 0
    data: List[int] = field(default_factory=list)
    crc: int = 0
    crc_mode: int = -1
    flags: int = 0
    pause_ticks: int = 0
    max_dev: float = 0.0          # worst nibble deviation, ticks
    tick_ns: float = 0.0
    fc1: int = 0
    fc2: int = 0
    pulses: List[Tuple[float, float]] = field(default_factory=list)  # sync, status, data.., crc, [pause]
    slow: Optional[SlowMessage] = None

    @property
    def ok(self) -> bool:
        return self.err == OK

    @property
    def err_name(self) -> str:
        return ERR_NAMES[self.err]

    @property
    def data_hex(self) -> str:
        return "".join("%X" % d for d in self.data)


@dataclass
class Stats:
    pulses: int = 0
    frames: int = 0
    ok: int = 0
    err: List[int] = field(default_factory=lambda: [0] * 6)
    pauses: int = 0
    cal_flags: int = 0
    margin_flags: int = 0
    resyncs: int = 0
    glitches: int = 0


class Decoder:
    """Streaming SENT decoder, same algorithm as core/sent_rx.c."""

    SYNC_TOL = 0.20
    NIB_MAX = 27.5
    NIB_MIN = 11.5
    MARGIN = 0.35
    LEARN_VOTES = 3
    CRC_LOCK_VOTES = 4
    SEG_MAX = MAX_DATA + 4

    def __init__(self, clk_hz: float = 1e9, tick_ns: float = 0.0, tol_pct: float = 25.0,
                 nibbles: int = 0, crc_mode: int = CRC_AUTO, fmt: str = "12_12",
                 no_pause: bool = False) -> None:
        self.clk_hz = float(clk_hz)
        self.tick_ns = float(tick_ns)
        self.tol = tol_pct / 100.0
        self.nibbles = nibbles
        self.crc_mode = crc_mode
        self.fmt = fmt
        self.no_pause = no_pause
        self.slow = SlowDecoder()
        self.reset()

    def reset(self) -> None:
        self.have_sync = False
        self.locked = False
        self.sync = 0.0
        self.prev_sync = 0.0
        self.sync_ts = 0.0
        self.seg: List[Tuple[float, float]] = []  # (length, start)
        self.learned_n = 0
        self.learn_votes = 0
        self.crc_lock = -1
        self.crc_votes = 0
        self.carry = 0.0
        self.carry_start: Optional[float] = None
        self.st = Stats()
        self.slow.reset(True)
        self._last_edge: Optional[float] = None

    # helpers ------------------------------------------------------------
    def _sync_in_range(self, clocks: float) -> bool:
        tick = clocks * 1e9 / self.clk_hz / SYNC_TICKS
        if self.tick_ns > 0:
            return self.tick_ns * (1 - self.tol) <= tick <= self.tick_ns * (1 + self.tol)
        return 1500.0 <= tick <= 100000.0

    def _known_n(self) -> int:
        if self.nibbles:
            return self.nibbles
        return self.learned_n if self.learn_votes >= self.LEARN_VOTES else 0

    def _open(self, clocks: float, start: float) -> None:
        self.have_sync = True
        self.sync = clocks
        self.sync_ts = start
        self.seg = []

    def _lose_lock(self) -> None:
        if self.locked:
            self.st.resyncs += 1
        self.locked = False
        self.slow.reset(False)

    def _crc_ok(self, status: int, d: List[int], c: int) -> Tuple[bool, int]:
        if self.crc_mode != CRC_AUTO:
            return crc4(self.crc_mode, status, d) == c, self.crc_mode
        if self.crc_lock >= 0 and self.crc_votes >= self.CRC_LOCK_VOTES:
            return crc4(self.crc_lock, status, d) == c, self.crc_lock
        for m in (CRC_2010, CRC_LEGACY, CRC_STATUS):
            if crc4(m, status, d) == c:
                return True, m
        return False, -1

    def _nibbles(self, p: List[float], f: Frame) -> int:
        scale = SYNC_TICKS / self.sync
        worst, v, err = 0.0, [], OK
        for x in p:
            r = x * scale
            if r < self.NIB_MIN:
                return E_SHORT
            if r > self.NIB_MAX:
                err = E_NIBBLE
                v.append(0)
                continue
            q = int(r + 0.5)
            worst = max(worst, abs(r - q))
            v.append(q - NIBBLE_OFFSET)
        f.status, f.data, f.crc = v[0], v[1:-1], v[-1]
        f.max_dev = worst
        f.flags = F_MARGIN if worst > self.MARGIN else 0
        if err != OK:
            return err
        ok, mode = self._crc_ok(f.status, f.data, f.crc)
        f.crc_mode = mode
        return OK if ok else E_CRC

    def _decode(self, seg: List[Tuple[float, float]]) -> Frame:
        lens = [s[0] for s in seg]
        cnt = len(lens)
        scale = SYNC_TICKS / self.sync
        n_cfg = self._known_n()
        f = Frame(ts=self.sync_ts, tick_ns=self.sync * 1e9 / self.clk_hz / SYNC_TICKS)
        f.pulses = [(self.sync_ts, self.sync_ts + self.sync)] + [(s[1], s[1] + s[0]) for s in seg]
        last = lens[-1] * scale if cnt else 0.0
        pause, err = False, OK
        if n_cfg:
            if cnt == n_cfg + 3 and not self.no_pause:
                pause = True
            elif cnt != n_cfg + 2:
                err = E_LENGTH
        elif last > self.NIB_MAX and not self.no_pause:
            pause = True
        body = cnt - (1 if pause else 0)
        if err == OK and not 3 <= body <= MAX_DATA + 2:
            err = E_LENGTH
        if err == OK:
            err = self._nibbles(lens[:body], f)
            if not n_cfg and not pause and not self.no_pause and cnt >= 4:
                # last pulse is either the CRC or a nibble-sized pause
                alt = Frame(ts=f.ts, tick_ns=f.tick_ns, pulses=f.pulses)
                if self._nibbles(lens[:cnt - 1], alt) == OK:
                    if err != OK or abs(len(alt.data) - 6) <= abs(len(f.data) - 6):
                        f, pause, err = alt, True, OK
        if pause:
            f.flags |= F_PAUSE
            f.pause_ticks = min(int(last + 0.5), 65535)
            if last < PAUSE_MIN - 0.5 and err == OK:
                err = E_SHORT
        if self.prev_sync and abs(self.sync - self.prev_sync) * 64 > self.prev_sync:
            f.flags |= F_CAL
        if self.tick_ns > 0 and abs(f.tick_ns - self.tick_ns) > 0.2 * self.tick_ns:
            f.flags |= F_TICK
        f.err = err
        return f

    def _commit(self, f: Frame) -> Frame:
        st = self.st
        st.frames += 1
        st.err[f.err] += 1
        if f.flags & F_PAUSE:
            st.pauses += 1
        if f.flags & F_CAL:
            st.cal_flags += 1
        if f.flags & F_MARGIN:
            st.margin_flags += 1
        if f.err != OK:
            self._lose_lock()
            return f
        st.ok += 1
        self.locked = True
        self.prev_sync = self.sync
        f.fc1, f.fc2 = unpack(self.fmt, f.data)
        if not self.nibbles:
            if len(f.data) == self.learned_n:
                self.learn_votes = min(self.learn_votes + 1, 255)
            elif self.learn_votes <= 1:
                self.learned_n, self.learn_votes = len(f.data), 1
            else:
                self.learn_votes -= 1
        if self.crc_mode == CRC_AUTO and self.crc_votes < self.CRC_LOCK_VOTES:
            if f.crc_mode == self.crc_lock:
                self.crc_votes += 1
            else:
                self.crc_lock, self.crc_votes = f.crc_mode, 1
        f.slow = self.slow.feed(f.status, f.ts)
        return f

    def _feed_one(self, clocks: float, start: float) -> List[Frame]:
        if not self.have_sync:
            if self._sync_in_range(clocks):
                self._open(clocks, start)
            return []
        r = clocks * SYNC_TICKS / self.sync
        sync_like = abs(r - SYNC_TICKS) <= self.SYNC_TOL * SYNC_TICKS
        if not self.seg and r > self.NIB_MAX:
            if self.locked:
                self.st.pauses += 1
            if self._sync_in_range(clocks) or sync_like:
                self._open(clocks, start)
            else:
                self.have_sync = False
            return []
        if sync_like:
            f = self._commit(self._decode(self.seg))
            self._open(clocks, start)
            return [f]
        if r < self.NIB_MIN and not self.locked:
            pending = self.seg + [(clocks, start)]
            self.have_sync = False
            out: List[Frame] = []
            for c, s in pending:
                out += self._feed_one(c, s)
            return out
        if r > self.NIB_MAX and not self.locked:
            if 3 <= len(self.seg) < self.SEG_MAX:
                f = self._decode(self.seg + [(clocks, start)])
                if f.err == OK:
                    self._commit(f)
                    self.have_sync = False
                    return [f]
            if self._sync_in_range(clocks):
                self._open(clocks, start)
            else:
                self.have_sync = False
            return []
        n_known = self._known_n()
        if self.locked and n_known and len(self.seg) >= n_known + (2 if self.no_pause else 3):
            f = self._decode(self.seg)
            if f.err == OK:
                self._commit(f)
                self.st.err[E_SYNC] += 1
                self._lose_lock()
            else:
                f.err = E_LENGTH
                self._commit(f)
            self.have_sync = False
            return [f]
        if len(self.seg) >= self.SEG_MAX:
            f = Frame(ts=self.sync_ts, err=E_LENGTH)
            self._commit(f)
            self.have_sync = False
            return [f]
        self.seg.append((clocks, start))
        return []

    def feed_interval(self, clocks: float, start: float = 0.0) -> List[Frame]:
        """One falling-to-falling interval starting at ``start``."""
        self.st.pulses += 1
        if self.locked:
            glitch = self.sync * 11 / SYNC_TICKS
        elif self.have_sync:
            glitch = self.sync * 2 / SYNC_TICKS
        else:
            glitch = self.clk_hz / 1e6
        if clocks < glitch:
            if self.carry_start is None:
                self.carry_start = start
            self.carry += clocks
            self.st.glitches += 1
            return []
        if self.carry_start is not None:
            start = self.carry_start
        clocks += self.carry
        self.carry, self.carry_start = 0.0, None
        return self._feed_one(clocks, start)

    def feed_edge(self, t: float) -> List[Frame]:
        """Timestamp of one falling edge."""
        prev, self._last_edge = self._last_edge, t
        if prev is None:
            return []
        return self.feed_interval(t - prev, prev)

    def decode_edges(self, edges: Iterable[float]) -> Iterator[Frame]:
        for t in edges:
            yield from self.feed_edge(t)


# ----------------------------------------------------------------- encoder


@dataclass
class EncoderConfig:
    nibbles: int = 6
    crc_mode: int = CRC_2010
    fmt: str = "12_12"
    status_bits: int = 0
    pause_ticks: int = 0          # 0 = none
    const_frame_ticks: int = 0    # constant frame length (pause adjusts)
    slow_fmt: int = SLOW_NONE
    slow_msgs: List[Tuple[int, int]] = field(default_factory=list)


class Encoder:
    """Frame -> list of pulse lengths in ticks (mirror of core/sent_tx.c)."""

    def __init__(self, cfg: Optional[EncoderConfig] = None) -> None:
        self.cfg = cfg or EncoderConfig()
        self.slow_msg = 0
        self.slow_idx = 0

    def frame(self, fc1: int = 0, fc2: int = 0, raw: Optional[Sequence[int]] = None) -> List[int]:
        c = self.cfg
        d = list(raw) if raw is not None else pack(c.fmt, fc1, fc2)
        d = (d + [0] * c.nibbles)[:c.nibbles]
        status = c.status_bits & 3
        if c.slow_fmt != SLOW_NONE and c.slow_msgs:
            mid, val = c.slow_msgs[self.slow_msg % len(c.slow_msgs)]
            b3, b2, n = slow_bits(c.slow_fmt, mid, val, self.slow_idx)
            status |= (b3 << 3) | (b2 << 2)
            self.slow_idx += 1
            if self.slow_idx >= n:
                self.slow_idx = 0
                self.slow_msg = (self.slow_msg + 1) % len(c.slow_msgs)
        crc = crc4(c.crc_mode, status, d)
        ticks = [SYNC_TICKS, NIBBLE_OFFSET + status] + [NIBBLE_OFFSET + x for x in d] + [NIBBLE_OFFSET + crc]
        if c.pause_ticks:
            ticks.append(c.pause_ticks)
        elif c.const_frame_ticks:
            ticks.append(max(PAUSE_MIN, c.const_frame_ticks - sum(ticks)))
        return ticks


def edges_from_ticks(pulses: Iterable[int], tick: float, t0: float = 0.0) -> Iterator[float]:
    """Falling-edge timestamps for a stream of pulse lengths (in ticks)."""
    t = t0
    yield t
    for p in pulses:
        t += p * tick
        yield t


# ------------------------------------------------------------- file input


def read_edges_csv(path: str, falling: bool = True) -> Tuple[List[float], List[float]]:
    """Read a logic-analyzer CSV export (``time[s],level`` with a header,
    as written by Saleae Logic / rusEFI tests). Returns (falling edge
    times, rising edge times) in seconds."""
    fall: List[float] = []
    rise: List[float] = []
    prev = None
    with open(path) as fp:
        for line in fp:
            parts = line.replace(" ", "").split(",")
            if len(parts) < 2:
                continue
            try:
                t, v = float(parts[0]), int(float(parts[1]))
            except ValueError:
                continue
            if prev is not None and v != prev:
                (fall if v == 0 else rise).append(t)
            prev = v
    return (fall, rise) if falling else (rise, fall)


def read_edges_vcd(path: str, signal: Optional[str] = None) -> Tuple[List[float], List[float]]:
    """Minimal single-bit VCD reader. Returns (falling, rising) in seconds."""
    scale = 1e-9
    ident = None
    fall: List[float] = []
    rise: List[float] = []
    t = 0
    prev = None
    units = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9, "ps": 1e-12, "fs": 1e-15}
    with open(path) as fp:
        text = fp.read()
    head, _, body = text.partition("$enddefinitions")
    import re
    m = re.search(r"\$timescale\s+(\d+)\s*(\w+)\s+\$end", head)
    if m:
        scale = int(m.group(1)) * units[m.group(2)]
    for vm in re.finditer(r"\$var\s+\w+\s+1\s+(\S+)\s+(\S+)", head):
        if ident is None or (signal and vm.group(2) == signal):
            ident = vm.group(1)
    for tok in body.split():
        if tok.startswith("#"):
            t = int(tok[1:])
        elif len(tok) >= 2 and tok[0] in "01" and tok[1:] == ident:
            v = int(tok[0])
            if prev is not None and v != prev:
                (fall if v == 0 else rise).append(t * scale)
            prev = v
    return fall, rise


def write_vcd(path: str, levels: Iterable[Tuple[float, int]], timescale_ns: int = 10, name: str = "sent") -> None:
    """Write (time[s], level) transitions as a 1-bit VCD file."""
    with open(path, "w") as fp:
        fp.write("$timescale %dns $end\n$scope module opensent $end\n$var wire 1 ! %s $end\n"
                 "$upscope $end\n$enddefinitions $end\n" % (timescale_ns, name))
        last = None
        for t, v in levels:
            tt = int(round(t * 1e9 / timescale_ns))
            if tt != last:
                fp.write("#%d\n" % tt)
                last = tt
            fp.write("%d!\n" % v)
