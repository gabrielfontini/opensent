"""Tests for the pure-Python codec, cross-checked against the C core."""
import os
import random
import subprocess

import pytest

from opensent import codec

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DATA = os.path.join(ROOT, "tests", "data")
REPLAY = os.path.join(ROOT, "build-native", "sent_replay")
CAPTURES = sorted(f for f in os.listdir(DATA) if f.endswith(".csv")) if os.path.isdir(DATA) else []

TABLE = [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5]


def test_crc4_matches_j2716_table():
    rnd = random.Random(1)
    for _ in range(500):
        d = [rnd.randrange(16) for _ in range(6)]
        c = 5
        for x in d:
            c = x ^ TABLE[c]
        assert codec.crc4(codec.CRC_LEGACY, 0, d) == c
        assert codec.crc4(codec.CRC_2010, 0, d) == TABLE[c]


def test_status_variant_equals_2010_when_status_is_zero():
    rnd = random.Random(2)
    for _ in range(200):
        d = [rnd.randrange(16) for _ in range(6)]
        assert codec.crc4(codec.CRC_STATUS, 0, d) == codec.crc4(codec.CRC_2010, 0, d)


def test_crc6_x6_term():
    assert codec.crc6(0) ^ codec.crc6(1) == 25


@pytest.mark.parametrize("fmt", list(codec.FORMATS))
def test_pack_unpack(fmt):
    n1, n2, _ = codec.FORMATS[fmt]
    rnd = random.Random(3)
    for _ in range(100):
        a = rnd.randrange(1 << (4 * n1)) if n1 else 0
        b = rnd.randrange(1 << (4 * n2)) if n2 else 0
        assert codec.unpack(fmt, codec.pack(fmt, a, b)) == (a, b)


def _roundtrip(cfg, frames=300, tick=3e-6, jitter=0.0, seed=4, nibbles=0):
    rnd = random.Random(seed)
    enc = codec.Encoder(cfg)
    pulses, sent = [], []
    for _ in range(frames):
        fc1, fc2 = rnd.randrange(4096), rnd.randrange(4096)
        pulses += enc.frame(fc1, fc2)
        sent.append((fc1, fc2))
    edges = [e + rnd.uniform(-jitter, jitter) * tick for e in codec.edges_from_ticks(pulses, tick)]
    dec = codec.Decoder(clk_hz=1.0, nibbles=nibbles)
    return list(dec.decode_edges(edges)), sent, dec


@pytest.mark.parametrize("pause", [0, 12, 40, 56, 300, 768])
def test_roundtrip_pauses(pause):
    frames, sent, _ = _roundtrip(codec.EncoderConfig(pause_ticks=pause), jitter=0.1)
    good = [f for f in frames if f.ok]
    assert len(good) >= len(sent) - 3
    vals = [(f.fc1, f.fc2) for f in good]
    i = sent.index(vals[0])
    assert vals == sent[i:i + len(vals)]  # every frame, in order, correct


@pytest.mark.parametrize("slow,msgs", [
    (codec.SLOW_SHORT, [(1, 0), (5, 255), (15, 0x5A)]),
    (codec.SLOW_ENH12, [(0x01, 0), (0x23, 786), (0xFF, 4095)]),
    (codec.SLOW_ENH16, [(1, 0xFFFF), (9, 0x1234)]),
])
def test_slow_roundtrip(slow, msgs):
    cfg = codec.EncoderConfig(slow_fmt=slow, slow_msgs=msgs)
    _, _, dec = _roundtrip(cfg, frames=18 * len(msgs) * 3)
    assert {(m.id, m.value) for m in dec.slow.table.values()} == set(msgs)
    assert dec.slow.crc_err == 0


@pytest.mark.parametrize("cap", CAPTURES)
def test_real_capture_decodes(cap):
    fall, _ = codec.read_edges_csv(os.path.join(DATA, cap))
    dec = codec.Decoder(clk_hz=1.0)
    list(dec.decode_edges(fall))
    assert dec.st.frames > 50
    assert dec.st.ok / dec.st.frames > 0.95


@pytest.mark.skipif(not os.path.exists(REPLAY), reason="build-native/sent_replay not built")
@pytest.mark.parametrize("cap", CAPTURES)
def test_python_matches_c_core(cap):
    """Same algorithm in both languages: identical frame lists."""
    path = os.path.join(DATA, cap)
    rows = subprocess.run([REPLAY, path], capture_output=True, text=True, check=True).stdout.splitlines()[1:]
    c_frames = [tuple(r.split(",")[1:5]) for r in rows]
    fall, _ = codec.read_edges_csv(path)
    dec = codec.Decoder(clk_hz=1.0)
    py_frames = [(f.err_name, "%X" % f.status, f.data_hex, "%X" % f.crc) for f in dec.decode_edges(fall)]
    ok_c = [x for x in c_frames if x[0] == "OK"]
    ok_py = [x for x in py_frames if x[0] == "OK"]
    assert ok_py == ok_c
    assert len(py_frames) == len(c_frames)


@pytest.mark.parametrize("cap", CAPTURES[:2])
def test_report_renders(cap, tmp_path):
    from opensent import report
    out = tmp_path / "r.html"
    a = report.report_from_file(os.path.join(DATA, cap), str(out))
    html = out.read_text()
    assert a["frames"] > 50 and "tick_mean" in a
    assert "<svg" not in html  # charts are drawn client-side from the embedded JSON
    assert '"series"' in html and "Findings" in html
