"""Sensor health report: decode a SENT capture and write a self-contained
HTML page with timing statistics, signal plots, error breakdown and the
serial (slow channel) messages.

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import html
import json
import math
import statistics
from typing import Dict, List, Optional, Sequence, Tuple

from . import codec

# Common J2716 enhanced-serial message IDs (Appendix tables; check the
# sensor datasheet - manufacturers also use their own IDs)
SLOW_IDS = {
    0x01: "diagnostic status code",
    0x03: "channel 1/2 sensor type",
    0x05: "manufacturer code",
    0x06: "SENT standard revision",
    0x07: "fast channel 1 characteristic X1",
    0x08: "fast channel 1 characteristic X2",
    0x09: "fast channel 1 characteristic Y1",
    0x0A: "fast channel 1 characteristic Y2",
    0x23: "internal temperature",
    0x29: "sensor ID #1",
    0x2A: "sensor ID #2",
    0x2B: "sensor ID #3",
    0x2C: "sensor ID #4",
}
for _i in range(8):
    SLOW_IDS[0x90 + _i] = "OEM code %d" % (_i + 1)


def _nibble_devs(f: codec.Frame) -> List[float]:
    sync = f.pulses[0][1] - f.pulses[0][0]
    out = []
    for a, b in f.pulses[1:2 + len(f.data) + 1]:
        r = (b - a) * codec.SYNC_TICKS / sync
        out.append(r - round(r))
    return out


def analyze(fall: Sequence[float], rise: Sequence[float] = (), nominal_tick_us: float = 3.0,
            **decoder_kw) -> Dict:
    dec = codec.Decoder(clk_hz=1.0, **decoder_kw)
    frames = list(dec.decode_edges(fall))
    good = [f for f in frames if f.ok]
    st = dec.st
    a: Dict = {"frames": st.frames, "ok": st.ok, "errors": dict(zip(codec.ERR_NAMES[1:], st.err[1:])),
               "pauses": st.pauses, "cal": st.cal_flags, "margin": st.margin_flags, "glitches": st.glitches,
               "resyncs": st.resyncs, "nibbles": dec._known_n(), "crc": codec.CRC_NAMES.get(dec.crc_lock, "-"),
               "duration_s": (fall[-1] - fall[0]) if len(fall) > 1 else 0.0}
    a["ok_pct"] = 100.0 * st.ok / st.frames if st.frames else 0.0
    if good:
        ticks = [f.tick_ns / 1000.0 for f in good]
        a["tick_mean"] = statistics.fmean(ticks)
        a["tick_std_ppm"] = (statistics.pstdev(ticks) / a["tick_mean"] * 1e6) if len(ticks) > 1 else 0.0
        a["tick_min"], a["tick_max"] = min(ticks), max(ticks)
        a["tick_dev_pct"] = (a["tick_mean"] / nominal_tick_us - 1.0) * 100.0
        periods = [b.ts - x.ts for x, b in zip(good, good[1:]) if 0 < b.ts - x.ts < 0.1]
        a["frame_rate"] = 1.0 / statistics.fmean(periods) if periods else 0.0
        a["frame_us_mean"] = statistics.fmean(periods) * 1e6 if periods else 0.0
        a["frame_us_min"] = min(periods) * 1e6 if periods else 0.0
        a["frame_us_max"] = max(periods) * 1e6 if periods else 0.0
        devs = [d for f in good for d in _nibble_devs(f)]
        a["dev_abs_max"] = max(abs(d) for d in devs)
        a["dev_rms"] = math.sqrt(sum(d * d for d in devs) / len(devs))
        a["dev_hist"] = _histogram(devs, -0.5, 0.5, 20)
        a["pause_ticks"] = sorted({f.pause_ticks for f in good if f.pause_ticks})
        for k in ("fc1", "fc2"):
            v = [getattr(f, k) for f in good]
            a[k] = {"min": min(v), "max": max(v), "mean": statistics.fmean(v),
                    "std": statistics.pstdev(v) if len(v) > 1 else 0.0}
        t0 = good[0].ts
        step = max(1, len(good) // 1200)
        a["series"] = [[round((f.ts - t0) * 1e3, 4), f.fc1, f.fc2, round(f.tick_ns / 1000.0, 5)]
                       for f in good[::step]]
    lows = []
    if rise:
        j = 0
        for t in fall:
            while j < len(rise) and rise[j] <= t:
                j += 1
            if j < len(rise):
                lows.append(rise[j] - t)
    if lows and a.get("tick_mean"):
        lt = [x * 1e6 / a["tick_mean"] for x in lows if x * 1e6 < 40 * a["tick_mean"]]
        if lt:
            a["low_ticks_mean"] = statistics.fmean(lt)
            a["low_ticks_min"] = min(lt)
            a["low_ticks_max"] = max(lt)
    a["slow"] = [{"fmt": m.fmt_name, "id": m.id, "value": m.value, "count": m.count,
                  "meaning": SLOW_IDS.get(m.id, "") if m.fmt != codec.SLOW_SHORT else ""}
                 for m in sorted(dec.slow.table.values(), key=lambda m: (m.fmt, m.id))]
    a["slow_ok"], a["slow_err"] = dec.slow.ok, dec.slow.crc_err
    a["verdict"] = _verdict(a)
    return a


def _histogram(v: Sequence[float], lo: float, hi: float, n: int) -> List[int]:
    h = [0] * n
    for x in v:
        i = int((x - lo) / (hi - lo) * n)
        h[min(max(i, 0), n - 1)] += 1
    return h


def _verdict(a: Dict) -> List[Tuple[str, str]]:
    """(level, text) findings: ok / warn / bad"""
    out: List[Tuple[str, str]] = []
    if not a["frames"]:
        return [("bad", "No SENT frames found. Check polarity (--invert), the capture and the tick range.")]
    out.append(("ok" if a["ok_pct"] >= 99.9 else "warn" if a["ok_pct"] >= 98 else "bad",
                "%.2f %% of %d frames decoded without error." % (a["ok_pct"], a["frames"])))
    if "tick_dev_pct" in a:
        d = a["tick_dev_pct"]
        out.append(("ok" if abs(d) <= 10 else "warn" if abs(d) <= 20 else "bad",
                    "Tick %.4f us, %+.2f %% from nominal (J2716 transmitter limit +/-20 %%); frame-to-frame "
                    "jitter %.0f ppm." % (a["tick_mean"], d, a["tick_std_ppm"])))
        m = a["dev_abs_max"]
        out.append(("ok" if m < 0.25 else "warn" if m < 0.4 else "bad",
                    "Worst nibble timing error %.3f tick (decoding fails at 0.5); RMS %.3f tick." % (m, a["dev_rms"])))
    if a["cal"]:
        out.append(("warn", "%d frames violate the successive-calibration check (sync pulses differ > 1/64)."
                    % a["cal"]))
    if a["glitches"]:
        out.append(("warn", "%d glitches (sub-nibble pulses) were filtered: check the wiring, edge shaping "
                    "and the logic-analyzer threshold." % a["glitches"]))
    if "low_ticks_mean" in a:
        lt = a["low_ticks_min"]
        out.append(("ok" if lt >= 4 else "warn", "Low time %.2f..%.2f ticks (J2716: at least 4)."
                    % (a["low_ticks_min"], a["low_ticks_max"])))
    if a["slow_err"]:
        out.append(("warn", "%d serial messages with a CRC error." % a["slow_err"]))
    return out


# ------------------------------------------------------------------ HTML

_CSS = """
:root{--bg:#f6f7f9;--surface:#fff;--ink:#1b1f24;--muted:#5d6673;--grid:#e3e6ea;--line:#dde1e6;
--s1:#257de0;--s2:#f47230;--ok:#1a7f37;--warn:#9a5b00;--bad:#c62828;--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
@media (prefers-color-scheme:dark){:root{--bg:#111316;--surface:#191919;--ink:#efefef;--muted:#9c9c9c;--grid:#2c2f33;
--line:#2c2f33;--s1:#338ced;--s2:#e76624;--ok:#3fb950;--warn:#e3a008;--bad:#ff6b6b}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1100px;margin:0 auto;padding:22px 18px 40px}
h1{font-size:22px;margin:0 0 2px}h2{font-size:13px;text-transform:uppercase;letter-spacing:.6px;color:var(--muted);margin:0 0 10px}
.sub{color:var(--muted);margin-bottom:18px}
section{background:var(--surface);border:1px solid var(--line);border-radius:10px;padding:14px 16px;margin:14px 0}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
.tile{border:1px solid var(--line);border-radius:8px;padding:8px 11px}.tile .k{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.4px}
.tile .v{font:600 22px var(--mono)}.tile .u{font-size:12px;color:var(--muted);margin-left:3px}
ul.find{list-style:none;padding:0;margin:0}ul.find li{padding:5px 0;display:flex;gap:9px;align-items:baseline}
.badge{font:600 11px var(--mono);padding:1px 7px;border-radius:9px;border:1px solid currentColor;min-width:44px;text-align:center}
.b-ok{color:var(--ok)}.b-warn{color:var(--warn)}.b-bad{color:var(--bad)}
table{border-collapse:collapse;width:100%;font:12px var(--mono)}td,th{text-align:left;padding:4px 8px;border-bottom:1px solid var(--line)}
th{color:var(--muted);font-weight:500}td.n{text-align:right}
.chart{position:relative}.chart svg{width:100%;height:auto;display:block}
.legend{display:flex;gap:16px;font-size:12px;color:var(--muted);margin-bottom:4px}.legend i{display:inline-block;width:16px;height:3px;border-radius:2px;vertical-align:middle;margin-right:5px}
.tip{position:absolute;pointer-events:none;background:var(--surface);border:1px solid var(--line);border-radius:6px;padding:5px 8px;font:12px var(--mono);display:none;white-space:nowrap;box-shadow:0 2px 8px rgba(0,0,0,.12)}
.two{display:grid;grid-template-columns:1fr 1fr;gap:14px}@media(max-width:800px){.two{grid-template-columns:1fr}}
footer{color:var(--muted);font-size:12px;margin-top:20px}
"""

_JS = r"""
const D = JSON.parse(document.getElementById('data').textContent);
const NS = 'http://www.w3.org/2000/svg';
function el(n, a, p) { const e = document.createElementNS(NS, n); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; }
function niceTicks(lo, hi, n) { const span = hi - lo || 1, step0 = span / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 2.5, 5, 10].map(m => m * mag).find(s => span / s <= n) || 10 * mag; const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10)); return out; }
function lineChart(host, xs, ys, opt) {
  const W = opt.w || 1040, H = opt.h || 260, L = opt.w ? 62 : 56, R = opt.w ? 14 : 70, T = 10, B = 28, pw = W - L - R, ph = H - T - B;
  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, role: 'img', 'aria-label': opt.label }, host);
  const x0 = xs[0], x1 = xs[xs.length - 1] || 1;
  let y0 = Infinity, y1 = -Infinity; ys.forEach(s => s.v.forEach(v => { y0 = Math.min(y0, v); y1 = Math.max(y1, v); }));
  if (opt.yMin !== undefined) y0 = Math.min(y0, opt.yMin); if (opt.yMax !== undefined) y1 = Math.max(y1, opt.yMax);
  if (y1 - y0 < 1e-9) { y0 -= 1; y1 += 1; } const pad = (y1 - y0) * 0.06; y0 -= pad; y1 += pad;
  const X = v => L + (v - x0) / (x1 - x0 || 1) * pw, Y = v => T + ph - (v - y0) / (y1 - y0) * ph;
  for (const t of niceTicks(y0, y1, 5)) { el('line', { x1: L, x2: L + pw, y1: Y(t), y2: Y(t), stroke: 'var(--grid)' }, svg);
    const tx = el('text', { x: L - 6, y: Y(t) + 4, 'text-anchor': 'end', fill: 'var(--muted)', 'font-size': 11, 'font-family': 'var(--mono)' }, svg); tx.textContent = opt.fmtY ? opt.fmtY(t) : t; }
  for (const t of niceTicks(x0, x1, 8)) { const tx = el('text', { x: X(t), y: H - 8, 'text-anchor': 'middle', fill: 'var(--muted)', 'font-size': 11, 'font-family': 'var(--mono)' }, svg); tx.textContent = t + (opt.xUnit || ''); }
  ys.forEach(s => { let d = ''; s.v.forEach((v, i) => d += (i ? 'L' : 'M') + X(xs[i]).toFixed(1) + ' ' + Y(v).toFixed(1));
    el('path', { d, fill: 'none', stroke: s.c, 'stroke-width': 2, 'stroke-linejoin': 'round' }, svg);
    if (ys.length > 1) { const tx = el('text', { x: L + pw + 6, y: Y(s.v[s.v.length - 1]) + 4, fill: 'var(--ink)', 'font-size': 12 }, svg); tx.textContent = s.n; } });
  const cross = el('line', { y1: T, y2: T + ph, stroke: 'var(--muted)', 'stroke-dasharray': '3 3', visibility: 'hidden' }, svg);
  const dots = ys.map(s => el('circle', { r: 4, fill: s.c, stroke: 'var(--surface)', 'stroke-width': 2, visibility: 'hidden' }, svg));
  const tip = document.createElement('div'); tip.className = 'tip'; host.appendChild(tip);
  const hit = el('rect', { x: L, y: T, width: pw, height: ph, fill: 'transparent' }, svg);
  hit.addEventListener('mousemove', ev => { const r = svg.getBoundingClientRect(), sx = (ev.clientX - r.left) / r.width * W;
    const xv = x0 + (sx - L) / pw * (x1 - x0); let lo = 0, hi = xs.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; xs[m] < xv ? lo = m : hi = m; }
    const i = Math.abs(xs[lo] - xv) < Math.abs(xs[hi] - xv) ? lo : hi; const px = X(xs[i]);
    cross.setAttribute('x1', px); cross.setAttribute('x2', px); cross.setAttribute('visibility', 'visible');
    dots.forEach((d, k) => { d.setAttribute('cx', px); d.setAttribute('cy', Y(ys[k].v[i])); d.setAttribute('visibility', 'visible'); });
    tip.innerHTML = `${xs[i]}${opt.xUnit || ''}<br>` + ys.map(s => `${s.n}: <b>${opt.fmtY ? opt.fmtY(s.v[i]) : s.v[i]}</b>`).join('<br>');
    tip.style.display = 'block'; tip.style.left = Math.min(px / W * r.width + 12, r.width - 140) + 'px'; tip.style.top = '10px'; });
  hit.addEventListener('mouseleave', () => { tip.style.display = 'none'; cross.setAttribute('visibility', 'hidden'); dots.forEach(d => d.setAttribute('visibility', 'hidden')); });
}
function barChart(host, labels, vals, opt) {
  const W = 520, H = 230, L = 46, R = 24, T = 10, B = 30, pw = W - L - R, ph = H - T - B, n = vals.length, bw = pw / n;
  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, role: 'img', 'aria-label': opt.label }, host);
  const ym = Math.max(...vals, 1); const Y = v => T + ph - v / ym * ph;
  for (const t of niceTicks(0, ym, 4)) { el('line', { x1: L, x2: L + pw, y1: Y(t), y2: Y(t), stroke: 'var(--grid)' }, svg);
    const tx = el('text', { x: L - 6, y: Y(t) + 4, 'text-anchor': 'end', fill: 'var(--muted)', 'font-size': 11, 'font-family': 'var(--mono)' }, svg); tx.textContent = t; }
  const tip = document.createElement('div'); tip.className = 'tip'; host.appendChild(tip);
  vals.forEach((v, i) => { const x = L + i * bw + 1, h = Math.max(0, T + ph - Y(v));
    if (h > 0) el('path', { d: `M${x} ${T + ph}V${T + ph - h + Math.min(4, h)}q0 -${Math.min(4, h)} ${Math.min(4, h)} -${Math.min(4, h)}H${x + bw - 2 - Math.min(4, h)}q${Math.min(4, h)} 0 ${Math.min(4, h)} ${Math.min(4, h)}V${T + ph}Z`, fill: 'var(--s1)' }, svg);
    const hit = el('rect', { x: L + i * bw, y: T, width: bw, height: ph, fill: 'transparent' }, svg);
    hit.addEventListener('mousemove', ev => { const r = svg.getBoundingClientRect(); tip.innerHTML = `${labels[i]}<br><b>${v}</b> nibbles`; tip.style.display = 'block';
      tip.style.left = Math.min((x / W) * r.width + 10, r.width - 150) + 'px'; tip.style.top = '8px'; });
    hit.addEventListener('mouseleave', () => tip.style.display = 'none'); });
  [0, n / 2, n].forEach((k, j) => { const tx = el('text', { x: L + k * bw, y: H - 10, 'text-anchor': 'middle', fill: 'var(--muted)', 'font-size': 11, 'font-family': 'var(--mono)' }, svg); tx.textContent = ['-0.5', '0', '+0.5'][j]; });
}
if (D.series && D.series.length) {
  const xs = D.series.map(r => r[0]);
  lineChart(document.getElementById('c-fc'), xs, [{ n: 'FC1', v: D.series.map(r => r[1]), c: 'var(--s1)' }, { n: 'FC2', v: D.series.map(r => r[2]), c: 'var(--s2)' }], { xUnit: ' ms', label: 'Fast channels over time' });
  lineChart(document.getElementById('c-tick'), xs, [{ n: 'tick', v: D.series.map(r => r[3]), c: 'var(--s1)' }], { xUnit: ' ms', w: 520, h: 230, fmtY: v => (+v).toFixed(4), label: 'Tick per frame (us)' });
}
if (D.dev_hist) { const n = D.dev_hist.length; barChart(document.getElementById('c-dev'), D.dev_hist.map((_, i) => `${(-0.5 + i / n).toFixed(2)}..${(-0.5 + (i + 1) / n).toFixed(2)} tick`), D.dev_hist, { label: 'Nibble timing error histogram' }); }
"""


def _tile(k: str, v: str, unit: str = "") -> str:
    return '<div class="tile"><div class="k">%s</div><div class="v">%s<span class="u">%s</span></div></div>' % (
        html.escape(k), html.escape(v), html.escape(unit))


def render(a: Dict, title: str, source: str) -> str:
    e = html.escape
    tiles = [
        _tile("Frames decoded", "%d" % a["frames"]),
        _tile("Without error", "%.2f" % a["ok_pct"], "%"),
        _tile("Tick", "%.4f" % a.get("tick_mean", 0), "us"),
        _tile("Tick vs 3 us", "%+.2f" % a.get("tick_dev_pct", 0), "%"),
        _tile("Frame rate", "%.0f" % a.get("frame_rate", 0), "/s"),
        _tile("Data nibbles / CRC", "%s / %s" % (a["nibbles"] or "?", a["crc"])),
    ]
    finds = "".join('<li><span class="badge b-%s">%s</span><span>%s</span></li>' % (lvl, lvl.upper(), e(t))
                    for lvl, t in a["verdict"])
    rows = [("Capture length", "%.3f s" % a["duration_s"]),
            ("Frame period", "%.1f us (min %.1f, max %.1f)" % (a.get("frame_us_mean", 0), a.get("frame_us_min", 0),
                                                              a.get("frame_us_max", 0))),
            ("Tick range", "%.4f .. %.4f us, jitter %.0f ppm" % (a.get("tick_min", 0), a.get("tick_max", 0),
                                                                a.get("tick_std_ppm", 0))),
            ("Nibble timing error", "max %.3f tick, RMS %.3f tick" % (a.get("dev_abs_max", 0), a.get("dev_rms", 0))),
            ("Pause pulse", ", ".join("%d" % p for p in a.get("pause_ticks", [])[:6]) + (" ticks" if a.get("pause_ticks") else "none")),
            ("Low time", "%.2f .. %.2f ticks" % (a["low_ticks_min"], a["low_ticks_max"]) if "low_ticks_min" in a
             else "not captured (falling edges only)"),
            ("Errors", ", ".join("%s %d" % kv for kv in a["errors"].items())),
            ("Successive-calibration flags", "%d" % a["cal"]),
            ("Glitches filtered / resyncs", "%d / %d" % (a["glitches"], a["resyncs"]))]
    for k in ("fc1", "fc2"):
        if k in a:
            s = a[k]
            rows.append((k.upper(), "min %d  max %d  mean %.1f  std %.2f" % (s["min"], s["max"], s["mean"], s["std"])))
    stats = "".join("<tr><th>%s</th><td>%s</td></tr>" % (e(k), e(v)) for k, v in rows)
    slow = "".join('<tr><td>%s</td><td>0x%02X</td><td class="n">%d</td><td>0x%X</td><td class="n">%d</td><td>%s</td></tr>' % (
        m["fmt"], m["id"], m["value"], m["value"], m["count"], e(m["meaning"])) for m in a["slow"]) or \
        '<tr><td colspan="6">no serial messages</td></tr>'
    data = {k: a.get(k) for k in ("series", "dev_hist")}
    return """<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>%s</title><style>%s</style></head><body><main>
<h1>%s</h1><div class="sub">SENT (SAE J2716) sensor report &middot; %s &middot; generated by OpenSENT</div>
<section><div class="tiles">%s</div></section>
<section><h2>Findings</h2><ul class="find">%s</ul></section>
<section><h2>Fast channels</h2><div class="legend"><span><i style="background:var(--s1)"></i>FC1</span><span><i style="background:var(--s2)"></i>FC2</span></div><div class="chart" id="c-fc"></div></section>
<div class="two">
<section><h2>Tick per frame (us)</h2><div class="chart" id="c-tick"></div></section>
<section><h2>Nibble timing error (ticks)</h2><div class="chart" id="c-dev"></div></section>
</div>
<section><h2>Statistics</h2><table>%s</table></section>
<section><h2>Serial messages (slow channel) &middot; %d ok, %d CRC errors</h2><table><tr><th>Format</th><th>ID</th><th>Value</th><th>Hex</th><th>Count</th><th>Typical meaning (J2716)</th></tr>%s</table></section>
<footer>Typical meanings follow the common J2716 ID assignments; always check the sensor datasheet. Charts show every %s frame.</footer>
</main><script type="application/json" id="data">%s</script><script>%s</script></body></html>""" % (
        e(title), _CSS, e(title), e(source), "".join(tiles), finds, stats, a["slow_ok"], a["slow_err"], slow,
        "decoded" if len(a.get("series") or []) >= a["ok"] else "Nth (decimated)",
        json.dumps(data).replace("</", "<\\/"), _JS)


def report_from_file(path: str, out: str, title: Optional[str] = None, invert: bool = False, **kw) -> Dict:
    if path.lower().endswith(".vcd"):
        fall, rise = codec.read_edges_vcd(path)
    else:
        fall, rise = codec.read_edges_csv(path)
    if invert:
        fall, rise = rise, fall
    a = analyze(fall, rise, **kw)
    with open(out, "w") as fp:
        fp.write(render(a, title or path.rsplit("/", 1)[-1], path.rsplit("/", 1)[-1]))
    return a
