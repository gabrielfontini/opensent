"""opensent - command line tool for the OpenSENT SENT (SAE J2716) toolkit.

  opensent list                         find instruments
  opensent q "TX:TICK?" ...             raw commands
  opensent tx --tick 3 --fc1 1000 --fc2 3000 --on
  opensent monitor --ch 0               live decoded frames
  opensent stat --ch 0                  receiver statistics
  opensent bridge 1 --ch 0 --sig FC1 --in 0 4095 --out 0.5 4.5 --fail 0 100
  opensent selftest [--jumper]          end-to-end hardware verification
  opensent decode capture.csv|.vcd      offline decoder (no hardware)
  opensent synth out.vcd --fc1 ...      synthesize a SENT waveform file
  opensent sim                          start the host simulator (pty)

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time

from . import codec


def _dev(args):
    from .device import Device
    return Device(args.port)


def cmd_list(args) -> int:
    from .device import find_ports, Device
    ports = find_ports()
    if not ports:
        print("no OpenSENT instrument found")
        return 1
    for p in ports:
        try:
            with Device(p) as d:
                print("%s  %s" % (p, d.idn()))
        except Exception as e:  # noqa: BLE001
            print("%s  (%s)" % (p, e))
    return 0


def cmd_q(args) -> int:
    with _dev(args) as d:
        for c in args.commands:
            print(d.query(c))
    return 0


def cmd_tx(args) -> int:
    cmds = []
    if args.tick is not None:
        cmds.append("TX:TICK %g" % args.tick)
    if args.nibbles is not None:
        cmds.append("TX:NIB %d" % args.nibbles)
    if args.crc:
        cmds.append("TX:CRC %s" % args.crc)
    if args.fmt:
        cmds.append("TX:FMT %s" % args.fmt)
    if args.status is not None:
        cmds.append("TX:STATUS %d" % args.status)
    if args.fc1 is not None:
        cmds.append("TX:FC1 %d" % args.fc1)
    if args.fc2 is not None:
        cmds.append("TX:FC2 %d" % args.fc2)
    for w in args.wave or []:
        cmds.append("TX:WAVE %s" % w)
    if args.raw:
        cmds.append("TX:RAW %s" % args.raw)
    if args.pause:
        cmds.append("TX:PAUSE %s" % args.pause)
    if args.slow:
        cmds.append("TX:SLOW %s" % args.slow)
        cmds.append("TX:SLOW:CLR")
        for m in args.msg or []:
            i, v = m.split("=")
            cmds.append("TX:SLOW:ADD %d,%d" % (int(i, 0), int(v, 0)))
    if args.fault:
        cmds.append("TX:FAULT %s" % args.fault)
    if args.drive:
        cmds.append("TX:DRIVE %s" % args.drive)
    if args.on:
        cmds.append("TX:STATE ON")
    if args.off:
        cmds.append("TX:STATE OFF")
    with _dev(args) as d:
        for c in cmds:
            d.cmd(c)
        print(d.query("TX:INFO?"))
    return 0


def _fmt_stream(f) -> str:
    flags = []
    if f.flags & codec.F_PAUSE:
        flags.append("P%d" % f.pause)
    if f.flags & codec.F_CAL:
        flags.append("CAL")
    if f.flags & codec.F_MARGIN:
        flags.append("MARGIN")
    if f.flags & codec.F_TICK:
        flags.append("TICK")
    return "%12.1f  RX%d %-6s S=%X D=%-8s C=%X  FC1=%5d FC2=%5d  tick=%7.1fns %s" % (
        f.ts_us, f.ch, f.err, f.status, f.data, f.crc, f.fc1, f.fc2, f.tick_ns, " ".join(flags))


def cmd_monitor(args) -> int:
    with _dev(args) as d:
        d.cmd("RX%d:STATE ON" % args.ch)
        n = 0
        t0 = time.monotonic()
        try:
            for f in d.iter_stream(1 << args.ch, args.every):
                if not args.errors_only or not f.ok:
                    print(_fmt_stream(f), flush=True)
                n += 1
                if args.count and n >= args.count:
                    break
        except KeyboardInterrupt:
            pass
        dt = time.monotonic() - t0
        print("# %d frames in %.1f s" % (n, dt), file=sys.stderr)
        print("# " + d.query("RX%d:STAT?" % args.ch), file=sys.stderr)
        slow = d.rx_slow(args.ch)
        for fmt, i, v in slow:
            print("# slow %s id 0x%02X = %d (0x%X)" % (fmt, i, v, v), file=sys.stderr)
    return 0


def cmd_stat(args) -> int:
    with _dev(args) as d:
        st = d.rx_stat(args.ch)
        for k, v in st.items():
            print("%-10s %s" % (k, v))
        for fmt, i, v in d.rx_slow(args.ch):
            print("slow %-5s id 0x%02X = %d (0x%X)" % (fmt, i, v, v))
    return 0


def cmd_bridge(args) -> int:
    with _dev(args) as d:
        d.cmd("RX%d:STATE ON" % args.ch)
        d.cmd("BRIDGE%d:MAP RX%d,%s,%g,%g,%g,%g" % (args.dac, args.ch, args.sig, args.inp[0], args.inp[1],
                                                    args.out[0], args.out[1]))
        if args.fail:
            d.cmd("BRIDGE%d:FAIL %g,%d" % (args.dac, args.fail[0], int(args.fail[1])))
        d.cmd("BRIDGE%d:STATE %s" % (args.dac, "OFF" if args.disable else "ON"))
        print(d.query("BRIDGE%d:MAP?" % args.dac), d.query("BRIDGE%d:OUT?" % args.dac))
    return 0


def cmd_selftest(args) -> int:
    from .selftest import SelfTest
    with _dev(args) as d:
        rep = SelfTest(d, jumper=args.jumper, quick=args.quick).run()
    return 0 if rep.ok else 1


def _load_edges(path: str, invert: bool):
    if path.lower().endswith(".vcd"):
        fall, rise = codec.read_edges_vcd(path)
    else:
        fall, rise = codec.read_edges_csv(path)
    return rise if invert else fall


def cmd_decode(args) -> int:
    edges = _load_edges(args.file, args.invert)
    dec = codec.Decoder(clk_hz=1.0, nibbles=args.nibbles, fmt=args.fmt,
                        crc_mode={"AUTO": codec.CRC_AUTO, "2010": codec.CRC_2010, "LEGACY": codec.CRC_LEGACY,
                                  "STATUS": codec.CRC_STATUS}[args.crc],
                        tick_ns=args.tick * 1000 if args.tick else 0.0)
    out = open(args.csv, "w") if args.csv else None
    if out:
        out.write("t_s,err,status,data,crc,crc_mode,fc1,fc2,tick_us,pause_ticks,flags,max_dev_ticks\n")
    shown = 0
    for f in dec.decode_edges(edges):
        tick_us = f.tick_ns / 1000.0
        if out:
            out.write("%.9f,%s,%X,%s,%X,%s,%d,%d,%.4f,%d,%d,%.3f\n" % (
                f.ts, f.err_name, f.status, f.data_hex, f.crc, codec.CRC_NAMES.get(f.crc_mode, "-"), f.fc1,
                f.fc2, tick_us, f.pause_ticks, f.flags, f.max_dev))
        if not args.quiet and (shown < args.show or (not f.ok and args.errors)):
            print("%12.6f s %-6s S=%X D=%-8s C=%X FC1=%5d FC2=%5d tick=%.3fus%s%s" % (
                f.ts, f.err_name, f.status, f.data_hex, f.crc, f.fc1, f.fc2, tick_us,
                " pause=%d" % f.pause_ticks if f.pause_ticks else "",
                "  slow %s id 0x%02X=%d" % (f.slow.fmt_name, f.slow.id, f.slow.value) if f.slow else ""))
            shown += 1
    if out:
        out.close()
    st = dec.st
    okp = 100.0 * st.ok / st.frames if st.frames else 0.0
    print("frames %d  ok %d (%.1f %%)  crc %d nibble %d length %d short %d sync %d  pauses %d  glitches %d" % (
        st.frames, st.ok, okp, st.err[1], st.err[2], st.err[3], st.err[4], st.err[5], st.pauses, st.glitches))
    print("nibbles %s  crc %s  slow ok %d err %d" % (dec._known_n() or "?", codec.CRC_NAMES.get(dec.crc_lock, "-"),
                                                     dec.slow.ok, dec.slow.crc_err))
    for m in sorted(dec.slow.table.values(), key=lambda m: (m.fmt, m.id)):
        print("  slow %-5s id 0x%02X = %5d (0x%04X)  x%d" % (m.fmt_name, m.id, m.value, m.value, m.count))
    return 0 if st.ok else 1


def cmd_synth(args) -> int:
    cfg = codec.EncoderConfig(nibbles=args.nibbles, crc_mode={"2010": 0, "LEGACY": 1, "STATUS": 2}[args.crc],
                              fmt=args.fmt, pause_ticks=args.pause,
                              slow_fmt=codec.SLOW_NAMES.index(args.slow) if args.slow else 0,
                              slow_msgs=[tuple(int(x, 0) for x in m.split("=")) for m in (args.msg or [])])
    enc = codec.Encoder(cfg)
    tick = args.tick * 1e-6
    low = 5 * tick
    levels = [(0.0, 1)]
    t = 10 * tick
    for i in range(args.frames):
        fc1 = (args.fc1 + i * args.ramp) % (1 << (4 * codec.FORMATS[args.fmt][0])) if codec.FORMATS[args.fmt][0] else 0
        for p in enc.frame(fc1, args.fc2):
            levels.append((t, 0))
            levels.append((t + low, 1))
            t += p * tick
    levels.append((t, 0))
    levels.append((t + low, 1))
    if args.out.lower().endswith(".vcd"):
        codec.write_vcd(args.out, levels, timescale_ns=args.timescale)
    else:
        with open(args.out, "w") as fp:
            fp.write("Time[s], Channel 0\n")
            for tt, v in levels:
                fp.write("%.9f, %d\n" % (tt, v))
    print("wrote %d frames (%.3f s) to %s" % (args.frames, t, args.out))
    return 0


def _find_sim() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "..", "..", "build-sim", "opensent-sim"), shutil.which("opensent-sim") or ""):
        if cand and os.path.exists(cand):
            return os.path.abspath(cand)
    return ""


def cmd_sim(args) -> int:
    exe = _find_sim()
    if not exe:
        print("opensent-sim not built: cmake -S firmware/sim -B build-sim && cmake --build build-sim")
        return 1
    link = args.link
    print("simulator: %s -> %s (Ctrl-C to stop)" % (exe, link))
    try:
        return subprocess.call([exe, "--link", link])
    except KeyboardInterrupt:
        return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="opensent", description="OpenSENT - SAE J2716 SENT toolkit")
    ap.add_argument("--port", help="serial port (default: auto-detect by USB product 'OpenSENT')")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="find instruments").set_defaults(fn=cmd_list)

    p = sub.add_parser("q", help="send raw protocol commands")
    p.add_argument("commands", nargs="+")
    p.set_defaults(fn=cmd_q)

    p = sub.add_parser("tx", help="configure the sensor emulator")
    p.add_argument("--tick", type=float, help="tick in us (3 = J2716 default)")
    p.add_argument("--nibbles", type=int)
    p.add_argument("--crc", choices=["2010", "LEGACY", "STATUS"])
    p.add_argument("--fmt", choices=list(codec.FORMATS))
    p.add_argument("--status", type=int, help="application status bits 0..3")
    p.add_argument("--fc1", type=int)
    p.add_argument("--fc2", type=int)
    p.add_argument("--wave", action="append", help="CH,KIND,MIN,MAX,PERIOD_S e.g. 1,SINE,0,4095,2")
    p.add_argument("--raw", help="raw data nibbles in hex")
    p.add_argument("--pause", help="OFF | ticks | CONST,frame_ticks")
    p.add_argument("--slow", choices=["OFF", "SHORT", "ENH12", "ENH16"])
    p.add_argument("--msg", action="append", help="slow message ID=VALUE (repeat)")
    p.add_argument("--fault", help="TYPE[,COUNT[,EVERY]]  CRC|SYNC|NIBBLE|SHORT|LONG|CAL|SCRC|OFF")
    p.add_argument("--drive", choices=["PP", "OD"])
    p.add_argument("--on", action="store_true")
    p.add_argument("--off", action="store_true")
    p.set_defaults(fn=cmd_tx)

    p = sub.add_parser("monitor", help="live decoded frames")
    p.add_argument("--ch", type=int, default=0)
    p.add_argument("--every", type=int, default=1, help="show every Nth good frame (errors always)")
    p.add_argument("--count", type=int, default=0)
    p.add_argument("--errors-only", action="store_true")
    p.set_defaults(fn=cmd_monitor)

    p = sub.add_parser("stat", help="receiver statistics")
    p.add_argument("--ch", type=int, default=0)
    p.set_defaults(fn=cmd_stat)

    p = sub.add_parser("bridge", help="SENT -> analog output for ECUs without SENT input")
    p.add_argument("dac", type=int, choices=[1, 2])
    p.add_argument("--ch", type=int, default=0)
    p.add_argument("--sig", default="FC1", choices=["FC1", "FC2"])
    p.add_argument("--in", dest="inp", type=float, nargs=2, default=[0, 4095], metavar=("MIN", "MAX"))
    p.add_argument("--out", type=float, nargs=2, default=[0, 3.3], metavar=("VMIN", "VMAX"))
    p.add_argument("--fail", type=float, nargs=2, metavar=("VOLTS", "TIMEOUT_MS"))
    p.add_argument("--disable", action="store_true")
    p.set_defaults(fn=cmd_bridge)

    p = sub.add_parser("selftest", help="end-to-end verification on the hardware (or simulator)")
    p.add_argument("--jumper", action="store_true", help="TX (PB10) is wired to RX0 (PA0): test RX0 too")
    p.add_argument("--quick", action="store_true")
    p.set_defaults(fn=cmd_selftest)

    p = sub.add_parser("decode", help="decode a logic analyzer capture (CSV time,level or VCD)")
    p.add_argument("file")
    p.add_argument("--nibbles", type=int, default=0)
    p.add_argument("--fmt", default="12_12", choices=list(codec.FORMATS))
    p.add_argument("--crc", default="AUTO", choices=["AUTO", "2010", "LEGACY", "STATUS"])
    p.add_argument("--tick", type=float, default=0.0, help="nominal tick in us (default: auto)")
    p.add_argument("--invert", action="store_true", help="signal is inverted (use rising edges)")
    p.add_argument("--csv", help="write every frame to this CSV")
    p.add_argument("--show", type=int, default=10, help="print the first N frames")
    p.add_argument("--errors", action="store_true", help="print every bad frame")
    p.add_argument("--quiet", action="store_true")
    p.set_defaults(fn=cmd_decode)

    p = sub.add_parser("synth", help="write a synthetic SENT waveform (VCD or CSV)")
    p.add_argument("out")
    p.add_argument("--frames", type=int, default=200)
    p.add_argument("--tick", type=float, default=3.0)
    p.add_argument("--nibbles", type=int, default=6)
    p.add_argument("--fmt", default="12_12", choices=list(codec.FORMATS))
    p.add_argument("--crc", default="2010", choices=["2010", "LEGACY", "STATUS"])
    p.add_argument("--fc1", type=int, default=1000)
    p.add_argument("--fc2", type=int, default=3000)
    p.add_argument("--ramp", type=int, default=1, help="fc1 increment per frame")
    p.add_argument("--pause", type=int, default=0)
    p.add_argument("--slow", choices=["SHORT", "ENH12", "ENH16"])
    p.add_argument("--msg", action="append")
    p.add_argument("--timescale", type=int, default=10, help="VCD timescale in ns")
    p.set_defaults(fn=cmd_synth)

    p = sub.add_parser("sim", help="run the host simulator behind a pty")
    p.add_argument("--link", default="/tmp/opensent-sim")
    p.set_defaults(fn=cmd_sim)

    args = ap.parse_args(argv)
    try:
        return args.fn(args)
    except Exception as e:  # noqa: BLE001
        from .device import DeviceError
        if isinstance(e, DeviceError):
            print("error: %s" % e, file=sys.stderr)
            return 2
        raise


if __name__ == "__main__":
    sys.exit(main())
