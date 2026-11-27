"""OpenSENT MCP server: lets an AI agent drive and verify the instrument.

Model Context Protocol over stdio (newline-delimited JSON-RPC 2.0), no
dependencies. Register it with an MCP client, e.g.

    claude mcp add opensent -- opensent-mcp            # or: python -m opensent.mcp_server

The serial port is auto-detected (USB product "OpenSENT"), or set
OPENSENT_PORT (a simulator pty works too).

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import json
import os
import sys
import traceback
from typing import Any, Callable, Dict, List, Optional

from . import __version__, codec
from .device import Device, parse_kv

PROTOCOL_VERSION = "2024-11-05"

_dev: Optional[Device] = None


def dev() -> Device:
    global _dev
    if _dev is None:
        _dev = Device(os.environ.get("OPENSENT_PORT") or None)
    return _dev


# ------------------------------------------------------------------ tools

def t_identify(_: Dict) -> Any:
    d = dev()
    return {"idn": d.idn(), "pins": d.query("PINS?"), "logic_analyzer": d.query("LA:INFO?"),
            "startup_script": d.query("CFG:INFO?"), "port": d.port}


def t_command(a: Dict) -> Any:
    replies = []
    for line in a["commands"]:
        try:
            replies.append({"command": line, "reply": dev().query(line)})
        except Exception as e:  # noqa: BLE001
            replies.append({"command": line, "error": str(e)})
            break
    return replies


def t_emulate(a: Dict) -> Any:
    d = dev()
    cmds = []
    if "tick_us" in a:
        cmds.append("TX:TICK %g" % a["tick_us"])
    if "crc" in a:
        cmds.append("TX:CRC %s" % a["crc"])
    if "format" in a:
        cmds.append("TX:FMT %s" % a["format"])
    if "nibbles" in a:
        cmds.append("TX:NIB %d" % a["nibbles"])
    if "status_bits" in a:
        cmds.append("TX:STATUS %d" % a["status_bits"])
    if "pause" in a:
        cmds.append("TX:PAUSE %s" % a["pause"])
    for ch in (1, 2):
        w = a.get("fc%d" % ch)
        if w is None:
            continue
        if isinstance(w, (int, float)):
            cmds.append("TX:FC%d %d" % (ch, int(w)))
        else:
            cmds.append("TX:WAVE %d,%s,%d,%d,%g" % (ch, w.get("kind", "CONST"), w.get("min", 0), w.get("max", 4095),
                                                   w.get("period_s", 1.0)))
    if "serial_format" in a:
        cmds += ["TX:SLOW %s" % a["serial_format"], "TX:SLOW:CLR"]
        for m in a.get("serial_messages", []):
            cmds.append("TX:SLOW:ADD %d,%d" % (m["id"], m["value"]))
    if "enabled" in a:
        cmds.append("TX:STATE %s" % ("ON" if a["enabled"] else "OFF"))
    for c in cmds:
        d.cmd(c)
    return {"applied": cmds, "tx": d.tx_info()}


def t_fault(a: Dict) -> Any:
    d = dev()
    d.cmd("TX:FAULT %s,%d,%d" % (a["type"], a.get("count", 1), a.get("every", 1)))
    return {"fault": d.query("TX:FAULT?")}


def t_receiver(a: Dict) -> Any:
    d = dev()
    ch = a.get("channel", 0)
    if a.get("enable", True):
        d.cmd("RX%d:STATE ON" % ch)
    for k, cmd in (("tick_us", "TICK"), ("nibbles", "NIB"), ("crc", "CRC"), ("format", "FMT")):
        if k in a:
            d.cmd("RX%d:%s %s" % (ch, cmd, a[k]))
    if a.get("clear"):
        d.cmd("RX%d:CLR" % ch)
    return {"stats": d.rx_stat(ch), "last_frame": d.rx_last(ch),
            "serial_messages": [{"format": f, "id": i, "value": v} for f, i, v in d.rx_slow(ch)]}


def t_frames(a: Dict) -> Any:
    ch = a.get("channel", 0)
    d = dev()
    d.cmd("RX%d:STATE ON" % ch)
    fr = d.stream(1 << ch, float(a.get("seconds", 0.2)), int(a.get("every", 1)))
    limit = int(a.get("max_frames", 50))
    return {"count": len(fr), "bad": sum(not f.ok for f in fr),
            "frames": [f.__dict__ for f in fr[:limit]]}


def t_report(a: Dict) -> Any:
    from . import report
    if a.get("file"):
        path = a["file"]
        fall, rise = codec.read_edges_vcd(path) if path.lower().endswith(".vcd") else codec.read_edges_csv(path)
    else:
        level0, _, times = dev().capture(a.get("channel", 0), int(a.get("edges", 6000)))
        fall = times[0::2] if level0 else times[1::2]
        rise = times[1::2] if level0 else times[2::2]
    an = report.analyze(fall, rise)
    if a.get("html"):
        with open(a["html"], "w") as fp:
            fp.write(report.render(an, a.get("title", "SENT report"), a.get("file") or "instrument"))
    keep = {k: v for k, v in an.items() if k not in ("series", "dev_hist")}
    keep["verdict"] = [{"level": l, "text": t} for l, t in an["verdict"]]
    return keep


def t_bridge(a: Dict) -> Any:
    d = dev()
    n = a.get("output", 1)
    if a.get("enabled", True):
        d.cmd("RX%d:STATE ON" % a.get("channel", 0))
        d.cmd("BRIDGE%d:MAP RX%d,%s,%g,%g,%g,%g" % (n, a.get("channel", 0), a.get("signal", "FC1"),
                                                    a.get("in_min", 0), a.get("in_max", 4095), a.get("v_min", 0.5),
                                                    a.get("v_max", 3.0)))
        d.cmd("BRIDGE%d:FAIL %g,%d" % (n, a.get("fail_volts", 0.0), a.get("timeout_ms", 100)))
    d.cmd("BRIDGE%d:STATE %s" % (n, "ON" if a.get("enabled", True) else "OFF"))
    out = d.query("BRIDGE%d:OUT?" % n).split(",")
    return {"commanded_v": float(out[0]), "state": out[1], "measured_v": float(out[2])}


def t_can(a: Dict) -> Any:
    d = dev()
    d.cmd("CAN:MODE %s" % ("LOOPBACK" if a.get("loopback") else "NORMAL"))
    d.cmd("CAN:BITRATE %d" % a.get("bitrate", 500000))
    if "channel" in a:
        d.cmd("RX%d:STATE ON" % a["channel"])
        d.cmd("CAN:MAP RX%d,%d,%d" % (a["channel"], a.get("id", 0x5A0), a.get("period_ms", 10)))
    d.cmd("CAN:STATE %s" % ("ON" if a.get("enabled", True) else "OFF"))
    return {"status": parse_kv(d.query("CAN:STAT?")), "last_rx": d.query("CAN:RX?")}


def t_selftest(a: Dict) -> Any:
    from .selftest import SelfTest
    lines: List[str] = []
    rep = SelfTest(dev(), quick=a.get("quick", True), log=lines.append).run()
    return {"passed": rep.ok, "checks": len(rep.results), "failed": [r.__dict__ for r in rep.results if not r.ok],
            "log": lines[-60:]}


S = {"type": "object"}
TOOLS: Dict[str, Dict[str, Any]] = {
    "sent_identify": {"fn": t_identify, "description": "Identify the OpenSENT instrument (model, firmware, pins, "
                      "logic analyzer and startup-script state).", "inputSchema": {"type": "object", "properties": {}}},
    "sent_command": {"fn": t_command, "description": "Send raw protocol lines (see docs/protocol.md), e.g. "
                     "'RX0:STAT?' or 'TX:TICK 3;TX:STATE ON'. Returns one reply per line.",
                     "inputSchema": {"type": "object", "properties": {"commands": {"type": "array", "items": {
                         "type": "string"}}}, "required": ["commands"]}},
    "sent_emulate": {"fn": t_emulate, "description": "Configure the SENT sensor emulator on the TX pin (PB10). "
                     "fc1/fc2 are a number (constant) or {kind: CONST|RAMP|TRI|SINE|SQUARE|COUNT, min, max, period_s}.",
                     "inputSchema": {"type": "object", "properties": {
                         "enabled": {"type": "boolean"}, "tick_us": {"type": "number"},
                         "crc": {"enum": ["2010", "LEGACY", "STATUS"]},
                         "format": {"enum": list(codec.FORMATS)}, "nibbles": {"type": "integer"},
                         "status_bits": {"type": "integer"}, "pause": {"type": "string",
                                                                       "description": "OFF | ticks | CONST,frame_ticks"},
                         "fc1": {}, "fc2": {},
                         "serial_format": {"enum": ["OFF", "SHORT", "ENH12", "ENH16"]},
                         "serial_messages": {"type": "array", "items": {"type": "object", "properties": {
                             "id": {"type": "integer"}, "value": {"type": "integer"}}}}}}},
    "sent_inject_fault": {"fn": t_fault, "description": "Inject protocol faults into the emulated sensor: "
                          "CRC, SYNC, NIBBLE, SHORT, LONG, CAL (calibration drift), SCRC (serial CRC), OFF.",
                          "inputSchema": {"type": "object", "properties": {
                              "type": {"enum": ["CRC", "SYNC", "NIBBLE", "SHORT", "LONG", "CAL", "SCRC", "OFF"]},
                              "count": {"type": "integer", "description": "0 = forever"},
                              "every": {"type": "integer"}}, "required": ["type"]}},
    "sent_receiver": {"fn": t_receiver, "description": "Enable/configure a SENT receiver (0 = PA0, 1 = PA1, "
                      "2 = monitor of the TX pin) and return its statistics, last frame and serial messages.",
                      "inputSchema": {"type": "object", "properties": {
                          "channel": {"type": "integer"}, "enable": {"type": "boolean"}, "clear": {"type": "boolean"},
                          "tick_us": {"type": "string"}, "nibbles": {"type": "string"}, "crc": {"type": "string"},
                          "format": {"type": "string"}}}},
    "sent_frames": {"fn": t_frames, "description": "Stream decoded frames from a receiver for a short time.",
                    "inputSchema": {"type": "object", "properties": {
                        "channel": {"type": "integer"}, "seconds": {"type": "number"}, "every": {"type": "integer"},
                        "max_frames": {"type": "integer"}}}},
    "sent_report": {"fn": t_report, "description": "Sensor health analysis (tick accuracy, jitter, nibble margin, "
                    "low time, errors, serial IDs) from a live two-edge capture or a CSV/VCD file; optional HTML.",
                    "inputSchema": {"type": "object", "properties": {
                        "channel": {"type": "integer"}, "edges": {"type": "integer"}, "file": {"type": "string"},
                        "html": {"type": "string"}, "title": {"type": "string"}}}},
    "sent_bridge": {"fn": t_bridge, "description": "SENT -> analog bridge on DAC PA4/PA5 with fail-safe; returns "
                    "commanded and measured (on-chip read-back) volts.",
                    "inputSchema": {"type": "object", "properties": {
                        "output": {"type": "integer"}, "enabled": {"type": "boolean"}, "channel": {"type": "integer"},
                        "signal": {"enum": ["FC1", "FC2"]}, "in_min": {"type": "number"}, "in_max": {"type": "number"},
                        "v_min": {"type": "number"}, "v_max": {"type": "number"},
                        "fail_volts": {"type": "number"}, "timeout_ms": {"type": "integer"}}}},
    "sent_can": {"fn": t_can, "description": "SENT -> CAN gateway (FDCAN1 PD0/PD1, or internal loopback).",
                 "inputSchema": {"type": "object", "properties": {
                     "enabled": {"type": "boolean"}, "channel": {"type": "integer"}, "id": {"type": "integer"},
                     "period_ms": {"type": "integer"}, "bitrate": {"type": "integer"},
                     "loopback": {"type": "boolean"}}}},
    "sent_selftest": {"fn": t_selftest, "description": "Run the end-to-end hardware self test (no wiring needed). "
                      "Resets the instrument configuration.",
                      "inputSchema": {"type": "object", "properties": {"quick": {"type": "boolean"}}}},
}


# ---------------------------------------------------------------- JSON-RPC

def handle(msg: Dict) -> Optional[Dict]:
    mid = msg.get("id")
    method = msg.get("method", "")
    if mid is None:
        return None  # notification
    try:
        if method == "initialize":
            res: Any = {"protocolVersion": PROTOCOL_VERSION, "capabilities": {"tools": {}},
                        "serverInfo": {"name": "opensent", "version": __version__},
                        "instructions": "Drive an OpenSENT SAE J2716 SENT instrument: emulate sensors, inject "
                                        "faults, decode and analyze SENT lines, bridge to analog/CAN. Verify what "
                                        "you configure with sent_receiver (channel 2 monitors the TX pin)."}
        elif method == "ping":
            res = {}
        elif method == "tools/list":
            res = {"tools": [{"name": n, "description": t["description"], "inputSchema": t["inputSchema"]}
                             for n, t in TOOLS.items()]}
        elif method == "tools/call":
            p = msg.get("params", {})
            tool = TOOLS.get(p.get("name"))
            if not tool:
                return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32602, "message": "unknown tool"}}
            try:
                out = tool["fn"](p.get("arguments") or {})
                res = {"content": [{"type": "text", "text": json.dumps(out, indent=1, default=str)}],
                       "isError": False}
            except Exception as e:  # noqa: BLE001
                global _dev
                if _dev is not None and "timeout" in str(e).lower():
                    _dev = None  # reconnect next time
                res = {"content": [{"type": "text", "text": "error: %s" % e}], "isError": True}
        else:
            return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32601, "message": "method not found"}}
        return {"jsonrpc": "2.0", "id": mid, "result": res}
    except Exception as e:  # noqa: BLE001
        traceback.print_exc(file=sys.stderr)
        return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32603, "message": str(e)}}


def main() -> int:
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        rep = handle(msg)
        if rep is not None:
            sys.stdout.write(json.dumps(rep) + "\n")
            sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
