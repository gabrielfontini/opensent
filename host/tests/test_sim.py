"""Run the end-to-end self test against the host simulator (no hardware)."""
import os
import subprocess
import tempfile
import time

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SIM = os.path.join(ROOT, "build-sim", "opensent-sim")


@pytest.fixture
def sim_port():
    if not os.path.exists(SIM):
        pytest.skip("simulator not built")
    link = os.path.join(tempfile.mkdtemp(), "tty")
    p = subprocess.Popen([SIM, "--link", link], stdout=subprocess.PIPE)
    p.stdout.readline()
    time.sleep(0.1)
    yield link
    p.kill()
    p.wait()


def test_protocol_basics(sim_port):
    from opensent.device import Device, DeviceError
    with Device(sim_port) as d:
        assert d.idn().startswith("OpenSENT,SIM,")
        assert d.query("TX:TICK?") == "3.0000"
        assert d.query("TX:TICK 5;TX:NIB 4") == "OK;OK"
        with pytest.raises(DeviceError):
            d.query("TX:TICK 500")
        assert d.query("SYST:ERR?").startswith("-222")
        with pytest.raises(DeviceError):
            d.query("NOPE")


def test_selftest_on_simulator(sim_port):
    from opensent.device import Device
    from opensent.selftest import SelfTest
    with Device(sim_port) as d:
        rep = SelfTest(d, jumper=True, quick=True, log=lambda s: None).run()
    assert rep.ok, [r for r in rep.results if not r.ok]


def test_mcp_server_on_simulator(sim_port):
    """An MCP client drives the instrument: emulate, inject faults, read back."""
    import json
    import sys
    msgs = [
        {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2024-11-05"}},
        {"jsonrpc": "2.0", "method": "notifications/initialized"},
        {"jsonrpc": "2.0", "id": 2, "method": "tools/list"},
        {"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "sent_emulate", "arguments": {
            "enabled": True, "fc1": 1000, "fc2": 2000}}},
        {"jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": {"name": "sent_receiver", "arguments": {
            "channel": 2}}},
    ]
    env = dict(os.environ, OPENSENT_PORT=sim_port)
    p = subprocess.Popen([sys.executable, "-m", "opensent.mcp_server"], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, text=True, env=env)
    for m in msgs[:4]:
        p.stdin.write(json.dumps(m) + "\n")
    p.stdin.flush()
    time.sleep(0.5)
    p.stdin.write(json.dumps(msgs[4]) + "\n")
    p.stdin.close()
    out = [json.loads(line) for line in p.stdout]
    p.wait(timeout=10)
    by_id = {m["id"]: m for m in out}
    assert by_id[1]["result"]["serverInfo"]["name"] == "opensent"
    assert "sent_selftest" in [t["name"] for t in by_id[2]["result"]["tools"]]
    rx = json.loads(by_id[4]["result"]["content"][0]["text"])
    assert rx["stats"]["ok"] > 50 and rx["last_frame"]["fc1"] == 1000 and rx["last_frame"]["fc2"] == 2000
