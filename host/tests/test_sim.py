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
