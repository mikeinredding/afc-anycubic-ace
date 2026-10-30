"""
Runs the ACE Pro unit and lanes against a simulated ACE on Klipper's real
reactor (see tests/acepro_reactor_harness.py). Each scenario runs in its own
subprocess because conftest mocks chelper and the real reactor needs it.

Skipped unless KLIPPER_PATH points at a Klipper (or Kalico) checkout, e.g.
    KLIPPER_PATH=~/klipper ./run-tests.sh
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.join(HERE, "acepro_reactor_harness.py")
KLIPPER_PATH = os.environ.get("KLIPPER_PATH", "")

pytestmark = pytest.mark.skipif(
    not os.path.isfile(os.path.join(KLIPPER_PATH, "klippy", "reactor.py")),
    reason="KLIPPER_PATH not set to a Klipper checkout",
)

SCENARIOS = [
    "connect",
    "keepalive_holds",
    "usb_reset_recovers",
    "poll_presence",
    "busy_flicker_ignored",
    "load_unload",
    "slot_switch",
    "jam_fails_cleanly",
    "empty_slot",
    "toolhead_occupied",
    "offline",
    "forbidden_waits",
    "stop_not_too_early",
    "stale_assist",
    "calibrate",
    "rfid_tag_applied",
    "runout_pause_doesnt_stall_keepalive",
    "garbage_resync",
    "late_reply_dropped",
    "trace_recording",
    "offline_prep_then_online",
    "dryer",
]


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_ace_on_real_reactor(scenario):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(p for p in sys.path if p)
    proc = subprocess.run([sys.executable, HARNESS, scenario], capture_output=True,
                          text=True, timeout=120, env=env)
    assert proc.returncode == 0, proc.stderr
    result = json.loads(proc.stdout.strip().splitlines()[-1])
    assert "exception" not in result, result["exception"]
    failed = {k: v for k, v in result["checks"].items() if not v}
    assert not failed, "failed checks {} (info: {})".format(list(failed), result["info"])
