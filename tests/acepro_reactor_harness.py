"""
Runs AFC_ACEPRO / AFC_ACEPRO_lane against a simulated ACE Pro
(tests/acepro_sim.py) on Klipper's *real* reactor, so the serial I/O,
completions, mutex and timer interplay are exercised for real.

Run as a subprocess (tests/test_AFC_ACEPRO_reactor.py does this) because the
AFC test conftest replaces chelper with a mock, and the real reactor needs the
real one:

    KLIPPER_PATH=~/klipper python3 tests/acepro_reactor_harness.py <scenario>

Prints one JSON object: {"checks": {name: bool}, "info": {...}}.
A reactor watchdog timer runs every 5 ms during each scenario; a blocking
implementation would show up as long gaps.
"""
from __future__ import annotations

import json
import os
import sys
import traceback
from unittest.mock import MagicMock

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE = os.environ["KLIPPER_PATH"]
KLIPPY = os.path.join(FIRMWARE, "klippy")
sys.path.insert(0, REPO)

# Real reactor + chelper. Klipper keeps klippy/ as a flat module directory;
# Kalico makes it a package (klippy.reactor with relative imports).
if os.path.isfile(os.path.join(KLIPPY, "__init__.py")):
    import types  # noqa: E402
    pkg = types.ModuleType("klippy")
    pkg.__path__ = [KLIPPY]
    sys.modules["klippy"] = pkg
    from klippy import reactor as kreactor  # noqa: E402
else:
    sys.path.append(KLIPPY)
    import reactor as kreactor  # noqa: E402
import tests.conftest  # noqa: E402,F401  installs Klipper mocks for everything else
from tests.conftest import MockAFC  # noqa: E402
from tests.acepro_sim import FakeACE  # noqa: E402
from extras.AFC_ACEPRO import afcACEPRO  # noqa: E402
from extras.AFC_ACEPRO_lane import AFCACEPROLane  # noqa: E402

# Short tube and quick timings so each scenario takes seconds
FAST = {
    "bowden_length": 120.0,
    "approach_length": 30.0,
    "feed_speed": 100.0,
    "approach_speed": 40.0,
    "retract_speed": 100.0,
    "grab_length": 20.0,
    "grab_speed": 50.0,
    "status_interval": 0.1,
    "request_timeout": 1.0,
    "poll_interval": 0.5,
    "reconnect_interval": 0.5,
    "start_timeout": 1.0,
    "move_timeout_margin": 3.0,
    "boot_wait": 0.0,
    "calibration_step": 25.0,
    "unload_retract_steps": "10:600, 10:1800",
}


class Defaults:
    """Stands in for a Klipper config section: returns each option's real
    default unless overridden."""

    def __init__(self, values):
        self.values = values

    def get(self, key, default=None):
        return self.values.get(key, default)

    def getint(self, key, default=None, **kw):
        return int(self.values.get(key, default))

    def getfloat(self, key, default=None, **kw):
        value = self.values.get(key, default)
        return None if value is None else float(value)

    def getboolean(self, key, default=None):
        return bool(self.values.get(key, default))


class Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, msg):
        self.lines.append("{}: {}".format(level, msg))

    def debug(self, msg):
        self._log("debug", msg)

    def info(self, msg):
        self._log("info", msg)

    def warning(self, msg):
        self._log("warning", msg)

    def error(self, msg):
        self._log("error", msg)


class Watchdog:
    def __init__(self, r, period=0.005):
        self.r = r
        self.period = period
        self.max_gap = 0.0
        self.slow_ticks = 0
        self.last = None
        self.timer = r.register_timer(self._tick, r.NOW)

    def _tick(self, eventtime):
        now = self.r.monotonic()
        if self.last is not None:
            gap = now - self.last - self.period
            self.max_gap = max(self.max_gap, gap)
            if gap > 0.05:
                self.slow_ticks += 1
        self.last = now
        return now + self.period

    def stop(self):
        self.r.unregister_timer(self.timer)

    @property
    def never_stalled(self):
        return self.slow_ticks <= 2 and self.max_gap < 0.25


class FakeExtruder:
    def __init__(self, sim):
        self.sim = sim
        self.name = "extruder"
        self.tool_start = "pin"
        self.fila_tool_start = None      # newer AFC reads the raw switch through this
        self.tool_stn = 30.0
        self.tool_load_speed = 25.0
        self.estats = MagicMock()
        self.moved = []
        self.lane_loaded = None

    @property
    def tool_start_state(self):
        return self.sim.toolhead_sensor


class FakeHub:
    """[AFC_hub] with switch_pin on the splitter sensor (the K1 Max's stock
    rear runout sensor), or virtual."""

    def __init__(self, sim, real=True):
        self.sim = sim
        self.switch_pin = "PC15" if real else "virtual"

    @property
    def state(self):
        return self.sim.hub_sensor


class Rig:
    def __init__(self, r, sim, hub=True, **overrides):
        self.r = r
        self.sim = sim
        self.logger = Logger()
        self.scripts = []

        afc = MockAFC()
        afc.tool_cut = True
        afc.tool_cut_cmd = "AFC_CUT"
        afc.park = False
        afc.park_cmd = None
        afc.form_tip = False
        afc._check_extruder_temp = MagicMock(return_value=False)
        afc.error.handle_lane_failure = MagicMock()
        afc.save_vars = MagicMock()
        afc.function.is_printing = MagicMock(return_value=False)

        def run_script_from_command(cmd):
            self.scripts.append(cmd)
            if cmd.startswith("AFC_CUT"):
                sim.cut()

        afc.gcode.run_script_from_command = run_script_from_command
        afc.gcode.run_script = MagicMock(side_effect=lambda cmd: self.scripts.append(cmd))
        self.extruder = FakeExtruder(sim)
        self.hub = FakeHub(sim, real=hub)

        def move_e_pos(distance, speed, label="", wait_tool=False):
            self.extruder.moved.append((round(distance, 3), label))
            duration = min(abs(distance) / max(speed, 0.1), 1.0)
            for _ in range(10):
                sim.extruder_move(distance / 10.0)
                r.pause(r.monotonic() + duration / 10.0)

        afc.move_e_pos = move_e_pos
        self.afc = afc
        self.unit = self.make_unit(**overrides)

    def make_unit(self, **overrides):
        unit = afcACEPRO.__new__(afcACEPRO)
        unit.reactor = self.r
        unit.logger = self.logger
        unit.name = "ACE_1"
        unit.lanes = {}
        unit.afc = self.afc
        values = dict(FAST)
        values.update(overrides)
        values.setdefault("serial", self.sim.path)
        unit._load_ace_config(Defaults(values))
        unit._init_ace_runtime()
        # LED handling needs a full AFC lane/unit; record the calls instead
        for name in ("lane_tool_loaded", "lane_tool_unloaded", "lane_loaded", "lane_not_ready",
                     "lane_tool_loaded_idle"):
            setattr(unit, name, MagicMock())
        unit.on_filament_insert = MagicMock(
            side_effect=lambda lane: unit._tag_wait.__setitem__(
                lane.name, unit.reactor.monotonic() + unit.rfid_wait) if unit.rfid_apply else None)
        unit.on_filament_remove = MagicMock()
        return unit

    def lane(self, index, name=None):
        unit = self.unit
        lane = AFCACEPROLane.__new__(AFCACEPROLane)
        lane.name = name or "lane{}".format(index)
        lane.index = index
        lane.unit_obj = unit
        lane.afc = self.afc
        lane.reactor = self.r
        lane.logger = self.logger
        lane.extruder_obj = self.extruder
        lane.hub_obj = self.hub
        lane.ace_bowden_length = None
        lane.assist_enabled = True
        lane.select_lane = MagicMock()
        lane.disable_buffer = MagicMock()
        lane.set_loaded = MagicMock()
        lane.set_unloaded = MagicMock()
        lane._perform_pause_runout = MagicMock()
        lane._perform_infinite_runout = MagicMock()
        lane.loaded_to_hub = False
        lane.last_load_report = {}
        lane._ace_present_last = None
        lane._load_state = False
        lane.status = None
        lane.tool_loaded = False
        lane.runout_lane = None
        lane.spool_id = None
        lane._afc_prep_done = True
        unit.lanes[lane.name] = lane
        return lane

    @property
    def failures(self):
        return [c.args[1] for c in self.afc.error.handle_lane_failure.call_args_list]

    def cut_calls(self):
        return [c for c in self.scripts if c.startswith("AFC_CUT")]

    def start(self):
        """klippy:ready: start the poll and event timers."""
        self.unit._ace_handle_ready()

    def stop(self):
        self.unit._ace_handle_disconnect()


def wait(r, seconds):
    r.pause(r.monotonic() + seconds)


def _prep_lane(rig, lane):
    lane.prep_state = None
    lane.map = "T{}".format(lane.index - 1)
    lane.send_lane_data = MagicMock()
    lane.do_enable = MagicMock()
    lane.set_afc_prep_done = MagicMock(side_effect=lambda: setattr(lane, "_afc_prep_done", True))
    return rig.unit.system_Test(lane, 0, False, False)


def _load(rig, lane):
    lane.cmd_AFC_ACEPRO_TOOL_LOAD(None)


def _unload(rig, lane):
    lane.cmd_AFC_ACEPRO_TOOL_UNLOAD(None)


class Gcmd:
    def __init__(self, **params):
        self.params = params
        self.replies = []

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_int(self, key, default=None, **kw):
        v = self.params.get(key, default)
        return None if v is None else int(v)

    def get_float(self, key, default=None, **kw):
        v = self.params.get(key, default)
        return None if v is None else float(v)

    def respond_info(self, msg):
        self.replies.append(msg)

    def error(self, msg):
        return RuntimeError(msg)


# -- scenarios ---------------------------------------------------------------

def sc_connect(r):
    sim = FakeACE()
    rig = Rig(r, sim)
    ok = rig.unit.connect()
    sim.close()
    return {
        "online": ok and rig.unit.online,
        "info_read": rig.unit.info.get("firmware") == "V1.3.84",
    }, {"info": rig.unit.info}


def sc_keepalive_holds(r):
    """Polling keeps the ACE on USB well past its 3 s timeout."""
    sim = FakeACE(keepalive=1.5)
    rig = Rig(r, sim, poll_interval=0.5)
    rig.unit.connect()
    rig.start()
    dog = Watchdog(r)
    wait(r, 5.0)
    dog.stop()
    rig.stop()
    sim.close()
    return {
        "no_usb_reset": sim.usb_resets == 0,
        "stayed_online": rig.unit.online,
        "polled": sim.count("get_status") >= 8,
        "reactor_never_stalled": dog.never_stalled,
    }, {"resets": sim.usb_resets, "polls": sim.count("get_status")}


def sc_usb_reset_recovers(r):
    """The ACE drops off USB; the unit reopens the same path and turns feed
    assist back on for the loaded lane."""
    sim = FakeACE()
    rig = Rig(r, sim)
    lane = rig.lane(1)
    rig.unit.connect()
    lane.tool_loaded = True
    rig.extruder.lane_loaded = lane.name
    rig.start()
    wait(r, 1.2)
    assist_before = sim.assist_slot
    dog = Watchdog(r)
    sim.usb_reset()
    wait(r, 0.2)
    went_offline_or_reopened = (not rig.unit.online) or rig.unit.transport.opens > 1
    wait(r, 3.0)
    dog.stop()
    rig.stop()
    sim.close()
    return {
        "assist_on_before": assist_before == 0,
        "noticed_reset": went_offline_or_reopened,
        "back_online": rig.unit.online,
        "port_reopened": rig.unit.transport.opens >= 2,
        "assist_back_on": sim.assist_slot == 0,
        "reactor_never_stalled": dog.never_stalled,
    }, {"opens": rig.unit.transport.opens, "assist": sim.assist_slot,
        "log": [l for l in rig.logger.lines if "ACE" in l][-8:]}


def sc_poll_presence(r):
    sim = FakeACE(present=(True, True, False, False))
    rig = Rig(r, sim, poll_interval=0.2)
    lanes = [rig.lane(i) for i in (1, 2, 3)]
    rig.unit.connect()
    rig.start()
    wait(r, 0.8)
    initial = [l._ace_present_last for l in lanes]
    sim.insert(2)
    sim.remove(0)
    wait(r, 1.2)
    after = [l._ace_present_last for l in lanes]
    rig.stop()
    sim.close()
    return {
        "initial_presence": initial == [True, True, False],
        "presence_follows_ace": after == [False, True, True],
        "insert_marks_loaded": lanes[2].set_loaded.called,
        "insert_event_sent": rig.unit.on_filament_insert.called,
        "removal_marks_unloaded": lanes[0].set_unloaded.called,
    }, {"initial": initial, "after": after}


def sc_busy_flicker_ignored(r):
    """Other slots read 'empty' while the ACE is busy moving one: that's not
    a removal."""
    sim = FakeACE(present=(True, True, False, False))
    rig = Rig(r, sim, poll_interval=0.2)
    l1, l2 = rig.lane(1), rig.lane(2)
    rig.unit.connect()
    rig.start()
    wait(r, 0.6)
    # A long eject on slot 1 keeps the unit busy for a while (no operation
    # context: a raw move, the way AFC_ACEPRO_RAW would)
    rig.unit.request("unwind_filament", {"index": 0, "length": 60, "speed": 40, "mode": 0})
    wait(r, 1.4)
    rig.stop()
    sim.close()
    return {
        "lane2_not_removed": l2._ace_present_last is True and not l2.set_unloaded.called,
        "lane1_still_present": l1._ace_present_last is True,
    }, {"l1": l1._ace_present_last, "l2": l2._ace_present_last}


def sc_load_unload(r):
    sim = FakeACE(reply_delay=0.02)
    rig = Rig(r, sim)
    lane = rig.lane(1)
    rig.unit.connect()
    rig.start()
    dog = Watchdog(r)

    _load(rig, lane)
    loaded_sensor = sim.toolhead_sensor
    gripped = sim.slots[0].position >= sim.grip_len
    assist_after_load = sim.assist_slot
    report = dict(lane.last_load_report)
    tooled = lane.tool_loaded and rig.extruder.lane_loaded == lane.name
    moves_after_load = [m for m in rig.extruder.moved]
    wait(r, 1.0)

    _unload(rig, lane)
    dog.stop()
    rig.stop()
    sim.close()
    return {
        "no_failures": rig.failures == [],
        "reached_toolhead": loaded_sensor,
        "gears_hold_filament": gripped,
        "feed_assist_on": assist_after_load == 0,
        "grab_then_tool_stn": [m[0] for m in moves_after_load] == [20.0, 10.0],
        "afc_marked_tool_loaded": tooled,
        "cut_before_unload": len(rig.cut_calls()) == 1,
        "toolhead_clear_after_unload": not sim.toolhead_sensor,
        "hub_clear_after_unload": not sim.hub_sensor,
        "staged_not_past_slot_sensor": sim.slots[0].position > sim.empty_at,
        "assist_off_after_unload": sim.assist_slot is None,
        "afc_marked_tool_unloaded": not lane.tool_loaded and rig.extruder.lane_loaded is None,
        "shared_path_free": lane._load_state is False,
        "no_usb_reset": sim.usb_resets == 0,
        "reactor_never_stalled": dog.never_stalled,
    }, {"max_gap_ms": round(dog.max_gap * 1000, 2), "report": report,
        "position": sim.slots[0].position, "moves": rig.extruder.moved,
        "failures": rig.failures, "ignored_stops": sim.ignored_stops}


def sc_slot_switch(r):
    sim = FakeACE()
    rig = Rig(r, sim)
    l1, l2 = rig.lane(1), rig.lane(2)
    rig.unit.connect()
    rig.start()
    _load(rig, l1)
    first = sim.slots[0].position >= sim.path_len
    _unload(rig, l1)
    _load(rig, l2)
    second = sim.slots[1].position >= sim.path_len
    rig.stop()
    sim.close()
    return {
        "no_failures": rig.failures == [],
        "slot1_loaded_first": first,
        "slot2_loaded_second": second,
        "slot1_back": sim.slots[0].position < sim.hub_len,
        "assist_follows": sim.assist_slot == 1,
    }, {"failures": rig.failures, "positions": [s.position for s in sim.slots]}


def sc_jam_fails_cleanly(r):
    """A feed that stalls in the tube fails the load and pulls the filament
    back out of the shared splitter, so another lane can still load."""
    sim = FakeACE()
    sim.jam_at = (0, 80.0)
    rig = Rig(r, sim, load_attempts=1)
    lane = rig.lane(1)
    rig.unit.connect()
    dog = Watchdog(r)
    started = r.monotonic()
    _load(rig, lane)
    took = r.monotonic() - started
    dog.stop()
    sim.close()
    return {
        "failed": len(rig.failures) == 1 and "load failed" in rig.failures[0],
        "not_marked_loaded": not lane.tool_loaded,
        "pulled_back_out_of_splitter": not sim.hub_sensor,
        # ...but not so far that the slot sensor loses it (the spool stays staged)
        "slot_still_present": sim.slots[0].position > sim.empty_at,
        "failed_in_reasonable_time": took < 15.0,
        "reactor_never_stalled": dog.never_stalled,
    }, {"failures": rig.failures, "took": round(took, 2),
        "position": sim.slots[0].position, "report": lane.last_load_report}


def sc_empty_slot(r):
    sim = FakeACE(present=(True, False, False, False))
    rig = Rig(r, sim)
    lane = rig.lane(2)
    rig.unit.connect()
    _load(rig, lane)
    sim.close()
    return {
        "refused": len(rig.failures) == 1 and "no spool in slot 2" in rig.failures[0],
        "no_feed_sent": sim.count("feed_filament") == 0,
    }, {"failures": rig.failures}


def sc_toolhead_occupied(r):
    sim = FakeACE()
    sim.slots[0].position = sim.path_len + 5     # filament already at the toolhead
    rig = Rig(r, sim)
    lane = rig.lane(2)
    rig.unit.connect()
    _load(rig, lane)
    sim.close()
    return {
        "refused": len(rig.failures) == 1 and "toolhead sensor already" in rig.failures[0],
        "no_feed_sent": sim.count("feed_filament") == 0,
    }, {"failures": rig.failures}


def sc_offline(r):
    sim = FakeACE()
    rig = Rig(r, sim, request_timeout=0.2)
    lane = rig.lane(1)
    rig.unit.connect()
    sim.mute = True
    dog = Watchdog(r)
    started = r.monotonic()
    _load(rig, lane)
    took = r.monotonic() - started
    dog.stop()
    sim.close()
    return {
        "failed": len(rig.failures) == 1,
        "fails_fast": took < 6.0,
        "no_feed_movement": sim.slots[0].position == 0.0,
        "reactor_never_stalled": dog.never_stalled,
    }, {"failures": rig.failures, "took": round(took, 2)}


def sc_forbidden_waits(r):
    """A move while the ACE is busy is answered FORBIDDEN; the unit waits
    for ready instead of failing."""
    sim = FakeACE()
    rig = Rig(r, sim)
    lane = rig.lane(1)
    rig.unit.connect()
    # Keep the ACE busy with an unrelated slot-2 move for ~0.6 s
    rig.unit.request("unwind_filament", {"index": 1, "length": 30, "speed": 50, "mode": 0})
    _load(rig, lane)
    sim.close()
    return {
        "loaded": rig.failures == [] and sim.toolhead_sensor,
    }, {"failures": rig.failures}


def sc_stop_not_too_early(r):
    """Filament that reaches the sensor within ~250 ms of the feed's start
    still gets stopped: the stop is delayed so the ACE doesn't drop it."""
    sim = FakeACE()
    sim.slots[0].position = sim.path_len - 3.0
    rig = Rig(r, sim, hub=False)     # it is already past the splitter sensor
    lane = rig.lane(1)
    rig.unit.connect()
    _load(rig, lane)
    sim.close()
    return {
        "loaded": rig.failures == [],
        "no_stop_ignored": sim.ignored_stops == 0,
    }, {"failures": rig.failures, "ignored": sim.ignored_stops}


def sc_stale_assist(r):
    """The ACE is still assisting another slot that the unit doesn't know
    about (after a USB reset or a raw command): the load still works."""
    sim = FakeACE()
    rig = Rig(r, sim)
    lane = rig.lane(1)
    rig.unit.connect()
    sim.assist_slot = 1
    _load(rig, lane)
    sim.close()
    return {
        "loaded": rig.failures == [] and sim.toolhead_sensor,
        "assist_moved_to_slot1": sim.assist_slot == 0,
    }, {"failures": rig.failures}


def sc_calibrate(r):
    sim = FakeACE()
    rig = Rig(r, sim)
    lane = rig.lane(1)
    rig.unit.connect()
    gcmd = Gcmd()
    lane.cmd_AFC_ACEPRO_CALIBRATE(gcmd)
    text = "\n".join(gcmd.replies)
    import re
    m = re.search(r"bowden_length: (\d+)", text)
    measured = int(m.group(1)) if m else None
    sim.close()
    return {
        "reported": measured is not None,
        "close_to_path": measured is not None and abs(measured - sim.path_len) <= 15,
        "filament_back": not sim.toolhead_sensor and not sim.hub_sensor,
    }, {"text": text, "measured": measured, "position": sim.slots[0].position}


def sc_rfid_tag_applied(r):
    sim = FakeACE(present=(False, False, False, False))
    rig = Rig(r, sim, poll_interval=0.2)
    rig.lane(1)
    rig.unit.connect()
    rig.start()
    wait(r, 0.6)
    sim.insert(0, tag={"type": "PETG", "color": [255, 128, 0]})
    wait(r, 2.0)
    rig.stop()
    sim.close()
    return {
        "color_set": "SET_COLOR LANE=lane1 COLOR=ff8000" in rig.scripts,
        "material_set": "SET_MATERIAL LANE=lane1 MATERIAL=PETG" in rig.scripts,
    }, {"scripts": rig.scripts}


def sc_runout_pause_doesnt_stall_keepalive(r):
    """A runout's PAUSE can hold the gcode mutex for a long time; the status
    poll (the ACE's keepalive) must keep running meanwhile."""
    sim = FakeACE(keepalive=1.5)
    rig = Rig(r, sim, poll_interval=0.3)
    lane = rig.lane(1)
    rig.unit.connect()
    lane.tool_loaded = True
    rig.extruder.lane_loaded = lane.name
    rig.afc.function.is_printing = MagicMock(return_value=True)
    lane._perform_pause_runout = MagicMock(side_effect=lambda: r.pause(r.monotonic() + 4.0))
    rig.start()
    wait(r, 0.8)
    sim.remove(0)
    wait(r, 5.5)
    rig.stop()
    sim.close()
    return {
        "paused": lane._perform_pause_runout.called,
        "no_usb_reset": sim.usb_resets == 0,
    }, {"resets": sim.usb_resets}


def sc_garbage_resync(r):
    sim = FakeACE()
    sim.garbage_before_reply = b"\x00\xff\xaa\x05\x00junk\xfe"
    rig = Rig(r, sim)
    rig.unit.connect()
    statuses = [rig.unit.get_status_now() for _ in range(5)]
    sim.close()
    return {"parses_through_garbage": all(s is not None for s in statuses)}, {
        "crc_errors": rig.unit.transport._parser.crc_errors}


def sc_late_reply_dropped(r):
    """A reply that arrives after its request timed out isn't taken as the
    answer to the next one."""
    sim = FakeACE(reply_delay=0.4)
    rig = Rig(r, sim, request_timeout=0.25)
    rig.unit.transport.open()
    first = rig.unit.transport.request("get_info", timeout=0.25)
    sim.reply_delay = 0.0
    wait(r, 0.3)
    second = rig.unit.transport.request("get_status", timeout=0.5)
    sim.close()
    return {
        "first_timed_out": first is None,
        "second_is_status": second is not None and isinstance(second.result, dict)
        and "slots" in second.result,
        "late_counted": rig.unit.transport.late_replies >= 1,
    }, {"late": rig.unit.transport.late_replies}


def sc_trace_recording(r):
    import tempfile
    path = os.path.join(tempfile.mkdtemp(), "ace_trace.jsonl")
    sim = FakeACE()
    rig = Rig(r, sim, trace_file=path)
    lane = rig.lane(1)
    rig.unit.connect()
    _load(rig, lane)
    _unload(rig, lane)
    rig.unit.trace.stop()
    sim.close()
    events = [json.loads(line)["ev"] for line in open(path)]
    return {
        "has_load_events": "load_start" in events and "load_end" in events,
        "has_moves": "move" in events,
        "has_wire": "tx" in events and "rx" in events,
        "has_unload": "unload_end" in events,
    }, {"count": len(events)}


def sc_offline_prep_then_online(r):
    """ACE never answered during PREP; when it does, lanes sync quietly (no
    insert event) and a tool-loaded lane stays loaded."""
    sim = FakeACE(present=(True, True, False, False))
    sim.mute = True
    rig = Rig(r, sim, request_timeout=0.2, poll_interval=0.2)
    rig.afc.current = None
    lanes = [rig.lane(i) for i in (1, 2, 3)]
    lanes[0].tool_loaded = True
    rig.extruder.lane_loaded = "lane1"
    for l in lanes:
        l._afc_prep_done = False
    results = [_prep_lane(rig, l) for l in lanes]
    rig.start()
    sim.mute = False
    wait(r, 3.0)
    rig.stop()
    sim.close()
    return {
        "prep_saw_offline": results == [False, False, False],
        "came_online": rig.unit.online,
        "presence_synced": [l._ace_present_last for l in lanes] == [True, True, False],
        "no_insert_event": not rig.unit.on_filament_insert.called,
        "tool_loaded_kept": lanes[0].tool_loaded and not lanes[0].set_unloaded.called,
        "tool_led_set": rig.unit.lane_tool_loaded_idle.called,
    }, {"results": results, "presence": [l._ace_present_last for l in lanes]}


def sc_dryer(r):
    sim = FakeACE()
    rig = Rig(r, sim)
    rig.unit.connect()
    rig.unit.cmd_AFC_ACEPRO_DRY(Gcmd(TEMP=45, DURATION=60))
    drying = dict(sim.dryer)
    too_hot = False
    try:
        rig.unit.cmd_AFC_ACEPRO_DRY(Gcmd(TEMP=65))
    except RuntimeError:
        too_hot = True
    rig.unit.cmd_AFC_ACEPRO_DRY_STOP(Gcmd())
    sim.close()
    return {
        "started": drying.get("status") == "drying" and drying.get("target_temp") == 45,
        "refuses_above_max": too_hot,
        "stopped": sim.dryer["status"] == "stop",
    }, {"dryer": drying}


SCENARIOS = {name[3:]: fn for name, fn in globals().items() if name.startswith("sc_")}


def run(name):
    r = kreactor.Reactor()
    out = {"checks": {}, "info": {}}

    def body(eventtime):
        try:
            checks, info = SCENARIOS[name](r)
            out["checks"] = checks
            out["info"] = info
        except Exception:
            out["exception"] = traceback.format_exc()
        finally:
            r.end()
        return r.NEVER

    r.register_timer(body, r.NOW)
    r.run()
    return out


if __name__ == "__main__":
    names = sys.argv[1:] or sorted(SCENARIOS)
    results = {n: run(n) for n in names}
    print(json.dumps(results if len(names) > 1 else results[names[0]], default=str))
