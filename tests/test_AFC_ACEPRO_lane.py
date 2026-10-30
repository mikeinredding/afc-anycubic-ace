"""Unit tests for AFC_ACEPRO_lane decision logic (runout routing, refusals)
and the unit's presence debounce. ACE I/O is mocked here;
tests/test_AFC_ACEPRO_reactor.py covers it for real."""
from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_ACEPRO import afcACEPRO
from extras.AFC_ACEPRO_lane import AFCACEPROLane
from extras.AFC_lane import AFCLaneState
from tests.conftest import MockAFC, MockLogger, MockReactor


class AdvancingReactor(MockReactor):
    """MockReactor whose pause() moves the clock, so timeout loops end."""

    def pause(self, until):
        self._monotonic = max(self._monotonic, until)
        return until


def _make_lane(index=1, printing=False, tool_loaded=False, runout_lane=None):
    lane = AFCACEPROLane.__new__(AFCACEPROLane)
    afc = MockAFC()
    afc.tool_cut = True
    afc.tool_cut_cmd = "AFC_CUT"
    afc.park = False
    afc.form_tip = False
    afc._check_extruder_temp = MagicMock(return_value=False)
    afc.error.handle_lane_failure = MagicMock()
    afc.gcode.run_script_from_command = MagicMock()
    afc.save_vars = MagicMock()
    afc.function.is_printing = MagicMock(return_value=printing)

    unit = MagicMock()
    unit.online = True
    unit.cutter_sensor_pin = None
    unit.cutter_sensor_state = False
    unit.cutter_wait_time = 0.01
    unit.cutter_trigger_count = 0
    unit.cutter_require_trigger = True
    unit.bowden_length = 900.0
    unit.get_status_now = MagicMock(return_value={"status": "ready", "slots": [
        {"status": "ready"}, {"status": "ready"}, {"status": "empty"}, {"status": "ready"}]})
    unit.slot_presence = lambda st, slot: afcACEPRO.slot_presence(None, st, slot)

    lane.name = "lane{}".format(index)
    lane.index = index
    lane.afc = afc
    lane.unit_obj = unit
    lane.reactor = AdvancingReactor()
    lane.logger = MockLogger()
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.name = "extruder"
    lane.extruder_obj.tool_start = "pin"
    lane.extruder_obj.fila_tool_start = None
    lane.extruder_obj.tool_start_state = False
    lane.hub_obj = None
    lane.ace_bowden_length = None
    lane.assist_enabled = True
    lane.select_lane = MagicMock()
    lane.disable_buffer = MagicMock()
    lane.set_loaded = MagicMock()
    lane.set_unloaded = MagicMock()
    lane._perform_pause_runout = MagicMock()
    lane._perform_infinite_runout = MagicMock()
    lane._afc_prep_done = True
    lane._ace_present_last = True
    lane.status = AFCLaneState.LOADED
    lane.tool_loaded = tool_loaded
    lane.runout_lane = runout_lane
    lane.last_load_report = {}
    return lane


class TestPresence:
    def test_first_report_only_mirrors_state(self):
        lane = _make_lane()
        lane._ace_present_last = None
        lane.ace_update_presence(True)
        assert lane.prep_state is True and lane.loaded_to_hub is True
        lane.set_loaded.assert_not_called()

    def test_insert_marks_loaded(self):
        lane = _make_lane()
        lane._ace_present_last = False
        lane.status = AFCLaneState.NONE
        lane.ace_update_presence(True)
        lane.set_loaded.assert_called_once()
        lane.unit_obj.on_filament_insert.assert_called_once_with(lane)

    def test_removal_when_idle_unloads(self):
        lane = _make_lane()
        lane.ace_update_presence(False)
        lane.set_unloaded.assert_called_once()
        lane._perform_pause_runout.assert_not_called()

    def test_runout_while_printing_pauses(self):
        lane = _make_lane(printing=True, tool_loaded=True)
        lane.ace_update_presence(False)
        lane._perform_pause_runout.assert_called_once()
        lane._perform_infinite_runout.assert_not_called()

    def test_runout_with_runout_lane_hands_off(self):
        lane = _make_lane(printing=True, tool_loaded=True, runout_lane="lane2")
        lane.ace_update_presence(False)
        lane._perform_infinite_runout.assert_called_once()

    def test_loaded_lane_not_printing_keeps_tool_state(self):
        lane = _make_lane(printing=False, tool_loaded=True)
        lane.ace_update_presence(False)
        lane.set_unloaded.assert_not_called()
        lane._perform_pause_runout.assert_not_called()


class TestLoadRefusals:
    def test_refuses_when_toolhead_occupied(self):
        lane = _make_lane()
        lane.extruder_obj.tool_start_state = True
        lane.cmd_AFC_ACEPRO_TOOL_LOAD(None)
        assert "toolhead sensor already" in lane.afc.error.handle_lane_failure.call_args.args[1]
        lane.unit_obj.run_move.assert_not_called()

    def test_refuses_empty_slot(self):
        lane = _make_lane(index=3)
        lane.cmd_AFC_ACEPRO_TOOL_LOAD(None)
        assert "no spool in slot 3" in lane.afc.error.handle_lane_failure.call_args.args[1]

    def test_refuses_with_cutter_down(self):
        lane = _make_lane()
        lane.unit_obj.cutter_sensor_pin = "nozzle_mcu:PA8"
        lane.unit_obj.cutter_sensor_state = True
        lane.cmd_AFC_ACEPRO_TOOL_LOAD(None)
        assert "cutter" in lane.afc.error.handle_lane_failure.call_args.args[1]

    def test_refuses_when_splitter_occupied(self):
        lane = _make_lane()
        lane.hub_obj = MagicMock(switch_pin="PC15", state=True)
        lane.cmd_AFC_ACEPRO_TOOL_LOAD(None)
        assert "splitter" in lane.afc.error.handle_lane_failure.call_args.args[1]

    def test_virtual_hub_not_checked(self):
        lane = _make_lane()
        lane.hub_obj = MagicMock(switch_pin="virtual", state=True)
        assert lane._hub_sensor() is None

    def test_offline_refuses(self):
        lane = _make_lane()
        lane.unit_obj.online = False
        lane.unit_obj.connect = MagicMock(return_value=False)
        lane.unit_obj.transport.last_error = "open failed"
        lane.cmd_AFC_ACEPRO_TOOL_LOAD(None)
        assert "offline" in lane.afc.error.handle_lane_failure.call_args.args[1]


class TestBowden:
    def test_lane_override(self):
        lane = _make_lane()
        assert lane.bowden == 900.0
        lane.ace_bowden_length = 1100.0
        assert lane.bowden == 1100.0

    def test_slot_is_zero_based(self):
        assert _make_lane(index=1).slot == 0
        assert _make_lane(index=4).slot == 3


class TestCut:
    def test_unverified_cut_without_switch(self):
        lane = _make_lane()
        assert lane._run_cutter_sequence() is None
        lane.afc.gcode.run_script_from_command.assert_called_once_with("AFC_CUT EXTRUDER=extruder")

    def test_switch_never_fired_fails(self):
        lane = _make_lane()
        lane.unit_obj.cutter_sensor_pin = "nozzle_mcu:PA8"
        lane.afc.toolhead = None
        reason = lane._run_cutter_sequence()
        assert reason and "never fired" in reason
        assert lane.afc.gcode.run_script_from_command.call_count == 2   # retried once

    def test_switch_fired_passes(self):
        lane = _make_lane()
        unit = lane.unit_obj
        unit.cutter_sensor_pin = "nozzle_mcu:PA8"
        lane.afc.toolhead = None

        def cut(cmd):
            unit.cutter_trigger_count += 1
        lane.afc.gcode.run_script_from_command = MagicMock(side_effect=cut)
        assert lane._run_cutter_sequence() is None

    def test_no_cut_configured(self):
        lane = _make_lane()
        lane.afc.tool_cut = False
        assert lane._run_cutter_sequence() is None
        lane.afc.gcode.run_script_from_command.assert_not_called()


def _make_unit(**over):
    unit = afcACEPRO.__new__(afcACEPRO)
    unit.reactor = AdvancingReactor()
    unit.logger = MockLogger()
    unit.name = "ACE_1"
    unit.lanes = {}
    unit.afc = MockAFC()

    class Cfg:
        def __init__(self, values):
            self.values = values

        def get(self, k, d=None):
            return self.values.get(k, d)

        def getint(self, k, d=None, **kw):
            return int(self.values.get(k, d))

        def getfloat(self, k, d=None, **kw):
            v = self.values.get(k, d)
            return None if v is None else float(v)

        def getboolean(self, k, d=None):
            return bool(self.values.get(k, d))
    values = {"serial": "/dev/null"}
    values.update(over)
    unit._load_ace_config(Cfg(values))
    unit._init_ace_runtime()
    unit._event_timer = None
    return unit


class TestUnitDebounce:
    def _lane(self, unit, index):
        lane = MagicMock()
        lane.name = "lane{}".format(index)
        lane.slot = index - 1
        unit.lanes[lane.name] = lane
        return lane

    @staticmethod
    def _st(*slots, unit="ready"):
        return {"status": unit, "slots": [{"status": s} for s in slots]}

    def test_first_reading_taken_at_once(self):
        unit = _make_unit()
        lane = self._lane(unit, 1)
        unit._mirror_presence(self._st("ready"))
        lane.ace_update_presence.assert_called_once_with(True)

    def test_change_needs_confirmation(self):
        unit = _make_unit(presence_confirm=2)
        lane = self._lane(unit, 1)
        unit._mirror_presence(self._st("ready"))
        unit._mirror_presence(self._st("empty"))
        assert lane.ace_update_presence.call_count == 1
        unit._mirror_presence(self._st("empty"))
        lane.ace_update_presence.assert_called_with(False)

    def test_single_flicker_ignored(self):
        unit = _make_unit(presence_confirm=2)
        lane = self._lane(unit, 1)
        for st in ("ready", "empty", "ready", "empty", "ready"):
            unit._mirror_presence(self._st(st))
        assert lane.ace_update_presence.call_count == 1

    def test_empty_while_busy_never_counts(self):
        unit = _make_unit(presence_confirm=1)
        lane = self._lane(unit, 2)
        unit._mirror_presence(self._st("feeding", "ready"))
        for _ in range(5):
            unit._mirror_presence(self._st("feeding", "empty", unit="busy"))
        assert lane.ace_update_presence.call_count == 1


class TestUnitConfig:
    def test_speeds_capped(self):
        unit = _make_unit(feed_speed=250, retract_speed=0.2)
        assert unit.feed_speed == 100.0 and unit.retract_speed == 1.0

    def test_retract_steps(self):
        unit = _make_unit(unload_retract_steps="12:300, -8:1800")
        assert unit.unload_retract_steps == [(12.0, 300.0), (8.0, 1800.0)]

    def test_tag_applied_with_afc_commands(self):
        unit = _make_unit()
        lane = MagicMock(spool_id=None)
        lane.name = "lane1"
        cmds = unit.apply_tag(lane, {"material": "PLA", "color": "ff0000"}, MagicMock())
        assert cmds == ["SET_COLOR LANE=lane1 COLOR=ff0000", "SET_MATERIAL LANE=lane1 MATERIAL=PLA"]

    def test_tag_skipped_for_spoolman_lane(self):
        unit = _make_unit()
        lane = MagicMock(spool_id=12)
        lane.name = "lane1"
        run = MagicMock()
        assert unit.apply_tag(lane, {"material": "PLA", "color": "ff0000"}, run) == []
        run.assert_not_called()
