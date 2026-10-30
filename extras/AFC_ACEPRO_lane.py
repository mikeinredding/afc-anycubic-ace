# AFCProject Automated Filament Changer Software
#
# Lane for an Anycubic ACE Pro slot. The ACE owns the slot motor and sensor,
# so this lane has no prep/load pins: presence comes from the unit's status
# poll, and tool load/unload run as custom commands that sequence the ACE and
# the toolhead extruder together.
#
# Load:   wait ready -> feed bowden_length - approach_length fast -> slow
#         approach until the toolhead sensor -> the ACE pushes grab_length
#         while the extruder turns (the gears take it) -> feed assist on ->
#         extruder pushes the rest of tool_stn.
# Unload: cut (verified with the cutter switch if wired) -> feed assist off
#         -> extruder retract out of the gears -> the ACE unwinds
#         bowden_length, leaving the tip staged at its outlet; the toolhead
#         sensor must clear on the way.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import traceback
from typing import Any, Optional, Tuple

import configfile

CONFIG_ERROR = getattr(configfile, "error", Exception)

try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    raise CONFIG_ERROR(
        "Error when trying to import AFC_utils.ERROR_STR\n{}".format(traceback.format_exc())
    )

try:
    from extras.AFC_lane import AFCLane, AFCLaneState, AFCMoveWarning
except Exception:
    raise CONFIG_ERROR(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))


class AFCACEPROLane(AFCLane):
    cmd_AFC_ACEPRO_TOOL_LOAD_help = "Load an ACE Pro lane into the toolhead"
    cmd_AFC_ACEPRO_TOOL_UNLOAD_help = "Cut and unload an ACE Pro lane back to the ACE"
    cmd_AFC_ACEPRO_EJECT_help = "Pull an ACE Pro lane's filament back into the ACE"
    cmd_AFC_ACEPRO_CALIBRATE_help = "Measure an ACE Pro lane's bowden_length"
    cmd_AFC_ACEPRO_FEED_ASSIST_help = "Turn an ACE Pro lane's feed assist on or off"

    def __init__(self, config):
        super().__init__(config)
        if self.prep is not None or self.load is not None:
            raise CONFIG_ERROR(
                "[{}]: ACE lanes take spool presence from the ACE; remove prep/load pins".format(
                    config.get_name()))
        if self.index < 1 or self.index > 4:
            raise CONFIG_ERROR(
                "[{}]: unit must be '<ace_unit>:<1-4>', got index {}".format(
                    config.get_name(), self.index))
        # ACE outlet to the toolhead sensor for this lane; defaults to the unit's
        self.ace_bowden_length: Optional[float] = config.getfloat(
            "bowden_length", None, minval=50.)

        self._load_state = False
        self._ace_present_last: Optional[bool] = None
        self.assist_enabled = True
        self.last_load_report: dict = {}

        if self.custom_load_cmd is None:
            self.custom_load_cmd = "AFC_ACEPRO_TOOL_LOAD LANE={}".format(self.name)
        if self.custom_unload_cmd is None:
            self.custom_unload_cmd = "AFC_ACEPRO_TOOL_UNLOAD LANE={}".format(self.name)

        for cmd in ("TOOL_LOAD", "TOOL_UNLOAD", "EJECT", "CALIBRATE", "FEED_ASSIST"):
            name = "AFC_ACEPRO_" + cmd
            self.afc.gcode.register_mux_command(name, "LANE", self.name,
                                                getattr(self, "cmd_" + name),
                                                desc=getattr(self, "cmd_{}_help".format(name)))
        self._register_frontend_compat_aliases()

    # -- plumbing ------------------------------------------------------------

    def _register_frontend_compat_aliases(self) -> None:
        """Expose this lane as 'AFC_lane <name>' / 'AFC_stepper <name>' so
        Mainsail, Fluidd and GrumpyScreen find it like any other lane."""
        for alias in ("AFC_lane {}".format(self.name), "AFC_stepper {}".format(self.name)):
            if alias == self.fullname:
                continue
            existing = self.printer.lookup_object(alias, None)
            if existing not in (None, self):
                continue
            add_object = getattr(self.printer, "add_object", None)
            if callable(add_object):
                try:
                    add_object(alias, self)
                    continue
                except Exception:
                    pass
            for registry_name in ("_objects", "objects"):
                registry = getattr(self.printer, registry_name, None)
                if isinstance(registry, dict):
                    registry.setdefault(alias, self)
                    break

    @property
    def ace(self):
        return self.unit_obj

    @property
    def slot(self) -> int:
        """The ACE's 0-based slot index."""
        return self.index - 1

    @property
    def bowden(self) -> float:
        own = getattr(self, "ace_bowden_length", None)
        return own if own else self.ace.bowden_length

    def _pause(self, seconds: float) -> None:
        if seconds > 0:
            self.reactor.pause(self.reactor.monotonic() + seconds)

    def _e_move(self, distance: float, speed_mm_s: float, label: str) -> None:
        """Extruder-only move that leaves the user's M82/M83 mode alone."""
        self.afc.move_e_pos(distance, speed_mm_s, "ACE {}".format(label), wait_tool=True)

    def _toolhead_sensor(self) -> bool:
        return bool(self.get_toolhead_pre_sensor_state())

    def _hub_sensor(self) -> Optional[bool]:
        """The shared splitter sensor (a real [AFC_hub] switch_pin), or None
        when the hub is virtual."""
        hub = getattr(self, "hub_obj", None)
        if hub is None or str(getattr(hub, "switch_pin", "virtual")).lower() == "virtual":
            return None
        try:
            return bool(hub.state)
        except Exception:
            return None

    def _cutter_engaged(self) -> bool:
        unit = self.ace
        return bool(unit.cutter_sensor_pin and unit.cutter_sensor_state)

    def _trace(self, ev: str, **fields: Any) -> None:
        trace = getattr(self.ace, "trace", None)
        if trace is not None:
            trace.event(ev, lane=self.name, slot=self.slot + 1, **fields)

    def _fail(self, message: str) -> None:
        self._trace("fail", message=message, toolhead_sensor=self._toolhead_sensor())
        self.logger.error("ACE {} (slot {}): {}".format(self.name, self.slot + 1, message))
        self.afc.error.handle_lane_failure(self, "ACE {}: {}".format(self.name, message))

    # Lanes on an ACE never move filament through AFC's stepper paths
    def move(self, distance, speed, accel, assist_active=False, *args, **kwargs):
        self.logger.debug("ACE {}: direct lane moves are not supported".format(self.name))

    def move_advanced(self, distance, speed_mode, assist_active=None):
        self.logger.debug("ACE {}: direct lane moves are not supported".format(self.name))

    def move_to(self, distance, speed_mode, endstop=None, assist_active=None, use_homing=True):
        return False, 0, AFCMoveWarning.ERROR

    def do_enable(self, enable):
        return

    # -- presence from the ACE poll -------------------------------------------

    def ace_update_presence(self, present: bool) -> None:
        """Called (from the unit's event timer) with a debounced slot reading."""
        if present == self._ace_present_last:
            return
        previous = self._ace_present_last
        self._ace_present_last = present
        self.prep_state = present
        self.loaded_to_hub = present

        if not getattr(self, "_afc_prep_done", False):
            self.afc.save_vars()
            return
        if previous is None:
            # PREP ran while the ACE was offline: this is the first real
            # reading, not an insert or a runout.
            msg = self.unit_obj.apply_lane_state(self, present)
            if not present and not self.tool_loaded:
                self.set_unloaded()
            self.logger.info("ACE {}: ACE online, slot {}: {}".format(
                self.name, self.slot + 1, msg))
            self.afc.save_vars()
            return

        if present:
            self.logger.info("ACE {}: spool detected in slot {}".format(self.name, self.slot + 1))
            if self.status not in (AFCLaneState.LOADED, AFCLaneState.TOOLED,
                                   AFCLaneState.TOOL_LOADED):
                self.set_loaded()
            self.unit_obj.on_filament_insert(self)
        else:
            self.logger.info("ACE {}: slot {} is empty".format(self.name, self.slot + 1))
            self.unit_obj.on_filament_remove(self)
            if self.tool_loaded and self.afc.function.is_printing():
                self._trace("runout", runout_lane=self.runout_lane)
                fila = getattr(self, "fila_load", None)
                if fila is not None and not fila.runout_helper.sensor_enabled:
                    self.logger.warning("ACE runout on {} ignored, sensor disabled".format(
                        self.name))
                elif self.runout_lane is not None:
                    self._perform_infinite_runout()
                else:
                    self._perform_pause_runout()
            elif not self.tool_loaded:
                self.set_unloaded()
        self.afc.save_vars()

    # -- tool load -------------------------------------------------------------

    def _retreat(self, distance: float) -> None:
        """
        Pull a partial feed back so the shared path is clear for the next
        attempt (or for another lane). The fed distance is an estimate that
        can run long (time spent stalled counts as moving), and unwinding past
        the staging point pulls the tip out of the slot sensor. With a splitter
        sensor, unwind until it clears plus retreat_margin; without one, unwind
        the estimate.
        """
        if distance <= 1.0:
            return
        unit = self.ace
        margin = None
        if self._hub_sensor() is not None:
            res = unit.run_move("unwind_filament", self.slot, distance + 50.0,
                                unit.retract_speed, stop_when=lambda: not self._hub_sensor())
            if res["result"] == "stopped":
                margin = unit.run_move("unwind_filament", self.slot, unit.retreat_margin,
                                       unit.retract_speed)["result"]
        else:
            res = unit.run_move("unwind_filament", self.slot, distance, unit.retract_speed)
        self._trace("retreat", mm=round(distance, 1), result=res["result"], margin=margin,
                    unwound=round(res["moved"], 1), hub_sensor=self._hub_sensor())

    def _load_attempt(self, attempt: int) -> Tuple[bool, str]:
        unit = self.ace
        fed = 0.0
        sensor = self._toolhead_sensor
        bowden = self.bowden
        fast = max(bowden - unit.approach_length, 0.0)

        if fast >= 1.0:
            res = unit.run_move("feed_filament", self.slot, fast, unit.feed_speed,
                                stop_when=sensor)
            fed += res["moved"]
            self.last_load_report["fast"] = res["result"]
            if res["result"] in ("refused", "no_start"):
                return False, "fast feed {}: {}".format(res["result"], res["detail"])
            if res["result"] == "error" and not sensor():
                # feed_error: the ACE's own slip check. Retry from here slowly.
                self.logger.warning("ACE {}: fast feed reported {}".format(
                    self.name, res["error"]))
            if res["result"] == "timeout":
                self._retreat(fed)
                return False, "fast feed {}".format(res["detail"])

        if not sensor():
            remaining = max(bowden - fed, 0.0) + unit.approach_extra
            res = unit.run_move("feed_filament", self.slot, remaining, unit.approach_speed,
                                stop_when=sensor)
            fed += res["moved"]
            self.last_load_report["approach"] = res["result"]
            if not sensor():
                self._retreat(fed)
                why = res["error"] or res["detail"]
                return False, "filament did not reach the toolhead sensor after about {:.0f}mm ({})".format(
                    fed, why)

        self.last_load_report["fed_to_sensor"] = round(fed, 1)
        self._trace("at_sensor", fed=round(fed, 1), attempt=attempt + 1)

        # Hand-off: the ACE pushes into the gears while the extruder turns.
        grabbed = 0.0
        if unit.grab_length > 0:
            refused = unit._send_move("feed_filament", self.slot, unit.grab_length + 5.0,
                                      unit.grab_speed)
            if refused:
                self.logger.warning("ACE {}: grab feed {}".format(self.name, refused))
            started = self.reactor.monotonic()
            self._e_move(unit.grab_length, unit.grab_speed, "grab")
            grabbed = unit.grab_length
            unit.stop_move("stop_feed_filament", self.slot, started)
        if unit.feed_assist and self.assist_enabled:
            unit.start_assist(self.slot)
        if not sensor():
            return False, "toolhead sensor cleared during the hand-off"
        self.last_load_report["grab"] = grabbed
        return True, "loaded"

    def cmd_AFC_ACEPRO_TOOL_LOAD(self, gcmd: Any) -> None:
        """
        Load this ACE lane to the nozzle. AFC calls this through custom_load_cmd
        during a tool change; it can also be run by hand during bring-up.

        Usage
        -----
        `AFC_ACEPRO_TOOL_LOAD LANE=<lane>`
        """
        unit = self.ace
        started = self.reactor.monotonic()
        self._trace("load_start", toolhead_sensor=self._toolhead_sensor(),
                    hub_sensor=self._hub_sensor())
        self.select_lane()
        self.afc._check_extruder_temp(self)
        self.last_load_report = {"slot": self.slot + 1}

        if not unit.online and not unit.connect():
            self._fail("ACE is offline ({})".format(unit.transport.last_error or "no reply"))
            return
        if self._cutter_engaged():
            self._fail("cutter sensor is engaged; reset the cutter before loading")
            return
        if self._toolhead_sensor():
            self._fail("toolhead sensor already sees filament; unload the other lane first")
            return
        if self._hub_sensor():
            self._fail("splitter (hub) sensor already sees filament; clear the shared tube first")
            return
        status = unit.get_status_now()
        if status is not None and unit.slot_presence(status, self.slot) is False:
            self._fail("no spool in slot {}".format(self.slot + 1))
            return

        if self.afc.park and self.afc.park_cmd:
            self.afc.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(self.afc.park_cmd, self.extruder_obj.name))

        attempts_used = 0
        ok, detail = False, "not attempted"
        with unit.operation():
            unit.stop_all_assist()
            for attempt in range(unit.load_attempts):
                attempts_used = attempt + 1
                self.logger.info("ACE {}: loading slot {}, try {}/{}".format(
                    self.name, self.slot + 1, attempt + 1, unit.load_attempts))
                try:
                    ok, detail = self._load_attempt(attempt)
                except Exception as e:
                    ok, detail = False, "error: {}".format(e)
                    self.logger.debug(traceback.format_exc())
                if ok:
                    break
                self._trace("load_attempt_failed", attempt=attempt + 1, detail=detail,
                            toolhead_sensor=self._toolhead_sensor())
                self.logger.warning("ACE {}: load attempt {} failed: {}".format(
                    self.name, attempt + 1, detail))
                if self._toolhead_sensor() or not unit.online:
                    break

        self.last_load_report["result"] = detail
        if not ok:
            self._trace("load_end", ok=False, detail=detail, attempts=attempts_used,
                        seconds=round(self.reactor.monotonic() - started, 2))
            self._fail("load failed after {} attempt(s): {}".format(attempts_used, detail))
            return

        remaining = self.extruder_obj.tool_stn - self.last_load_report.get("grab", 0.0)
        if remaining > 0:
            self.afc.move_e_pos(remaining, self.extruder_obj.tool_load_speed,
                                "ACE tool_stn", wait_tool=True)
        self._load_state = True          # our filament now occupies the shared path
        self._mark_tool_state(loaded=True)
        self._trace("load_end", ok=True, detail=detail, attempts=attempts_used,
                    tool_stn_push=round(max(remaining, 0), 2),
                    seconds=round(self.reactor.monotonic() - started, 2))

    # -- tool unload -----------------------------------------------------------

    def _run_cutter_sequence(self) -> Optional[str]:
        """
        Run AFC's cut macro. With a cutter switch configured, verify it: the
        switch must have fired during the cut (blade reached the end of its
        travel, if cutter_require_trigger) and be released afterwards. Retries
        the cut; returns None on success or the reason it failed.
        """
        if not self.afc.tool_cut:
            return None
        unit = self.ace
        reason = None
        for attempt in range(2):
            triggers_before = unit.cutter_trigger_count
            self.extruder_obj.estats.increase_cut_total()
            self.afc.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(self.afc.tool_cut_cmd, self.extruder_obj.name))
            if not unit.cutter_sensor_pin:
                self._trace("cut", attempt=attempt + 1, verified=False)
                return None
            toolhead = getattr(self.afc, "toolhead", None)
            if toolhead is not None and hasattr(toolhead, "wait_moves"):
                toolhead.wait_moves()
            start = self.reactor.monotonic()
            self._pause(0.1)
            while self._cutter_engaged():
                if self.reactor.monotonic() - start > unit.cutter_wait_time:
                    break
                self._pause(0.01)
            released = not self._cutter_engaged()
            fired = unit.cutter_trigger_count > triggers_before
            self._trace("cut", attempt=attempt + 1, verified=True, fired=fired,
                        released=released, seconds=round(self.reactor.monotonic() - start, 2))
            if not released:
                reason = "cutter sensor did not release after cutting"
            elif not fired and unit.cutter_require_trigger:
                reason = ("cutter switch never fired during the cut (blade did not reach "
                          "the end of its travel; check pin_loc_xy / cut_move_dist)")
            else:
                return None
            self.logger.warning("ACE {}: cut {} not confirmed: {}".format(
                self.name, attempt + 1, reason))
        return reason

    def _run_post_cut_macros(self) -> None:
        if self.afc.park and self.afc.park_cmd:
            self.afc.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(self.afc.park_cmd, self.extruder_obj.name))
        if self.afc.form_tip:
            if self.afc.form_tip_cmd == "AFC":
                self.printer.lookup_object("AFC_form_tip").tip_form()
            else:
                self.afc.gcode.run_script_from_command(self.afc.form_tip_cmd)

    def _retract_to_ace(self) -> Tuple[bool, str]:
        unit = self.ace
        unit.stop_all_assist()
        # Out of the gears first: the ACE can't pull filament the gears hold
        for i, (dist, speed) in enumerate(unit.unload_retract_steps):
            self._e_move(-dist, speed / 60.0, "unload retract {}".format(i + 1))
            self._trace("retract_step", step=i + 1, mm=dist, speed=speed,
                        toolhead_sensor=self._toolhead_sensor())

        length = self.bowden + unit.unload_extra
        res = unit.run_move("unwind_filament", self.slot, length, unit.retract_speed,
                            watch=lambda: not self._toolhead_sensor())
        self.last_load_report["unwind"] = res["result"]
        cleared_at = res["watch_at"]
        if self._toolhead_sensor():
            return False, "toolhead sensor still triggered after unwinding {:.0f}mm ({})".format(
                res["moved"], res["error"] or res["detail"])
        if cleared_at is not None and cleared_at > unit.unload_clear_length:
            self.logger.warning("ACE {}: toolhead sensor only cleared after {:.0f}mm".format(
                self.name, cleared_at))
        if res["result"] not in ("done", "stopped"):
            return False, "unwind {}: {}".format(res["result"], res["error"] or res["detail"])
        hub = self._hub_sensor()
        if hub:
            return False, "splitter (hub) sensor still sees filament after the unwind"
        return True, "unwound {:.0f}mm in {:.1f}s".format(res["moved"], res["seconds"])

    def cmd_AFC_ACEPRO_TOOL_UNLOAD(self, gcmd: Any) -> None:
        """
        Cut and unload this ACE lane back to the ACE outlet. AFC calls this
        through custom_unload_cmd during a tool change.

        Usage
        -----
        `AFC_ACEPRO_TOOL_UNLOAD LANE=<lane>`
        """
        unit = self.ace
        started = self.reactor.monotonic()
        self._trace("unload_start", toolhead_sensor=self._toolhead_sensor())
        self.select_lane()
        self.afc._check_extruder_temp(self)
        self.disable_buffer()

        if not unit.online and not unit.connect():
            self._fail("ACE is offline ({})".format(unit.transport.last_error or "no reply"))
            return
        if self._cutter_engaged():
            self._fail("cutter sensor is engaged before cutting; check the cutter lever")
            return
        cut_problem = self._run_cutter_sequence()
        if cut_problem:
            self._fail(cut_problem)
            return
        self._run_post_cut_macros()

        with unit.operation():
            try:
                ok, detail = self._retract_to_ace()
            except Exception as e:
                ok, detail = False, "error: {}".format(e)
                self.logger.debug(traceback.format_exc())

        self.logger.info("ACE {}: unload {}".format(self.name, detail))
        self._trace("unload_end", ok=ok, detail=detail,
                    seconds=round(self.reactor.monotonic() - started, 2))
        if not ok:
            self._fail("unload failed: {}".format(detail))
            return
        self._load_state = False         # back at the ACE outlet, out of the shared path
        self._mark_tool_state(loaded=False)

    def _mark_tool_state(self, loaded: bool) -> None:
        """
        Record the tool state in AFC. AFC's own TOOL_LOAD/TOOL_UNLOAD flow
        does this after our custom command returns, but running the command
        by hand would otherwise leave AFC thinking nothing (or the old lane)
        is in the toolhead. Setting it twice is harmless.
        """
        try:
            if loaded:
                self.set_tool_loaded(normal_toolchange=True)
            else:
                self.set_tool_unloaded(normal_toolchange=True)
            self.afc.save_vars()
        except Exception as e:
            self.logger.warning("ACE {}: could not record tool state: {}".format(self.name, e))

    # -- other commands ----------------------------------------------------------

    def cmd_AFC_ACEPRO_EJECT(self, gcmd: Any) -> None:
        """
        Pull this lane's filament back into the ACE (eject_length), so the
        spool can be taken out. Not for filament in the toolhead.

        Usage
        -----
        `AFC_ACEPRO_EJECT LANE=<lane>`
        """
        unit = self.ace
        if not unit.online and not unit.connect():
            raise gcmd.error("ACE {}: offline".format(self.name))
        ok, detail = unit.eject(self)
        gcmd.respond_info("ACE {} (slot {}) eject: {}".format(self.name, self.slot + 1, detail))
        if not ok:
            raise gcmd.error("ACE {}: eject did not finish: {}".format(self.name, detail))

    def cmd_AFC_ACEPRO_CALIBRATE(self, gcmd: Any) -> None:
        """
        Measure bowden_length (ACE outlet to the toolhead sensor) by feeding
        in calibration_step pieces until the toolhead sensor triggers, then
        unwinding the same distance. The toolhead must be empty.

        Usage
        -----
        `AFC_ACEPRO_CALIBRATE LANE=<lane>`
        """
        unit = self.ace
        if not unit.online and not unit.connect():
            raise gcmd.error("ACE {}: offline".format(self.name))
        if self._toolhead_sensor():
            raise gcmd.error("ACE {}: toolhead sensor already triggered; unload first".format(
                self.name))
        if self.tool_loaded:
            raise gcmd.error("ACE {}: lane is loaded in the toolhead".format(self.name))
        sensor = self._toolhead_sensor
        total = 0.0
        last = None
        with unit.operation():
            unit.stop_all_assist()
            while total < unit.calibration_max and not sensor():
                step = min(unit.calibration_step, unit.calibration_max - total)
                last = unit.run_move("feed_filament", self.slot, step, unit.approach_speed,
                                     stop_when=sensor)
                total += last["moved"]
                if last["result"] not in ("done", "stopped"):
                    break
            reached = sensor()
            back = unit.run_move("unwind_filament", self.slot, total + 10.0, unit.retract_speed)
        self._trace("calibrate", reached=reached, mm=round(total, 1),
                    unwind=back["result"])
        if not reached:
            raise gcmd.error("ACE {}: toolhead sensor never triggered after {:.0f}mm ({})".format(
                self.name, total, (last or {}).get("detail", "")))
        # Round down to 5 mm: the slow approach covers the rest
        measured = int(total // 5 * 5)
        gcmd.respond_info(
            "ACE {} (slot {}): toolhead sensor reached after about {:.0f}mm.\n"
            "Set bowden_length: {} in [AFC_ACEPRO {}] (or on [AFC_ACEPRO_lane {}] if "
            "this lane's tube differs), then restart Klipper.".format(
                self.name, self.slot + 1, total, measured, unit.name, self.name))

    def cmd_AFC_ACEPRO_FEED_ASSIST(self, gcmd: Any) -> None:
        """
        Usage
        -----
        `AFC_ACEPRO_FEED_ASSIST LANE=<lane> ENABLE=<0|1>`
        """
        enable = gcmd.get_int("ENABLE", minval=0, maxval=1)
        unit = self.ace
        self.assist_enabled = bool(enable)
        if enable:
            ok = unit.start_assist(self.slot)
        else:
            ok = unit.stop_assist(self.slot) if unit.assist_slot == self.slot else True
        gcmd.respond_info("ACE {}: feed assist {}{}".format(
            self.name, "on" if enable else "off", "" if ok else " (ACE did not confirm)"))

    def get_status(self, eventtime=None, save_to_file=False):
        response = super().get_status(eventtime=eventtime, save_to_file=save_to_file)
        if not response or save_to_file:
            return response
        response["ace_slot"] = self.slot + 1
        response["ace_present"] = self._ace_present_last
        response["ace_last_load"] = dict(self.last_load_report)
        response["ace_tag"] = getattr(self.ace, "lane_tags", {}).get(self.name)
        return response


def load_config_prefix(config):
    return AFCACEPROLane(config)
