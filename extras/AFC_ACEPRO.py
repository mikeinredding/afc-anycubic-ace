# AFCProject Automated Filament Changer Software
#
# Anycubic ACE Pro (first generation) unit for AFC.
#
# The ACE is a closed box: it moves filament itself and reports its own slot
# sensors over USB (CDC serial, JSON-RPC in binary frames). Klipper never drives
# an ACE motor directly. This unit:
#   - owns the serial link to one ACE (non-blocking, reactor-driven),
#   - polls the ACE (which also keeps it from dropping off USB: it resets its
#     link after ~3 s without a complete frame),
#   - mirrors per-slot spool presence and RFID tags into AFC lanes,
#   - runs the feed / unwind / feed-assist operations AFC_ACEPRO_lane uses.
#
# STATUS: experimental. Follow docs/ACEPRO.md's staged test plan.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import collections
import errno
import json
import os
import queue
import termios
import threading
import time
import traceback
from typing import TYPE_CHECKING, Any, Callable, Deque, Dict, List, Optional, Tuple

import configfile

CONFIG_ERROR = getattr(configfile, "error", Exception)

try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    raise CONFIG_ERROR(
        "Error when trying to import AFC_utils.ERROR_STR\n{}".format(traceback.format_exc())
    )

try:
    from extras.AFC_unit import afcUnit
except Exception:
    raise CONFIG_ERROR(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try:
    from extras.AFC_lane import AFCMoveWarning
except Exception:
    raise CONFIG_ERROR(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try:
    from extras.AFC_respond import AFCprompt
except Exception:
    raise CONFIG_ERROR(ERROR_STR.format(import_lib="AFC_respond", trace=traceback.format_exc()))

try:
    from extras import AFC_ACEPRO_protocol as P
except Exception:
    raise CONFIG_ERROR(ERROR_STR.format(import_lib="AFC_ACEPRO_protocol",
                                        trace=traceback.format_exc()))

if TYPE_CHECKING:
    from gcode import GCodeCommand
    from extras.AFC_lane import AFCLane


# How often a running move checks the toolhead sensor (s). 20 ms is 2 mm at
# 100 mm/s, so a feed stops close to the sensor.
SENSOR_POLL = 0.02
# The ACE ignores a stop that arrives within ~250 ms of the move it ends
# (AFC ACE_series), so a stop is never sent earlier than this after the start
# and is confirmed against the status.
STOP_MIN_AFTER_START = 0.3
STOP_CONFIRM_TRIES = 3


class ACETrace:
    """
    Optional JSON-lines trace of what the unit does with the ACE: requests and
    replies (with latency), timeouts, connection changes, and load / unload /
    cut events with timings. For bring-up and tuning from real prints.

    Writes happen on a background thread so a slow SD/eMMC write can never
    stall Klipper's reactor; the reactor side only appends to a bounded queue.
    The file rotates at max_bytes, keeping `backups` older copies.
    """

    QUEUE_SIZE = 20000

    def __init__(self, path: Optional[str], max_bytes: int, backups: int,
                 logger: Any) -> None:
        self.path = os.path.expanduser(path) if path else None
        self.max_bytes = max_bytes
        self.backups = backups
        self.logger = logger
        self.enabled = False
        self.dropped = 0
        self.written = 0
        self.last_error: Optional[str] = None
        self._size = 0
        self._queue: "queue.Queue[Optional[str]]" = queue.Queue(maxsize=self.QUEUE_SIZE)
        self._thread: Optional[threading.Thread] = None
        self._t0 = time.monotonic()

    def start(self) -> bool:
        if not self.path:
            return False
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run, name="acepro-trace", daemon=True)
            self._thread.start()
        self.enabled = True
        return True

    def stop(self, timeout: float = 2.0) -> None:
        self.enabled = False
        if self._thread is not None and self._thread.is_alive():
            try:
                self._queue.put(None, timeout=timeout)
            except queue.Full:
                pass
            self._thread.join(timeout)
        self._thread = None

    def event(self, ev: str, **fields: Any) -> None:
        """Record one event. Cheap and non-blocking; safe from the reactor."""
        if not self.enabled:
            return
        record = {"ts": round(time.time(), 4), "t": round(time.monotonic() - self._t0, 4),
                  "ev": ev}
        record.update(fields)
        try:
            self._queue.put_nowait(json.dumps(record, separators=(",", ":"), default=str))
        except (queue.Full, TypeError, ValueError):
            self.dropped += 1

    def _open(self):
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        handle = open(self.path, "a", encoding="utf-8")
        self._size = os.path.getsize(self.path)
        return handle

    def _rotate(self, handle):
        handle.close()
        for i in range(self.backups, 0, -1):
            src = self.path if i == 1 else "{}.{}".format(self.path, i - 1)
            dst = "{}.{}".format(self.path, i)
            if os.path.exists(src):
                os.replace(src, dst)
        if self.backups == 0 and os.path.exists(self.path):
            os.remove(self.path)
        return self._open()

    def _run(self) -> None:
        try:
            handle = self._open()
        except OSError as e:
            self.last_error = "cannot open trace file: {}".format(e)
            self.enabled = False
            return
        try:
            while True:
                line = self._queue.get()
                if line is None:
                    break
                handle.write(line + "\n")
                self.written += 1
                self._size += len(line.encode("utf-8")) + 1
                if self._size >= self.max_bytes:
                    handle = self._rotate(handle)
                elif self._queue.empty():
                    handle.flush()
        except OSError as e:
            self.last_error = "trace write failed: {}".format(e)
            self.enabled = False
        finally:
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass


class ACETransport:
    """
    Reactor-safe request/response link to one ACE.

    Nothing here blocks the reactor: the port is opened O_NONBLOCK, received
    bytes arrive through reactor.register_fd(), and request() waits on a
    reactor completion. It must be called from a greenlet that may pause
    (gcode handlers and timer callbacks both qualify).

    The ACE handles one request at a time and loses output if a new request
    arrives before it has answered, so a mutex keeps exactly one request in
    flight. Replies carry the request id; a late reply to a request that timed
    out is recognised by its id and dropped.
    """

    def __init__(self, reactor: Any, path: str, baud: int, logger: Any,
                 name: str, debug: bool = False) -> None:
        self.reactor = reactor
        self.path = path
        self.baud = baud
        self.logger = logger
        self.name = name
        self.debug = debug

        self.fd: Optional[int] = None
        self._fd_handle = None
        self._parser = P.FrameParser()
        self._mutex = reactor.mutex()
        self._pending: Optional[Dict[str, Any]] = None
        self._next_id = 0
        self._timed_out_ids: Deque[int] = collections.deque(maxlen=64)
        self.last_error: Optional[str] = None
        self.last_tx = 0.0
        self.last_rx = 0.0
        self.tx_count = 0
        self.rx_count = 0
        self.timeouts = 0
        self.late_replies = 0
        self.unsolicited = 0
        self.opens = 0
        self.trace: Optional[ACETrace] = None
        self.on_unsolicited: Optional[Callable[[P.Reply], None]] = None
        self._traced_crc_errors = 0
        self._empty_reads = 0
        self._opened_node = ""

    def _trace(self, ev: str, **fields: Any) -> None:
        if self.trace is not None:
            self.trace.event(ev, **fields)

    # -- port management ---------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self.fd is not None

    def open(self) -> bool:
        if self.fd is not None:
            return True
        try:
            fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            self.last_error = "open {} failed: {}".format(self.path, e)
            self._trace("port_error", error=self.last_error)
            return False
        try:
            attrs = termios.tcgetattr(fd)
            baud_const = getattr(termios, "B{}".format(self.baud), termios.B115200)
            attrs[0] = 0                                    # iflag: raw
            attrs[1] = 0                                    # oflag: raw
            cflag = attrs[2]
            cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
            cflag &= ~getattr(termios, "CRTSCTS", 0)
            cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[2] = cflag
            attrs[3] = 0                                    # lflag: raw
            attrs[4] = baud_const
            attrs[5] = baud_const
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except termios.error as e:
            # A pty used for testing may reject some attributes; that's fine.
            self.logger.debug("{}: termios setup warning: {}".format(self.name, e))
        self.fd = fd
        self.opens += 1
        self._empty_reads = 0
        self._opened_node = os.path.realpath(self.path)
        self._parser.reset()
        self._fd_handle = self.reactor.register_fd(fd, self._on_readable)
        self.last_error = None
        self.last_rx = self.reactor.monotonic()
        self._trace("port_open", path=self.path, real=os.path.realpath(self.path))
        return True

    def _device_gone(self) -> bool:
        """The node we opened is no longer what the configured path points to
        (the ACE re-enumerated) or no longer exists."""
        try:
            return (not os.path.exists(self._opened_node)
                    or os.path.realpath(self.path) != self._opened_node)
        except Exception:
            return False

    def close(self) -> None:
        if self._fd_handle is not None:
            try:
                self.reactor.unregister_fd(self._fd_handle)
            except Exception:
                pass
            self._fd_handle = None
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
            self._trace("port_close", error=self.last_error)
        pending = self._pending
        if pending is not None and not pending["completion"].test():
            pending["completion"].complete(None)

    # -- I/O ---------------------------------------------------------------

    def _on_readable(self, eventtime: float) -> None:
        if self.fd is None:
            return
        chunks = []
        while True:
            try:
                chunk = os.read(self.fd, 4096)
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                # The ACE dropped off USB (it does this on its own after ~3 s
                # without a frame), was unplugged or powered off.
                self.last_error = "read error: {}".format(e)
                self.logger.warning("{}: serial {}".format(self.name, self.last_error))
                self.close()
                return
            if not chunk:
                break
            chunks.append(chunk)
        if not chunks:
            # Readable but nothing to read: a hung-up tty (USB gone) does
            # this on every reactor pass. Close once the node has gone or
            # it keeps happening, so the reactor doesn't spin on it.
            self._empty_reads += 1
            if self._device_gone() or self._empty_reads > 50:
                self.last_error = "device went away"
                self.logger.warning("{}: serial {}".format(self.name, self.last_error))
                self.close()
            return
        self._empty_reads = 0
        self.last_rx = eventtime
        for reply in self._parser.feed(b"".join(chunks)):
            self.rx_count += 1
            pending = self._pending
            if pending is not None and reply.id == pending["id"] \
                    and not pending["completion"].test():
                if not pending["quiet"]:
                    self._trace("rx", method=pending["method"], id=reply.id, code=reply.code,
                                msg=reply.msg, result=reply.result,
                                ms=round((self.reactor.monotonic() - pending["sent"]) * 1000, 1))
                pending["completion"].complete(reply)
            elif reply.id is not None and reply.id in self._timed_out_ids:
                self.late_replies += 1
                self._trace("late_reply", id=reply.id, code=reply.code, msg=reply.msg)
            else:
                self.unsolicited += 1
                self._trace("rx_unsolicited", id=reply.id, code=reply.code, msg=reply.msg,
                            result=reply.result)
                if self.on_unsolicited is not None:
                    try:
                        self.on_unsolicited(reply)
                    except Exception:
                        self.logger.debug("{}: unsolicited handler failed\n{}".format(
                            self.name, traceback.format_exc()))
        if self._parser.crc_errors != self._traced_crc_errors:
            self._trace("crc_error", total=self._parser.crc_errors,
                        new=self._parser.crc_errors - self._traced_crc_errors)
            self._traced_crc_errors = self._parser.crc_errors

    def _write_all(self, data: bytes) -> bool:
        view = memoryview(data)
        deadline = self.reactor.monotonic() + 0.5
        while view:
            if self.fd is None:
                return False
            try:
                written = os.write(self.fd, view)
                view = view[written:]
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    self.last_error = "write error: {}".format(e)
                    self.close()
                    return False
                now = self.reactor.monotonic()
                if now > deadline:
                    self.last_error = "write timeout"
                    return False
                self.reactor.pause(now + 0.002)
        return True

    def _encode(self, method: str, params: Optional[Dict[str, Any]]) -> Tuple[int, bytes]:
        for _ in range(8):
            rid = self._next_id
            self._next_id = (self._next_id + 1) % 0x7FFFFFFF
            frame = P.encode_frame(P.encode_payload(rid, method, params))
            if P.frame_is_safe(frame):
                return rid, frame
        return rid, frame

    def request(self, method: str, params: Optional[Dict[str, Any]] = None,
                timeout: float = 2.0, retries: int = 0,
                quiet: bool = False) -> Optional[P.Reply]:
        """
        Send one request and wait (cooperatively) for its reply. Returns the
        reply, or None after all attempts time out or the port is down.
        quiet=True leaves the tx/rx out of the trace (idle status polls);
        timeouts are always traced.
        """
        with self._mutex:
            for attempt in range(retries + 1):
                if self.fd is not None and self._device_gone():
                    self.last_error = "device re-enumerated"
                    self.close()
                if self.fd is None and not self.open():
                    return None
                rid, frame = self._encode(method, params)
                completion = self.reactor.completion()
                self._pending = {"id": rid, "method": method, "quiet": quiet,
                                 "completion": completion, "sent": self.reactor.monotonic()}
                if self.debug:
                    self.logger.debug("{}: TX {} {}".format(self.name, method, params))
                if not self._write_all(frame):
                    self._pending = None
                    self._trace("tx_failed", method=method, error=self.last_error)
                    continue
                self.tx_count += 1
                self.last_tx = self.reactor.monotonic()
                self._pending["sent"] = self.last_tx
                if not quiet:
                    self._trace("tx", method=method, id=rid, params=params, attempt=attempt + 1)
                result = completion.wait(self.reactor.monotonic() + timeout)
                self._pending = None
                if result is not None:
                    if self.debug:
                        self.logger.debug("{}: RX {} code={} msg={} {}".format(
                            self.name, method, result.code, result.msg, result.result))
                    return result
                if self.fd is None:
                    # Port went away while waiting (close() completed us with None)
                    self._trace("port_lost", method=method, id=rid)
                    continue
                self.timeouts += 1
                self._timed_out_ids.append(rid)
                self._trace("timeout", method=method, id=rid, attempt=attempt + 1,
                            of=retries + 1, timeout_s=timeout)
            return None

    @property
    def busy(self) -> bool:
        return self._mutex.test()


class afcACEPRO(afcUnit):
    """
    One Anycubic ACE Pro. Lanes are [AFC_ACEPRO_lane laneN] with unit:
    <name>:1..4 mapping to slots 1..4 (left to right, the ACE's 0..3).
    """

    CALIBRATION_UNSUPPORTED = ("ACE lanes are fed by the ACE itself; use "
                               "AFC_ACEPRO_CALIBRATE LANE=<lane> to measure bowden_length.")

    def __init__(self, config):
        super().__init__(config)
        self.type = config.get("type", "ACE_PRO")
        self.reactor = self.printer.get_reactor()
        self._load_ace_config(config)

        if self.cutter_sensor_pin is not None:
            buttons = self.printer.load_object(config, "buttons")
            buttons.register_buttons([self.cutter_sensor_pin], self._cutter_callback)

        self._init_ace_runtime()

        self.printer.register_event_handler("klippy:ready", self._ace_handle_ready)
        self.printer.register_event_handler("klippy:disconnect", self._ace_handle_disconnect)

        for cmd in ("STATUS", "RECONNECT", "RAW", "TRACE", "DRY", "DRY_STOP"):
            name = "AFC_ACEPRO_" + cmd
            handler = getattr(self, "cmd_" + name)
            self.gcode.register_mux_command(name, "UNIT", self.name, handler,
                                            desc=getattr(self, "cmd_{}_help".format(name)))

    def _load_ace_config(self, config) -> None:
        """Read every ACE-specific option. Kept separate so tests can apply
        the real defaults without constructing a full AFC unit."""
        self.serial_path = config.get("serial")
        self.baud = config.getint("baud", P.DEFAULT_BAUD)
        # The ACE resets its USB link after ~3 s without a complete frame, so
        # it is polled at least this often (and more often during moves).
        self.poll_interval = config.getfloat("poll_interval", 1.0, minval=0.2, maxval=2.0)
        self.request_timeout = config.getfloat("request_timeout", 1.5, minval=0.1)
        self.offline_after = config.getint("offline_after_failures", 3, minval=1)
        self.reconnect_interval = config.getfloat("reconnect_interval", 2.0, minval=0.5)
        # The ACE takes a few seconds to come up after power-on or a USB reset.
        self.boot_wait = config.getfloat("boot_wait", 15.0, minval=0.)
        self.protocol_debug = config.getboolean("protocol_debug", False)

        # --- Trace recording (see docs/ACEPRO.md, "Recording a trace") ---
        self.trace_file = config.get("trace_file", None)
        self.trace_max_size = config.getfloat("trace_max_size", 5.0, minval=0.1)   # MB
        self.trace_backups = config.getint("trace_backups", 1, minval=0, maxval=10)
        self.trace_polls = config.getboolean("trace_polls", False)

        # --- Feeding (mm, mm/s; the ACE caps speeds at 100 mm/s) ---
        # ACE outlet to the toolhead sensor. Measure with AFC_ACEPRO_CALIBRATE;
        # a lane can override it.
        self.bowden_length = config.getfloat("bowden_length", 1000.0, minval=50.)
        self.feed_speed = P.clamp_speed(config.getfloat("feed_speed", 80.0, above=0.))
        self.retract_speed = P.clamp_speed(config.getfloat("retract_speed", 80.0, above=0.))
        # The last stretch before the toolhead sensor runs slowly
        self.approach_length = config.getfloat("approach_length", 150.0, minval=0.)
        self.approach_speed = P.clamp_speed(config.getfloat("approach_speed", 20.0, above=0.))
        # How far past bowden_length the slow approach may go before giving up
        self.approach_extra = config.getfloat("approach_extra", 100.0, minval=0.)
        self.load_attempts = config.getint("load_attempts", 2, minval=1)
        # A failed load is pulled back; with a splitter sensor, this far past
        # the point where it clears
        self.retreat_margin = config.getfloat("retreat_margin", 30.0, minval=0.)
        self.ready_timeout = config.getfloat("ready_timeout", 20.0, minval=1.)
        self.status_interval = config.getfloat("status_interval", 0.3, minval=0.05)
        # A move counts as not started if the slot never reports moving in this time
        self.start_timeout = config.getfloat("start_timeout", 3.0, minval=0.2)
        # Extra time over the move's own duration before it counts as hung
        self.move_timeout_margin = config.getfloat("move_timeout_margin", 15.0, minval=1.)

        # --- Hand-off into the extruder gears ---
        # On the CFS kit toolhead the sensor sits above the gears: the ACE
        # pushes this much more while the extruder turns, so the gears grab.
        self.grab_length = config.getfloat("grab_length", 20.0, minval=0.)
        self.grab_speed = P.clamp_speed(config.getfloat("grab_speed", 8.0, above=0.))
        self.feed_assist = config.getboolean("feed_assist", True)

        # --- Unload ---
        # Extruder retract after the cut, out of the gears before the ACE
        # unwinds. "distance:speed_mm_per_min" pairs. K1 Max CFS kit: 10 mm
        # from the cutter to the gears.
        self.unload_retract_steps = self._parse_steps(config.get(
            "unload_retract_steps", "10:600, 10:1800"))
        # Unwind bowden_length + this, leaving the tip staged at the ACE outlet.
        # Too much pulls it back past the slot sensor and the slot reads empty.
        self.unload_extra = config.getfloat("unload_extra", 0.0)
        # The toolhead sensor must clear within this much unwinding
        self.unload_clear_length = config.getfloat("unload_clear_length", 150.0, minval=10.)
        self.eject_length = config.getfloat("eject_length", 100.0, minval=10.)
        self.cutter_wait_time = config.getfloat("cutter_wait_time", 3.0, minval=0.1)
        self.cutter_require_trigger = config.getboolean("cutter_require_trigger", True)
        self.cutter_sensor_pin = config.get("cutter_sensor_pin", None)

        # --- Presence / RFID ---
        # Consecutive identical slot readings needed to accept a change
        self.presence_confirm = config.getint("presence_confirm", 2, minval=1)
        # Apply the color/material the ACE reads from an Anycubic tag to a
        # freshly inserted spool (only if the lane has no Spoolman spool).
        self.rfid_apply = config.getboolean("rfid_apply", True)
        self.rfid_wait = config.getfloat("rfid_wait", 30.0, minval=1.)

        # --- Dryer ---
        self.max_dryer_temperature = config.getfloat("max_dryer_temperature", 55.0,
                                                     minval=0., maxval=70.)
        self.dryer_fan_speed = config.getint("dryer_fan_speed", 7000, minval=0)

        # --- Calibration ---
        self.calibration_step = config.getfloat("calibration_step", 50.0, minval=5.)
        self.calibration_max = config.getfloat("calibration_max", 3000.0, minval=100.)

    @staticmethod
    def _parse_steps(value: str):
        steps = []
        for item in (value or "").split(","):
            item = item.strip()
            if not item:
                continue
            try:
                dist, speed = item.split(":")
                steps.append((abs(float(dist)), float(speed)))
            except ValueError:
                raise CONFIG_ERROR(
                    "unload_retract_steps: expected 'distance:speed' pairs, got '{}'".format(item))
        return steps

    def _init_ace_runtime(self) -> None:
        self.cutter_sensor_state = False
        self.cutter_trigger_count = 0
        self.transport = ACETransport(self.reactor, self.serial_path, self.baud, self.logger,
                                      "ACE {}".format(self.name), debug=self.protocol_debug)
        self.transport.on_unsolicited = self._on_unsolicited
        self.trace = ACETrace(self.trace_file, int(self.trace_max_size * 1024 * 1024),
                              self.trace_backups, self.logger)
        self.transport.trace = self.trace if self.trace_file else None
        if self.trace_file:
            self.trace.start()
            self.trace.event("trace_start", unit=self.name, serial=self.serial_path,
                             settings=self._trace_settings())

        self.online = False
        self.info: Dict[str, Any] = {}
        self.status: Optional[Dict[str, Any]] = None
        self.status_time = 0.0
        self.assist_slot: Optional[int] = None
        self.last_move_error: Optional[str] = None
        self._consecutive_failures = 0
        self._last_reconnect_attempt = -1e9
        self._boot_wait_done = False
        self._operation_depth = 0
        self._quiet = False
        self._poll_timer = None
        self._event_timer = None
        self._events: Deque[Tuple[str, Any]] = collections.deque()
        self._events_running = False
        # Debounced presence per slot: the accepted value and a candidate count
        self._presence: Dict[int, Optional[bool]] = {}
        self._presence_candidate: Dict[int, Tuple[Optional[bool], int]] = {}
        # Lanes waiting for the ACE to identify a freshly inserted spool's tag
        self._tag_wait: Dict[str, float] = {}
        self.lane_tags: Dict[str, Optional[Dict[str, Any]]] = {}

    def _trace_settings(self) -> Dict[str, Any]:
        keys = ("poll_interval", "request_timeout", "bowden_length", "feed_speed",
                "retract_speed", "approach_length", "approach_speed", "approach_extra",
                "load_attempts", "grab_length", "grab_speed", "feed_assist",
                "unload_retract_steps", "unload_extra", "unload_clear_length",
                "start_timeout", "move_timeout_margin", "presence_confirm")
        return {k: getattr(self, k, None) for k in keys}

    # -- lifecycle ---------------------------------------------------------

    def handle_connect(self):
        super().handle_connect()
        self.logo = '<span class=success--text>ACE Pro Ready</span>\n'
        self.logo_error = '<span class=error--text>ACE Pro Not Ready</span>\n'

    def _ace_handle_ready(self):
        self._event_timer = self.reactor.register_timer(self._run_events, self.reactor.NEVER)
        self._poll_timer = self.reactor.register_timer(self._poll, self.reactor.NOW)

    def _ace_handle_disconnect(self):
        for attr in ("_poll_timer", "_event_timer"):
            timer = getattr(self, attr)
            if timer is not None:
                self.reactor.unregister_timer(timer)
                setattr(self, attr, None)
        self.transport.close()
        self.trace.event("trace_stop", reason="klippy disconnect")
        self.trace.stop()

    def _cutter_callback(self, eventtime, state):
        state = bool(state)
        if state and not self.cutter_sensor_state:
            self.cutter_trigger_count += 1
        self.cutter_sensor_state = state
        self.trace.event("cutter_sensor", triggered=state, count=self.cutter_trigger_count)

    # -- connection --------------------------------------------------------

    def connect(self) -> bool:
        """Open the port and ask the ACE who it is."""
        if not self.transport.open():
            self._set_offline(self.transport.last_error)
            return False
        reply = self.transport.request("get_info", timeout=self.request_timeout, retries=1)
        if reply is None:
            self._set_offline("no reply to get_info")
            # Close so the next attempt reopens: after a USB reset the path
            # points at a new device node
            self.transport.close()
            return False
        if isinstance(reply.result, dict):
            self.info = dict(reply.result)
        return self._set_online()

    def _set_online(self) -> bool:
        if not self.online:
            self.online = True
            self._consecutive_failures = 0
            # The ACE forgets feed assist across a USB reset; the poll's
            # watchdog turns it back on for the loaded lane.
            self.assist_slot = None
            self.trace.event("online", info=self.info)
            self.logger.info("ACE {}: online ({} firmware {})".format(
                self.name, self.info.get("model", "ACE"), self.info.get("firmware", "?")))
        return True

    def _set_offline(self, reason: Optional[str]) -> None:
        if self.online:
            self.logger.warning("ACE {}: offline ({})".format(self.name, reason))
            self.trace.event("offline", reason=reason)
        self.online = False
        self.assist_slot = None

    def request(self, method: str, params: Optional[Dict[str, Any]] = None,
                timeout: Optional[float] = None, retries: int = 1) -> Optional[P.Reply]:
        """A request on this unit's link; counts failures toward offline."""
        if self.transport.fd is None and self.online:
            # The port dropped (USB reset). Reopen before giving up.
            self._set_offline(self.transport.last_error or "port closed")
        if not self.online and not self._reconnect_now():
            return None
        reply = self.transport.request(method, params, timeout=timeout or self.request_timeout,
                                       retries=retries, quiet=self._quiet)
        if reply is None:
            self._consecutive_failures += 1
            if self._consecutive_failures >= self.offline_after or self.transport.fd is None:
                self._set_offline("no reply to {}".format(method))
                self.transport.close()
        else:
            self._consecutive_failures = 0
        return reply

    def _reconnect_now(self) -> bool:
        now = self.reactor.monotonic()
        if now - self._last_reconnect_attempt < self.reconnect_interval:
            return False
        self._last_reconnect_attempt = now
        try:
            return self.connect()
        except Exception:
            self.logger.debug("ACE {}: reconnect failed\n{}".format(
                self.name, traceback.format_exc()))
            return False

    def _on_unsolicited(self, reply: P.Reply) -> None:
        if isinstance(reply.result, dict) and "slots" in reply.result:
            self._store_status(reply.result)

    # -- status ------------------------------------------------------------

    def _store_status(self, status: Dict[str, Any]) -> None:
        self.status = status
        self.status_time = self.reactor.monotonic()

    def get_status_now(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        reply = self.request("get_status", timeout=timeout, retries=0)
        if reply is not None and reply.ok and isinstance(reply.result, dict):
            self._store_status(reply.result)
            return reply.result
        return None

    @property
    def operation_active(self) -> bool:
        return self._operation_depth > 0

    class _Operation:
        def __init__(self, unit):
            self.unit = unit

        def __enter__(self):
            self.unit._operation_depth += 1
            return self.unit

        def __exit__(self, *exc):
            self.unit._operation_depth -= 1
            if self.unit._operation_depth == 0:
                # Slot readings taken mid-move don't count toward presence
                self.unit._presence_candidate.clear()
            return False

    def operation(self):
        """Context manager marking a move sequence: presence isn't mirrored
        from the flickering mid-move slot states while one runs."""
        return self._Operation(self)

    def _poll(self, eventtime: float) -> float:
        """
        Status poll, also the ACE's keepalive. Runs every poll_interval and
        never waits on anything but the serial link: work that can block (gcode
        for runouts, tag colours) is handed to _run_events.
        """
        try:
            if not self.online:
                self._reconnect_now()
                return self.reactor.monotonic() + min(self.poll_interval, 0.5)
            if self.transport.busy:
                # A move is talking to the ACE; that keeps the link alive too
                return self.reactor.monotonic() + 0.2
            if self.reactor.monotonic() - self.transport.last_tx < self.poll_interval * 0.5:
                return self.transport.last_tx + self.poll_interval
            self._quiet = not self.trace_polls
            try:
                status = self.get_status_now()
            finally:
                self._quiet = False
            if status is not None and not self.operation_active:
                self._mirror_presence(status)
                self._check_tags(status)
                self._assist_watchdog()
        except Exception:
            self.logger.debug("ACE {}: poll failed\n{}".format(self.name, traceback.format_exc()))
        return self.reactor.monotonic() + self.poll_interval

    def _queue_event(self, kind: str, payload: Any) -> None:
        self._events.append((kind, payload))
        if self._event_timer is not None:
            self.reactor.update_timer(self._event_timer, self.reactor.NOW)
        else:
            self._drain_events()

    def _run_events(self, eventtime: float) -> float:
        self._drain_events()
        return self.reactor.NEVER

    def _drain_events(self) -> None:
        if self._events_running:
            return
        self._events_running = True
        try:
            while self._events:
                kind, payload = self._events.popleft()
                try:
                    if kind == "presence":
                        lane, present = payload
                        lane.ace_update_presence(present)
                    elif kind == "tag":
                        lane, tag = payload
                        self.apply_tag(lane, tag, self.afc.gcode.run_script)
                except Exception:
                    self.logger.warning("ACE {}: {} event failed\n{}".format(
                        self.name, kind, traceback.format_exc()))
        finally:
            self._events_running = False

    def slot_presence(self, status: Optional[Dict[str, Any]], slot: int) -> Optional[bool]:
        return P.presence(status, slot)

    def _mirror_presence(self, status: Dict[str, Any]) -> None:
        for lane in list(self.lanes.values()):
            if not hasattr(lane, "ace_update_presence"):
                continue
            slot = lane.slot
            reading = P.presence(status, slot)
            if reading is None:
                self._presence_candidate.pop(slot, None)
                continue
            if reading == self._presence.get(slot):
                self._presence_candidate.pop(slot, None)
                continue
            value, count = self._presence_candidate.get(slot, (None, 0))
            count = count + 1 if value == reading else 1
            if self._presence.get(slot) is None or count >= self.presence_confirm:
                self._presence_candidate.pop(slot, None)
                self._presence[slot] = reading
                self.trace.event("presence", lane=lane.name, slot=slot + 1, present=reading)
                self._queue_event("presence", (lane, reading))
            else:
                self._presence_candidate[slot] = (reading, count)

    def _check_tags(self, status: Dict[str, Any]) -> None:
        if not self._tag_wait:
            return
        now = self.reactor.monotonic()
        for name, deadline in list(self._tag_wait.items()):
            lane = self.lanes.get(name)
            if lane is None or now > deadline:
                self._tag_wait.pop(name, None)
                continue
            tag = P.slot_tag(status, lane.slot)
            if tag is not None:
                self._tag_wait.pop(name, None)
                self.lane_tags[name] = tag
                self.trace.event("rfid", lane=name, tag=tag)
                self._queue_event("tag", (lane, tag))

    def apply_tag(self, lane: AFCLane, tag: Dict[str, Any], run_gcode) -> List[str]:
        """Set the lane's color and material from a tag with AFC's own
        commands. A lane with a Spoolman spool is left alone."""
        if getattr(lane, "spool_id", None):
            self.logger.info("ACE {}: lane has Spoolman spool {}; tag not applied".format(
                lane.name, lane.spool_id))
            return []
        cmds = []
        if tag.get("color"):
            cmds.append("SET_COLOR LANE={} COLOR={}".format(lane.name, tag["color"]))
        if tag.get("material"):
            cmds.append("SET_MATERIAL LANE={} MATERIAL={}".format(lane.name, tag["material"]))
        for cmd in cmds:
            run_gcode(cmd)
        if cmds:
            self.logger.info("ACE {}: tag {} {}".format(
                lane.name, tag.get("material") or "", tag.get("color") or ""))
        return cmds

    def on_filament_insert(self, lane: AFCLane):
        super().on_filament_insert(lane)
        if self.rfid_apply:
            self._tag_wait[lane.name] = self.reactor.monotonic() + self.rfid_wait

    def on_filament_remove(self, lane: AFCLane):
        super().on_filament_remove(lane)
        self._tag_wait.pop(lane.name, None)
        self.lane_tags.pop(lane.name, None)

    # -- feed assist -------------------------------------------------------

    def _assisted_lane(self) -> Optional[AFCLane]:
        """This unit's lane that is loaded in the toolhead, if any."""
        for lane in self.lanes.values():
            if not getattr(lane, "tool_loaded", False):
                continue
            extruder = getattr(lane, "extruder_obj", None)
            if extruder is not None and getattr(extruder, "lane_loaded", lane.name) != lane.name:
                continue
            return lane
        return None

    def _assist_watchdog(self) -> None:
        if not self.feed_assist or self.operation_active:
            return
        lane = self._assisted_lane()
        if lane is None or not getattr(lane, "assist_enabled", True):
            return
        if self.assist_slot != lane.slot:
            self.start_assist(lane.slot)

    def start_assist(self, slot: int) -> bool:
        if self.assist_slot is not None and self.assist_slot != slot:
            self.stop_assist(self.assist_slot)
        reply = self.request("start_feed_assist", {"index": slot})
        if reply is not None and reply.ok:
            self.assist_slot = slot
            self.trace.event("assist", slot=slot + 1, on=True)
            return True
        self.trace.event("assist_failed", slot=slot + 1,
                         msg=reply.msg if reply is not None else "no reply")
        return False

    def stop_assist(self, slot: Optional[int] = None) -> bool:
        slot = self.assist_slot if slot is None else slot
        if slot is None:
            return True
        reply = self.request("stop_feed_assist", {"index": slot})
        ok = reply is not None and reply.code == 0
        if ok and self.assist_slot == slot:
            self.assist_slot = None
        self.trace.event("assist", slot=slot + 1, on=False, ok=ok)
        return ok

    def stop_all_assist(self) -> None:
        """Before any move: the ACE drives one slot at a time and doesn't ack a
        feed while another slot assists. Our record of which slot assists can
        be stale (USB reset, a raw command), and a stop on an idle slot is
        harmless, so stop all four."""
        for slot in range(P.SLOTS):
            self.request("stop_feed_assist", {"index": slot}, retries=0)
        if self.assist_slot is not None:
            self.trace.event("assist", slot=self.assist_slot + 1, on=False, ok=True)
        self.assist_slot = None

    # -- moves (used by lanes) ---------------------------------------------

    def wait_ready(self, timeout: Optional[float] = None) -> bool:
        """Wait until the ACE reports itself ready (not busy). A command sent
        while it is busy is answered FORBIDDEN."""
        deadline = self.reactor.monotonic() + (timeout or self.ready_timeout)
        while True:
            status = self.get_status_now()
            if status is not None and status.get("status") == P.UNIT_READY:
                return True
            if self.reactor.monotonic() > deadline or not self.online:
                return False
            self.reactor.pause(self.reactor.monotonic() + self.status_interval)

    def _send_move(self, method: str, slot: int, length: float, speed: float) -> Optional[str]:
        """Send a feed/unwind. Returns None when the ACE accepted it, else why not."""
        params = {"index": slot, "length": round(float(length), 1), "speed": round(speed, 1)}
        if method == "unwind_filament":
            params["mode"] = 0
        for attempt in range(3):
            if not self.wait_ready():
                if not self.online:
                    return "ACE offline"
            reply = self.request(method, params)
            if reply is None:
                return "no reply to {}".format(method)
            if reply.ok:
                return None
            if not reply.forbidden:
                return "{} refused: code {} {}".format(method, reply.code, reply.msg)
            self.reactor.pause(self.reactor.monotonic() + 0.5)
        return "{} refused: ACE stayed busy (FORBIDDEN)".format(method)

    def stop_move(self, method: str, slot: int, started: float) -> bool:
        """Stop a feed/unwind and confirm the slot stopped moving."""
        wait_until = started + STOP_MIN_AFTER_START
        now = self.reactor.monotonic()
        if now < wait_until:
            self.reactor.pause(wait_until)
        for _ in range(STOP_CONFIRM_TRIES):
            self.request(method, {"index": slot})
            self.reactor.pause(self.reactor.monotonic() + 0.15)
            status = self.get_status_now()
            if status is not None and not P.slot_moving(status, slot):
                return True
        self.logger.warning("ACE {}: slot {} did not confirm stopped".format(self.name, slot + 1))
        return False

    def run_move(self, method: str, slot: int, length: float, speed: float,
                 stop_when: Optional[Callable[[], bool]] = None,
                 watch: Optional[Callable[[], bool]] = None) -> Dict[str, Any]:
        """
        Run one feed_filament / unwind_filament and follow it to the end.

        stop_when: checked every SENSOR_POLL; once true the move is stopped
            (e.g. the toolhead sensor triggered during a load).
        watch: checked the same way; the distance at which it first turns true
            is reported (e.g. the toolhead sensor clearing during an unwind).

        Returns {"result": "stopped" | "done" | "error" | "no_start" |
        "timeout" | "refused", "moved": estimated mm, "detail": str,
        "watch_at": mm or None, "seconds": float}.
        """
        speed = P.clamp_speed(speed)
        expected = length / speed
        out: Dict[str, Any] = {"result": None, "moved": 0.0, "detail": "", "watch_at": None,
                               "error": None}
        self.last_move_error = None
        refused = self._send_move(method, slot, length, speed)
        started = self.reactor.monotonic()
        if refused:
            out.update(result="refused", detail=refused, seconds=0.0)
            return out
        deadline = started + expected * 1.5 + self.move_timeout_margin
        stop_method = "stop_feed_filament" if method == "feed_filament" else "stop_unwind_filament"
        moving_since: Optional[float] = None
        moving_time = 0.0
        idle_reads = 0
        last_status = 0.0

        def moved_now() -> float:
            t = moving_time
            if moving_since is not None:
                t += self.reactor.monotonic() - moving_since
            elif not moving_time:
                # Not seen moving yet; the first poll may simply be late
                t = max(0.0, self.reactor.monotonic() - started - 0.2)
            return min(length, t * speed)

        while True:
            now = self.reactor.monotonic()
            if stop_when is not None and stop_when():
                out["moved"] = moved_now()
                self.stop_move(stop_method, slot, started)
                out.update(result="stopped", detail="stopped at the sensor")
                break
            if watch is not None and out["watch_at"] is None and watch():
                out["watch_at"] = moved_now()
            if now - last_status >= self.status_interval:
                last_status = now
                status = self.get_status_now()
                if status is None:
                    if not self.online:
                        out.update(result="error", detail="ACE went offline during the move",
                                   moved=moved_now())
                        break
                else:
                    err = P.slot_error(status, slot)
                    if err and not self.last_move_error:
                        self.last_move_error = err
                        out["error"] = err
                        self.logger.warning("ACE {}: slot {} reported {}".format(
                            self.name, slot + 1, err))
                    if P.slot_moving(status, slot):
                        if moving_since is None:
                            moving_since = self.reactor.monotonic()
                        idle_reads = 0
                    else:
                        if moving_since is not None:
                            moving_time += self.reactor.monotonic() - moving_since
                            moving_since = None
                        if moving_time > 0 or err:
                            idle_reads += 1
                            if idle_reads >= 2:
                                out.update(result="error" if err else "done",
                                           detail=err or "move finished",
                                           moved=min(length, moving_time * speed)
                                           if not err else moved_now())
                                if not err:
                                    out["moved"] = length
                                break
                        elif now - started > self.start_timeout:
                            if expected <= 1.0 and not err:
                                # A short move can start and end between polls
                                out.update(result="done", detail="short move", moved=length)
                            else:
                                out.update(result="no_start",
                                           detail="the slot never reported moving")
                            break
            if now > deadline:
                out["moved"] = moved_now()
                self.stop_move(stop_method, slot, started)
                out.update(result="timeout", detail="move did not finish in {:.0f}s".format(
                    deadline - started))
                break
            self.reactor.pause(self.reactor.monotonic() + SENSOR_POLL)
        out["seconds"] = round(self.reactor.monotonic() - started, 2)
        self.trace.event("move", method=method, slot=slot + 1, length=round(length, 1),
                         speed=speed, result=out["result"], moved=round(out["moved"], 1),
                         watch_at=out["watch_at"], error=out["error"], seconds=out["seconds"])
        return out

    # -- AFC unit hooks ----------------------------------------------------

    def system_Test(self, cur_lane: AFCLane, delay: float, assignTcmd: bool,
                    enable_movement: bool) -> bool:
        """Restore lane state during PREP from the ACE's own sensors."""
        if not self.online:
            self._last_reconnect_attempt = -1e9
            self._reconnect_now()
        if not self.online and not self._boot_wait_done:
            self._wait_for_ace()
        self._boot_wait_done = True
        status = self.status
        if self.online and (status is None or self.reactor.monotonic() - self.status_time > 2.0):
            status = self.get_status_now()

        present = self.slot_presence(status, cur_lane.slot) if self.online else None
        if present is None and self.online and status is not None:
            # Mid-move or busy: an unknown slot counts as unknown, not empty
            present = self._presence.get(cur_lane.slot)
        if not self.online or present is None:
            # Unknown, not empty: the first trusted reading syncs this lane
            # quietly instead of treating it as a new spool.
            cur_lane._ace_present_last = None
            msg = "<span class=error--text>ACE NOT RESPONDING</span>" if not self.online \
                else "SLOT STATE UNKNOWN"
            self.lane_not_ready(cur_lane)
        else:
            self._presence[cur_lane.slot] = present
            cur_lane._ace_present_last = present
            msg = self.apply_lane_state(cur_lane, present)

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.send_lane_data()
        cur_lane.do_enable(False)
        to_string = getattr(cur_lane, "map_to_string", None)
        tcmd = to_string() if callable(to_string) else str(cur_lane.map)
        self.logger.info("{lane} tool cmd: {tcmd:3} {msg}".format(
            lane=cur_lane.name, tcmd=tcmd, msg=msg))
        cur_lane.set_afc_prep_done()
        return self.online

    def _wait_for_ace(self) -> None:
        """Keep trying to reach an ACE that is still starting, up to boot_wait."""
        if self.boot_wait <= 0:
            return
        self.logger.info("ACE {}: waiting up to {:.0f}s for the ACE to start".format(
            self.name, self.boot_wait))
        deadline = self.reactor.monotonic() + self.boot_wait
        while not self.online and self.reactor.monotonic() < deadline:
            self.reactor.pause(self.reactor.monotonic() + 1.0)
            self._last_reconnect_attempt = -1e9
            self._reconnect_now()

    def apply_lane_state(self, cur_lane: AFCLane, present: bool) -> str:
        """Set a lane's presence and LEDs from the slot sensor; returns a
        PREP-style status message."""
        cur_lane.prep_state = present
        cur_lane.loaded_to_hub = present
        cur_lane._load_state = bool(cur_lane.tool_loaded)
        if not present:
            self.lane_not_ready(cur_lane)
            return "EMPTY READY FOR SPOOL"
        self.lane_loaded(cur_lane)
        msg = "FILAMENT PRESENT"
        if (cur_lane.tool_loaded
                and cur_lane.extruder_obj is not None
                and cur_lane.extruder_obj.lane_loaded == cur_lane.name):
            if self.afc.current == cur_lane.name:
                self.lane_tool_loaded(cur_lane)
            else:
                self.lane_tool_loaded_idle(cur_lane)
            msg += " in ToolHead"
        return msg

    def prep_load(self, lane: AFCLane) -> None:
        # The ACE stages a newly inserted spool at its outlet on its own.
        return

    def prep_post_load(self, lane: AFCLane) -> None:
        return

    def eject_lane(self, lane: AFCLane) -> None:
        """AFC's eject hook: pull a lane's filament back into the ACE."""
        ok, detail = self.eject(lane)
        log = self.logger.info if ok else self.logger.error
        log("ACE {}: eject {}: {}".format(self.name, lane.name, detail))

    def eject(self, lane: AFCLane) -> Tuple[bool, str]:
        if lane.tool_loaded:
            return False, "lane is loaded in the toolhead; use AFC_ACEPRO_TOOL_UNLOAD"
        with self.operation():
            self.stop_all_assist()
            res = self.run_move("unwind_filament", lane.slot, self.eject_length,
                                self.retract_speed)
        ok = res["result"] in ("done", "stopped")
        return ok, "unwound {:.0f}mm ({})".format(res["moved"], res["detail"])

    def move_to_hub(self, lane, dist, dir, use_homing=True, speed_mode=None, assist_active=None):
        self.logger.debug("ACE {}: move_to_hub not supported, the ACE feeds".format(self.name))
        return False, 0, AFCMoveWarning.ERROR

    def load_then_home(self, lane, distance, assist_active, endstop):
        self.logger.debug("ACE {}: load_then_home not supported, the ACE feeds".format(self.name))
        return False, 0, AFCMoveWarning.ERROR

    def calibrate_lane(self, cur_lane, tol):
        return False, self.CALIBRATION_UNSUPPORTED, 0.0

    def calibrate_bowden(self, cur_lane, dis, tol):
        return False, self.CALIBRATION_UNSUPPORTED, 0.0

    def calibrate_td1(self, cur_lane, dis, tol):
        return False, self.CALIBRATION_UNSUPPORTED, 0.0

    def cmd_UNIT_CALIBRATION(self, gcmd):
        prompt = AFCprompt(gcmd, self.logger)
        prompt.create_custom_p("{} Calibration".format(self.name),
                               self.CALIBRATION_UNSUPPORTED, cancel=True)

    def cmd_UNIT_LANE_CALIBRATION(self, gcmd):
        self.cmd_UNIT_CALIBRATION(gcmd)

    def cmd_UNIT_BOW_CALIBRATION(self, gcmd):
        self.cmd_UNIT_CALIBRATION(gcmd)

    def cmd_AFC_UNIT_TD_ONE_CALIBRATION(self, gcmd):
        self.cmd_UNIT_CALIBRATION(gcmd)

    # -- gcode -------------------------------------------------------------

    cmd_AFC_ACEPRO_STATUS_help = "Query an ACE Pro and report its state"

    def cmd_AFC_ACEPRO_STATUS(self, gcmd: GCodeCommand) -> None:
        """
        Usage
        -----
        `AFC_ACEPRO_STATUS UNIT=<unit>`
        """
        if not self.online:
            self._last_reconnect_attempt = -1e9
            self._reconnect_now()
        if not self.online:
            gcmd.respond_info("ACE {}: offline ({})".format(
                self.name, self.transport.last_error or "no reply"))
            return
        status = self.get_status_now() or self.status or {}
        dryer = P.dryer_of(status)
        t = self.transport
        lines = [
            "ACE {} @ {}".format(self.name, self.serial_path),
            "  model: {} firmware {} boot {}".format(
                self.info.get("model", "?"), self.info.get("firmware", "?"),
                self.info.get("boot_firmware", "?")),
            "  state: {} {}".format(status.get("status", "?"), status.get("action", "") or ""),
            "  temperature: {} C, fan {} rpm".format(status.get("temp", "?"),
                                                     status.get("fan_speed", "?")),
            "  dryer: {}{}".format(dryer.get("status", "?"),
                                   " {} C, {} of {} min left".format(
                                       dryer.get("target_temp"), dryer.get("remain_time"),
                                       dryer.get("duration"))
                                   if dryer.get("status") == "drying" else ""),
            "  feed assist: {}".format("slot {}".format(self.assist_slot + 1)
                                       if self.assist_slot is not None else "off"),
            "  cutter sensor: {}".format(
                ("triggered" if self.cutter_sensor_state else "open")
                if self.cutter_sensor_pin else "not configured"),
            "  link: tx {} rx {} timeouts {} late {} crc errors {} reopened {}x".format(
                t.tx_count, t.rx_count, t.timeouts, t.late_replies, t._parser.crc_errors,
                max(t.opens - 1, 0)),
            "  trace: {}".format(self._trace_summary()),
        ]
        for lane in self.lanes.values():
            slot = getattr(lane, "slot", None)
            if slot is None:
                continue
            line = "  {} -> slot {}: {}".format(lane.name, slot + 1,
                                                P.slot_status(status, slot) or "?")
            tag = P.slot_tag(status, slot)
            if tag:
                line += ", tag {} {}".format(tag.get("material") or "", tag.get("color") or "")
            lines.append(line)
        gcmd.respond_info("\n".join(lines))

    cmd_AFC_ACEPRO_RECONNECT_help = "Close and reopen the link to an ACE Pro"

    def cmd_AFC_ACEPRO_RECONNECT(self, gcmd: GCodeCommand) -> None:
        self.transport.close()
        self._set_offline("reconnect requested")
        ok = self.connect()
        gcmd.respond_info("ACE {}: {}".format(
            self.name, "online" if ok else "offline ({})".format(
                self.transport.last_error or "no reply")))

    def _trace_summary(self) -> str:
        if not self.trace_file:
            return "off (set trace_file to enable)"
        if not self.trace.enabled:
            return "paused ({})".format(self.trace.last_error or self.trace.path)
        extra = ", {} dropped".format(self.trace.dropped) if self.trace.dropped else ""
        return "recording to {} ({} events{})".format(self.trace.path, self.trace.written, extra)

    cmd_AFC_ACEPRO_TRACE_help = "Control the ACE trace recording or add a marker to it"

    def cmd_AFC_ACEPRO_TRACE(self, gcmd: GCodeCommand) -> None:
        """
        Usage
        -----
        `AFC_ACEPRO_TRACE UNIT=<unit> [ENABLE=<0|1>] [MARK=<text>]`
        """
        enable = gcmd.get_int("ENABLE", None, minval=0, maxval=1)
        mark = gcmd.get("MARK", None)
        if enable is not None:
            if enable:
                if not self.trace_file:
                    raise gcmd.error("ACE {}: set trace_file in [AFC_ACEPRO {}] first".format(
                        self.name, self.name))
                self.trace.start()
                self.transport.trace = self.trace
                self.trace.event("trace_resume")
            else:
                self.trace.event("trace_pause")
                self.trace.enabled = False
        if mark is not None:
            self.trace.event("mark", text=mark)
        gcmd.respond_info("ACE {} trace: {}".format(self.name, self._trace_summary()))

    cmd_AFC_ACEPRO_RAW_help = "Send one raw ACE request (debugging only)"

    def cmd_AFC_ACEPRO_RAW(self, gcmd: GCodeCommand) -> None:
        """
        Send one request and print the reply. For bring-up only: moves sent
        this way bypass all of AFC's checks.

        Usage
        -----
        `AFC_ACEPRO_RAW UNIT=<unit> METHOD=<method> [PARAMS=<json>] [TIMEOUT=<s>]`

        Example
        -------
        ```
        AFC_ACEPRO_RAW UNIT=ACE_1 METHOD=get_filament_info PARAMS='{"index":0}'
        ```
        """
        method = gcmd.get("METHOD")
        raw = gcmd.get("PARAMS", None)
        params = None
        if raw:
            raw = raw.strip()
            if len(raw) >= 2 and raw[0] == raw[-1] and raw[0] in "'\"":
                raw = raw[1:-1]
            try:
                params = json.loads(raw)
            except ValueError as e:
                raise gcmd.error("PARAMS must be JSON: {}".format(e))
        timeout = gcmd.get_float("TIMEOUT", 3.0, above=0.)
        reply = self.request(method, params, timeout=timeout, retries=0)
        if reply is None:
            gcmd.respond_info("ACE {}: {} (no reply)".format(self.name, method))
        else:
            gcmd.respond_info("ACE {}: {} -> code={} msg={} result={}".format(
                self.name, method, reply.code, reply.msg,
                json.dumps(reply.result, separators=(",", ":"))))

    cmd_AFC_ACEPRO_DRY_help = "Start the ACE Pro's filament dryer"

    def cmd_AFC_ACEPRO_DRY(self, gcmd: GCodeCommand) -> None:
        """
        Usage
        -----
        `AFC_ACEPRO_DRY UNIT=<unit> [TEMP=<C>] [DURATION=<minutes>]`
        """
        if self.max_dryer_temperature <= 0:
            raise gcmd.error("ACE {}: dryer disabled (max_dryer_temperature: 0)".format(self.name))
        temp = gcmd.get_float("TEMP", min(50.0, self.max_dryer_temperature), minval=20.)
        if temp > self.max_dryer_temperature:
            raise gcmd.error("ACE {}: TEMP {:.0f} is above max_dryer_temperature {:.0f}".format(
                self.name, temp, self.max_dryer_temperature))
        duration = gcmd.get_int("DURATION", 240, minval=1, maxval=24 * 60)
        reply = self.request("drying", {"temp": int(round(temp)), "fan_speed": self.dryer_fan_speed,
                                        "duration": duration})
        if reply is None or reply.code != 0:
            raise gcmd.error("ACE {}: dryer did not start ({})".format(
                self.name, reply.msg if reply is not None else "no reply"))
        self.trace.event("dryer", on=True, temp=temp, duration=duration)
        gcmd.respond_info("ACE {}: drying at {:.0f} C for {} min".format(self.name, temp, duration))

    cmd_AFC_ACEPRO_DRY_STOP_help = "Stop the ACE Pro's filament dryer"

    def cmd_AFC_ACEPRO_DRY_STOP(self, gcmd: GCodeCommand) -> None:
        reply = self.request("drying_stop")
        if reply is None or reply.code != 0:
            raise gcmd.error("ACE {}: dryer did not stop ({})".format(
                self.name, reply.msg if reply is not None else "no reply"))
        self.trace.event("dryer", on=False)
        gcmd.respond_info("ACE {}: dryer stopped".format(self.name))

    def get_status(self, eventtime=None):
        response = super().get_status(eventtime)
        status = self.status or {}
        dryer = P.dryer_of(status)
        response["ace"] = {
            "online": self.online,
            "model": self.info.get("model"),
            "firmware": self.info.get("firmware"),
            "state": status.get("status"),
            "temperature": status.get("temp"),
            "dryer": dryer.get("status"),
            "dryer_target": dryer.get("target_temp"),
            "dryer_remaining": dryer.get("remain_time"),
            "feed_assist_slot": self.assist_slot + 1 if self.assist_slot is not None else None,
            "cutter_sensor": self.cutter_sensor_state if self.cutter_sensor_pin else None,
            "trace": self.trace.path if self.trace.enabled else None,
        }
        return response


def load_config_prefix(config):
    return afcACEPRO(config)
