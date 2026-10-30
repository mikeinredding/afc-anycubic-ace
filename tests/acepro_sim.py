"""
A simulated Anycubic ACE Pro (V1) on a pseudo-terminal, for exercising
AFC_ACEPRO against Klipper's real reactor without hardware.

It speaks the real wire format (via AFC_ACEPRO_protocol) and models just
enough physics to drive the load/unload sequences. Positions are mm of
filament past the slot's staging point at the ACE outlet:

  hub_len   the shared splitter sensor (an optional [AFC_hub] switch)
  path_len  the toolhead sensor (above the extruder gears)
  grip_len  past this the extruder gears hold the filament: the ACE can't
            push further unless the extruder turns, or pull back unless the
            extruder has retracted out of the gears
  empty_at  unwound this far back, the slot sensor no longer sees filament

Quirks it reproduces, from the ACE_series driver and ACEResearch notes: the
ACE drops off USB after ~3 s without a complete frame (here: the pty is torn
down and a new one appears behind the same symlink, like /dev/serial/by-id),
a stop within ~250 ms of a move's start is ignored, a move sent while busy is
answered FORBIDDEN, a slot reads 'empty' for a moment while the unit is busy,
and a jammed feed ends in 'feed_error'.
"""
from __future__ import annotations

import json
import os
import pty
import select
import tempfile
import threading
import time
import tty

from extras import AFC_ACEPRO_protocol as P


class Slot:
    def __init__(self, index):
        self.index = index
        self.present = False
        self.position = 0.0
        self.tag = None            # {"type":..., "color":[r,g,b]} once identified
        self.rfid = P.RFID_NONE
        self.rfid_ready_at = 0.0
        self.error_until = 0.0
        self.error = None


class FakeACE:
    def __init__(self, present=(True, True, False, False), path_len=120.0, grip_len=135.0,
                 hub_len=60.0, empty_at=-40.0, keepalive=3.0, reset_delay=0.4,
                 start_delay=0.05, stop_ignore=0.25, reply_delay=0.005, tag_delay=0.6):
        self.slots = [Slot(i) for i in range(P.SLOTS)]
        for i, p in enumerate(present):
            self.slots[i].present = bool(p)
        self.path_len = path_len
        self.grip_len = grip_len
        self.hub_len = hub_len
        self.empty_at = empty_at
        self.keepalive = keepalive        # None = never drops off USB
        self.reset_delay = reset_delay
        self.start_delay = start_delay
        self.stop_ignore = stop_ignore
        self.reply_delay = reply_delay
        self.tag_delay = tag_delay

        self.lock = threading.RLock()
        self.move = None                  # {"slot","dir","remaining","speed","start","started"}
        self.assist_slot = None
        self.dryer = {"status": "stop", "target_temp": 0, "duration": 0, "remain_time": 0}
        self.log = []                     # (method, params) of every request
        self.frames_seen = 0
        self.usb_resets = 0
        self.mute = False                 # True: answer nothing (booting / hung)
        self.drop_replies = 0             # swallow the next N replies
        self.garbage_before_reply = b""
        self.jam_at = None                # (slot, position): a feed stalls here
        self.flicker_empty_when_busy = True
        self.ignored_stops = 0
        self.extruder_moves = []
        self._last_frame = time.monotonic()
        self._stop = False

        self.dir = tempfile.mkdtemp(prefix="acesim-")
        self.path = os.path.join(self.dir, "usb-ANYCUBIC_ACE_1-if00")
        self._open_pty()
        self._physics = threading.Thread(target=self._run_physics, daemon=True)
        self._physics.start()

    # -- USB device ----------------------------------------------------------

    def _open_pty(self):
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        self.slave_fd = slave
        name = os.ttyname(slave)
        tmp = self.path + ".new"
        if os.path.lexists(tmp):
            os.remove(tmp)
        os.symlink(name, tmp)
        os.replace(tmp, self.path)
        self._buf = b""
        self._parser = P.FrameParser()
        self._last_frame = time.monotonic()
        self._reader = threading.Thread(target=self._run_reader, args=(self.master,), daemon=True)
        self._reader.start()

    def usb_reset(self):
        """What the real ACE does after ~3 s without a frame: the device
        disappears and comes back as a new node."""
        with self.lock:
            self.usb_resets += 1
            old_master, old_slave = self.master, self.slave_fd
            self.master = None
            try:
                os.remove(self.path)
            except OSError:
                pass
            self.move = None
            self.assist_slot = None
        for fd in (old_slave, old_master):
            try:
                os.close(fd)
            except OSError:
                pass
        time.sleep(self.reset_delay)
        if not self._stop:
            with self.lock:
                self._open_pty()

    def close(self):
        self._stop = True
        for fd in (getattr(self, "master", None), getattr(self, "slave_fd", None)):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass

    # -- helpers for tests ---------------------------------------------------

    def insert(self, index, tag=None):
        with self.lock:
            s = self.slots[index]
            s.present = True
            s.position = 0.0
            s.tag = tag
            s.rfid = P.RFID_IDENTIFYING if tag else P.RFID_NONE
            s.rfid_ready_at = time.monotonic() + self.tag_delay

    def remove(self, index):
        with self.lock:
            s = self.slots[index]
            s.present = False
            s.tag = None
            s.rfid = P.RFID_NONE

    def count(self, method):
        return sum(1 for m, _ in self.log if m == method)

    def methods(self):
        return [m for m, _ in self.log]

    @property
    def toolhead_sensor(self):
        with self.lock:
            return any(s.position >= self.path_len for s in self.slots)

    @property
    def hub_sensor(self):
        with self.lock:
            return any(s.position >= self.hub_len for s in self.slots)

    def extruder_move(self, distance):
        """The toolhead extruder turned by distance (harness move_e_pos)."""
        with self.lock:
            self.extruder_moves.append(distance)
            for s in self.slots:
                if s.position < self.grip_len:
                    continue          # the gears turn on nothing
                if distance > 0:
                    s.position += distance
                else:
                    # Once the tip is out of the gears they stop pulling
                    s.position = max(s.position + distance, self.grip_len - 0.5)

    def cut(self, below_gears=10.0):
        """AFC_CUT: the blade sits below the gears; the part beyond it stays
        in the nozzle as a stub."""
        with self.lock:
            for s in self.slots:
                if s.position > self.grip_len + below_gears:
                    s.position = self.grip_len + below_gears

    # -- wire ----------------------------------------------------------------

    def _run_reader(self, master):
        while not self._stop and master == self.master:
            try:
                r, _, _ = select.select([master], [], [], 0.05)
            except (OSError, ValueError):
                return
            if not r:
                continue
            try:
                data = os.read(master, 4096)
            except OSError:
                return
            if not data:
                continue
            for msg in self._parser.feed(data):
                self._last_frame = time.monotonic()
                self.frames_seen += 1
                self._handle(master, msg.raw)

    def _send(self, master, obj):
        if self.drop_replies > 0:
            self.drop_replies -= 1
            return
        payload = json.dumps(obj, separators=(",", ":")).encode()
        frame = self.garbage_before_reply + P.encode_frame(payload) \
            if len(payload) <= P.MAX_REQUEST_PAYLOAD else None
        if frame is None:   # big replies: same framing, no request size limit
            import struct
            frame = (self.garbage_before_reply + P.FRAME_HEAD + struct.pack("<H", len(payload))
                     + payload + struct.pack("<H", P.crc16(payload)) + P.FRAME_TAIL)
        try:
            os.write(master, frame)
        except OSError:
            pass

    def _reply(self, master, rid, result=None, code=0, msg="success"):
        if self.reply_delay:
            time.sleep(self.reply_delay)
        self._send(master, {"id": rid, "code": code, "msg": msg, "result": result})

    def _handle(self, master, req):
        method = req.get("method")
        params = req.get("params") or {}
        rid = req.get("id")
        with self.lock:
            self.log.append((method, params))
        if self.mute:
            return
        handler = getattr(self, "m_" + str(method), None)
        if handler is None:
            self._reply(master, rid, code=1, msg="unknown method")
            return
        with self.lock:
            result = handler(params)
        if result == "NO_REPLY":
            return
        if isinstance(result, tuple):
            res, code, msg = result
        else:
            res, code, msg = result, 0, "success"
        self._reply(master, rid, res, code, msg)

    # -- methods -------------------------------------------------------------

    def m_get_info(self, params):
        return {"id": 1, "slots": 4, "model": "Anycubic Color Engine Pro",
                "firmware": "V1.3.84", "boot_firmware": "V1.0.1"}

    def _slot_status(self, s):
        now = time.monotonic()
        m = self.move
        if m is not None and m["slot"] == s.index and m["started"]:
            return "feeding" if m["dir"] > 0 else "unwinding"
        if s.error and now < s.error_until:
            return s.error
        if not s.present or s.position < self.empty_at:
            return "empty"
        if self.flicker_empty_when_busy and m is not None and m["slot"] != s.index:
            return "empty"
        return "ready"

    def m_get_status(self, params):
        now = time.monotonic()
        slots = []
        for s in self.slots:
            if s.tag and s.rfid == P.RFID_IDENTIFYING and now >= s.rfid_ready_at:
                s.rfid = P.RFID_IDENTIFIED
            entry = {"index": s.index, "status": self._slot_status(s), "sku": "", "brand": "",
                     "type": "", "color": [0, 0, 0], "rfid": s.rfid}
            if s.tag and s.rfid == P.RFID_IDENTIFIED:
                entry.update(type=s.tag.get("type", ""), color=s.tag.get("color", [0, 0, 0]),
                             brand="Anycubic")
            slots.append(entry)
        m = self.move
        busy = m is not None
        return {"status": "busy" if busy else "ready",
                "action": ("feeding" if m["dir"] > 0 else "unwinding") if busy else "",
                "dryer_status": dict(self.dryer), "temp": 26, "enable_rfid": 1,
                "fan_speed": 7000 if self.dryer["status"] == "drying" else 0,
                "feed_assist_count": 0, "cont_assist_time": 0.0, "slots": slots}

    def _start_move(self, params, direction):
        if self.move is not None:
            return None, 0, "FORBIDDEN"
        index = int(params.get("index", -1))
        if not 0 <= index < P.SLOTS or not self.slots[index].present:
            return None, 1, "slot not ready"
        if self.assist_slot is not None and self.assist_slot != index:
            return "NO_REPLY"            # the ACE drives one slot at a time
        self.move = {"slot": index, "dir": direction, "remaining": float(params["length"]),
                     "speed": float(params["speed"]), "start": time.monotonic(),
                     "started": False, "stalled_since": None}
        return None

    def m_feed_filament(self, params):
        return self._start_move(params, +1)

    def m_unwind_filament(self, params):
        return self._start_move(params, -1)

    def _stop_move(self, params, direction):
        m = self.move
        if m is None or m["slot"] != int(params.get("index", -1)) or m["dir"] != direction:
            return None
        if time.monotonic() - m["start"] < self.stop_ignore:
            self.ignored_stops += 1
            return None
        self.move = None
        return None

    def m_stop_feed_filament(self, params):
        return self._stop_move(params, +1)

    def m_stop_unwind_filament(self, params):
        return self._stop_move(params, -1)

    def m_start_feed_assist(self, params):
        index = int(params.get("index", -1))
        if self.move is not None:
            return None, 0, "FORBIDDEN"
        self.assist_slot = index
        return None

    def m_stop_feed_assist(self, params):
        if self.assist_slot == int(params.get("index", -1)):
            self.assist_slot = None
        return None, 0, ""

    def m_drying(self, params):
        self.dryer = {"status": "drying", "target_temp": params.get("temp"),
                      "duration": params.get("duration"), "remain_time": params.get("duration")}
        return None, 0, "drying"

    def m_drying_stop(self, params):
        self.dryer = {"status": "stop", "target_temp": 0, "duration": 0, "remain_time": 0}
        return None

    def m_get_filament_info(self, params):
        s = self.slots[int(params.get("index", 0))]
        return {"index": s.index, "type": (s.tag or {}).get("type", ""),
                "color": (s.tag or {}).get("color", [0, 0, 0]), "rfid": s.rfid}

    # -- physics -------------------------------------------------------------

    def _run_physics(self):
        last = time.monotonic()
        while not self._stop:
            time.sleep(0.005)
            now = time.monotonic()
            dt = now - last
            last = now
            if self.keepalive is not None and self.master is not None \
                    and now - self._last_frame > self.keepalive:
                self.usb_reset()
                last = time.monotonic()
                continue
            with self.lock:
                m = self.move
                if m is None:
                    continue
                if not m["started"]:
                    if now - m["start"] >= self.start_delay:
                        m["started"] = True
                    continue
                s = self.slots[m["slot"]]
                step = min(m["speed"] * dt, m["remaining"])
                blocked = False
                if m["dir"] > 0:
                    limit = self.grip_len + 2.0
                    if self.jam_at is not None and self.jam_at[0] == s.index:
                        limit = min(limit, self.jam_at[1])
                    if s.position + step > limit:
                        step = max(limit - s.position, 0.0)
                        blocked = True
                    s.position += step
                else:
                    if s.position >= self.grip_len:
                        step, blocked = 0.0, True      # the gears hold it
                    s.position -= step
                m["remaining"] -= step
                if blocked and step < 1e-6:
                    if m["stalled_since"] is None:
                        m["stalled_since"] = now
                    elif now - m["stalled_since"] > 0.6:
                        s.error = "feed_error" if m["dir"] > 0 else "unwind_error"
                        s.error_until = now + 1.0
                        self.move = None
                        continue
                else:
                    m["stalled_since"] = None
                if m["remaining"] <= 1e-6:
                    self.move = None
