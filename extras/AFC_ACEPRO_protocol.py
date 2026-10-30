# AFCProject Automated Filament Changer Software
#
# Anycubic ACE Pro (first generation) wire protocol.
#
# Pure-Python helpers with no Klipper imports so they can be unit tested on
# their own. The transport (serial port + Klipper reactor) lives in
# AFC_ACEPRO.py.
#
# Protocol knowledge comes from the community reverse-engineering work of:
#   - printers-for-people/ACEResearch (PROTOCOL.md: framing, methods)
#   - utkabobr/DuckACE, BlackFrogKok/BunnyACE, szkrisz/ACEPROSV08,
#     swilsonnc/ACEPROK1Max (K1 Max with the CFS upgrade kit)
#   - Kobra-S1/ACEPRO and the AFC ACE_series branch (slot states, quirks)
# All GPL-3.0, same as this project.
#
# Frame: FF AA | payload length (u16 LE) | JSON | CRC-16/MCRF4XX (u16 LE) | FE
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import json
import struct
from typing import Any, Dict, List, NamedTuple, Optional

DEFAULT_BAUD = 115200        # USB CDC: the value is ignored by the device

FRAME_HEAD = b"\xff\xaa"
FRAME_TAIL = b"\xfe"
# The ACE freezes on a frame longer than about 1 KB, so never send one.
MAX_REQUEST_PAYLOAD = 1024
# Replies can be larger (get_status with four slots is ~700 bytes); anything
# past this is a false header inside other data.
MAX_REPLY_PAYLOAD = 8192

SLOTS = 4

# The firmware takes feed/unwind speeds up to 100 mm/s.
MAX_SPEED = 100.0

# Slot "status" values seen from V1 firmware
SLOT_READY = "ready"         # spool in the slot, staged at the outlet
SLOT_EMPTY = "empty"
# Feed assist is not in this list: the slot keeps reporting 'ready' while
# it assists during a print, and presence must keep working then.
SLOT_MOVING = ("feeding", "unwinding", "shifting", "preloading", "rollback", "winding")
# get_status "status"
UNIT_READY = "ready"
UNIT_BUSY = "busy"

# get_filament_info / slot "rfid"
RFID_NONE = 0
RFID_FAILED = 1
RFID_IDENTIFIED = 2
RFID_IDENTIFYING = 3


def crc16(data: bytes) -> int:
    """CRC-16/MCRF4XX (reflected 0x1021, init 0xFFFF, no final xor)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def encode_payload(request_id: int, method: str, params: Optional[Dict[str, Any]] = None) -> bytes:
    request: Dict[str, Any] = {"id": request_id, "method": method}
    if params is not None:
        request["params"] = params
    return json.dumps(request, separators=(",", ":")).encode("utf-8")


def encode_frame(payload: bytes) -> bytes:
    if len(payload) > MAX_REQUEST_PAYLOAD:
        raise ValueError("ACE payload too large ({} > {} bytes)".format(
            len(payload), MAX_REQUEST_PAYLOAD))
    return (FRAME_HEAD + struct.pack("<H", len(payload)) + payload
            + struct.pack("<H", crc16(payload)) + FRAME_TAIL)


def frame_is_safe(frame: bytes) -> bool:
    """
    False if the frame carries the header bytes anywhere past its own header.
    A corrupted header followed by a stray FF AA inside the frame can make the
    ACE wait for a huge frame and freeze (ACEResearch), so the transport
    re-encodes such a request with the next id.
    """
    return frame.find(FRAME_HEAD, 1) < 0


class Reply(NamedTuple):
    id: Optional[int]
    code: int
    msg: str
    result: Any
    raw: Dict[str, Any]

    @property
    def ok(self) -> bool:
        return self.code == 0 and (self.msg or "").upper() != "FORBIDDEN"

    @property
    def forbidden(self) -> bool:
        return (self.msg or "").upper() == "FORBIDDEN"


def to_reply(obj: Dict[str, Any]) -> Reply:
    rid = obj.get("id")
    try:
        rid = int(rid) if rid is not None else None
    except (TypeError, ValueError):
        rid = None
    try:
        code = int(obj.get("code", 0) or 0)
    except (TypeError, ValueError):
        code = -1
    return Reply(rid, code, str(obj.get("msg", "") or ""), obj.get("result"), obj)


class FrameParser:
    """Reassembles frames from a byte stream. Resyncs past garbage, bad
    lengths, bad footers and CRC errors."""

    def __init__(self) -> None:
        self.buf = b""
        self.crc_errors = 0
        self.json_errors = 0
        self.dropped_bytes = 0

    def reset(self) -> None:
        self.buf = b""

    def feed(self, chunk: bytes) -> List[Reply]:
        self.buf += chunk
        out: List[Reply] = []
        while True:
            start = self.buf.find(FRAME_HEAD)
            if start < 0:
                # Keep a trailing FF: it may be the first byte of a header
                keep = 1 if self.buf.endswith(FRAME_HEAD[:1]) else 0
                self.dropped_bytes += len(self.buf) - keep
                self.buf = self.buf[len(self.buf) - keep:]
                return out
            if start:
                self.dropped_bytes += start
                self.buf = self.buf[start:]
            if len(self.buf) < 4:
                return out
            length = struct.unpack_from("<H", self.buf, 2)[0]
            if length > MAX_REPLY_PAYLOAD:
                self.buf = self.buf[2:]
                self.dropped_bytes += 2
                continue
            end = 4 + length + 3
            if len(self.buf) < end:
                return out
            payload = self.buf[4:4 + length]
            crc = struct.unpack_from("<H", self.buf, 4 + length)[0]
            if self.buf[4 + length + 2:end] != FRAME_TAIL or crc != crc16(payload):
                self.crc_errors += 1
                self.buf = self.buf[2:]
                self.dropped_bytes += 2
                continue
            self.buf = self.buf[end:]
            try:
                obj = json.loads(payload.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                self.json_errors += 1
                continue
            if isinstance(obj, dict):
                out.append(to_reply(obj))


# -- status helpers ------------------------------------------------------------

def slots_of(status: Any) -> List[Dict[str, Any]]:
    if not isinstance(status, dict):
        return []
    slots = status.get("slots")
    if not isinstance(slots, list):
        return []
    return [s if isinstance(s, dict) else {} for s in slots]


def slot_status(status: Any, index: int) -> str:
    slots = slots_of(status)
    if 0 <= index < len(slots):
        return str(slots[index].get("status", "") or "")
    return ""


def slot_error(status: Any, index: int) -> Optional[str]:
    """The slot's firmware error verdict ('feed_error', ...) or None."""
    slots = slots_of(status)
    if not 0 <= index < len(slots):
        return None
    for key in ("status", "slot_status"):
        value = slots[index].get(key)
        if isinstance(value, str) and value.endswith("_error"):
            return value
    return None


def unit_busy(status: Any) -> bool:
    return isinstance(status, dict) and status.get("status") == UNIT_BUSY


def slot_moving(status: Any, index: int) -> bool:
    """
    True while the ACE moves filament for this slot. V1 reports the motion in
    the slot's own status ('feeding', 'unwinding', ...); the unit-wide
    'busy' covers moves whose slot status lags.
    """
    st = slot_status(status, index)
    if st in SLOT_MOVING:
        return True
    action = status.get("action") if isinstance(status, dict) else None
    return unit_busy(status) and action in ("feeding", "unwinding", "shifting")


def presence(status: Any, index: int) -> Optional[bool]:
    """
    True/False when the slot clearly reports a spool / no spool, None when the
    reading can't be trusted (moving, error, unit busy, missing). The slot
    flickers 'empty' while the unit is busy, so that doesn't count.
    """
    st = slot_status(status, index)
    if st == SLOT_READY:
        return True
    if st == SLOT_EMPTY and not unit_busy(status):
        return False
    return None


def rgb_to_hex(color: Any) -> Optional[str]:
    if isinstance(color, (list, tuple)) and len(color) >= 3:
        try:
            r, g, b = (max(0, min(255, int(c))) for c in color[:3])
        except (TypeError, ValueError):
            return None
        return "{:02x}{:02x}{:02x}".format(r, g, b)
    return None


def slot_tag(status: Any, index: int) -> Optional[Dict[str, Any]]:
    """
    What the ACE read from an Anycubic RFID tag in this slot, or None if it
    has not identified one. {material, color (hex, no #), brand, sku}
    """
    slots = slots_of(status)
    if not 0 <= index < len(slots):
        return None
    slot = slots[index]
    try:
        rfid = int(slot.get("rfid", RFID_NONE) or 0)
    except (TypeError, ValueError):
        return None
    if rfid != RFID_IDENTIFIED:
        return None
    material = str(slot.get("type", "") or "").strip()
    color = rgb_to_hex(slot.get("color"))
    if color == "000000" and not material:
        color = None
    if not material and not color:
        return None
    return {"material": material or None, "color": color,
            "brand": slot.get("brand") or None, "sku": slot.get("sku") or None}


def dryer_of(status: Any) -> Dict[str, Any]:
    if isinstance(status, dict) and isinstance(status.get("dryer_status"), dict):
        return status["dryer_status"]
    if isinstance(status, dict) and isinstance(status.get("dryer"), dict):
        return status["dryer"]
    return {}


def clamp_speed(speed: float) -> float:
    return max(1.0, min(float(speed), MAX_SPEED))
