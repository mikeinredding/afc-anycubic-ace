"""Unit tests for the ACE Pro wire format and status helpers."""
from __future__ import annotations

import json
import struct

import pytest

from extras import AFC_ACEPRO_protocol as P


def reply_frame(obj) -> bytes:
    payload = json.dumps(obj).encode()
    return (P.FRAME_HEAD + struct.pack("<H", len(payload)) + payload
            + struct.pack("<H", P.crc16(payload)) + P.FRAME_TAIL)


class TestCrc:
    def test_check_value(self):
        # CRC-16/MCRF4XX catalogue check value
        assert P.crc16(b"123456789") == 0x6F91

    def test_empty(self):
        assert P.crc16(b"") == 0xFFFF


class TestEncode:
    def test_frame_layout(self):
        payload = P.encode_payload(7, "get_status")
        frame = P.encode_frame(payload)
        assert frame[:2] == b"\xff\xaa"
        assert struct.unpack("<H", frame[2:4])[0] == len(payload)
        assert frame[4:4 + len(payload)] == payload
        assert struct.unpack("<H", frame[-3:-1])[0] == P.crc16(payload)
        assert frame[-1:] == b"\xfe"

    def test_params_omitted_when_none(self):
        assert json.loads(P.encode_payload(1, "get_info")) == {"id": 1, "method": "get_info"}

    def test_params_included(self):
        obj = json.loads(P.encode_payload(2, "feed_filament", {"index": 0, "length": 10}))
        assert obj["params"] == {"index": 0, "length": 10}

    def test_refuses_oversize(self):
        with pytest.raises(ValueError):
            P.encode_frame(b"x" * (P.MAX_REQUEST_PAYLOAD + 1))

    def test_frame_is_safe(self):
        assert P.frame_is_safe(b"\xff\xaa\x01\x00x\x00\x00\xfe")
        assert not P.frame_is_safe(b"\xff\xaa\x01\x00x\xff\xaa\xfe")


class TestParser:
    def test_roundtrip(self):
        parser = P.FrameParser()
        out = parser.feed(reply_frame({"id": 3, "code": 0, "msg": "success", "result": {"a": 1}}))
        assert len(out) == 1
        assert out[0].id == 3 and out[0].ok and out[0].result == {"a": 1}

    def test_split_across_chunks(self):
        parser = P.FrameParser()
        frame = reply_frame({"id": 1, "code": 0, "msg": "", "result": None})
        got = []
        for i in range(len(frame)):
            got += parser.feed(frame[i:i + 1])
        assert [r.id for r in got] == [1]

    def test_garbage_and_false_header(self):
        parser = P.FrameParser()
        data = (b"\x00\x13\xff\xaa\x05\x00junk\xfe" + reply_frame({"id": 9, "code": 0})
                + b"\xff" + reply_frame({"id": 10, "code": 0}))
        assert [r.id for r in parser.feed(data)] == [9, 10]
        assert parser.crc_errors >= 1

    def test_oversize_length_skipped(self):
        parser = P.FrameParser()
        data = b"\xff\xaa\xff\xff" + reply_frame({"id": 4, "code": 0})
        assert [r.id for r in parser.feed(data)] == [4]

    def test_bad_crc_dropped(self):
        parser = P.FrameParser()
        frame = bytearray(reply_frame({"id": 5, "code": 0}))
        frame[-2] ^= 0xFF
        assert parser.feed(bytes(frame)) == []
        assert parser.crc_errors == 1

    def test_bad_json_counted(self):
        parser = P.FrameParser()
        payload = b"{nope"
        frame = (P.FRAME_HEAD + struct.pack("<H", len(payload)) + payload
                 + struct.pack("<H", P.crc16(payload)) + P.FRAME_TAIL)
        assert parser.feed(frame) == []
        assert parser.json_errors == 1

    def test_forbidden_is_not_ok(self):
        r = P.to_reply({"id": 1, "code": 0, "msg": "FORBIDDEN"})
        assert not r.ok and r.forbidden

    def test_error_code_not_ok(self):
        assert not P.to_reply({"id": 1, "code": 2, "msg": "error_2"}).ok


def status(slots, unit="ready", action=""):
    return {"status": unit, "action": action, "slots": [
        {"index": i, "status": s} if isinstance(s, str) else dict(index=i, **s)
        for i, s in enumerate(slots)]}


class TestStatus:
    def test_presence_ready_and_empty(self):
        st = status(["ready", "empty", "feeding", "feed_error"])
        assert P.presence(st, 0) is True
        assert P.presence(st, 1) is False
        assert P.presence(st, 2) is None       # moving: unknown
        assert P.presence(st, 3) is None       # error: unknown
        assert P.presence(st, 9) is None

    def test_empty_while_busy_is_unknown(self):
        st = status(["feeding", "empty"], unit="busy", action="feeding")
        assert P.presence(st, 1) is None

    def test_moving(self):
        st = status(["feeding", "ready", "unwinding", "ready"])
        assert P.slot_moving(st, 0) and P.slot_moving(st, 2)
        assert not P.slot_moving(st, 1)

    def test_busy_counts_as_moving(self):
        assert P.slot_moving(status(["ready"], unit="busy", action="feeding"), 0)

    def test_assist_keeps_presence(self):
        # A slot feeding the print through feed assist still reads present
        st = status(["ready"])
        st["feed_assist_count"] = 3
        assert P.presence(st, 0) is True and not P.slot_moving(st, 0)

    def test_slot_error(self):
        st = status(["feed_error", {"status": "ready", "slot_status": "rollback_error"}, "ready"])
        assert P.slot_error(st, 0) == "feed_error"
        assert P.slot_error(st, 1) == "rollback_error"
        assert P.slot_error(st, 2) is None

    def test_tag(self):
        st = status([{"status": "ready", "rfid": 2, "type": "PLA", "color": [255, 0, 16],
                      "brand": "Anycubic", "sku": "X"},
                     {"status": "ready", "rfid": 3, "type": "PLA", "color": [1, 2, 3]},
                     {"status": "ready", "rfid": 2, "type": "", "color": [0, 0, 0]}])
        assert P.slot_tag(st, 0) == {"material": "PLA", "color": "ff0010",
                                     "brand": "Anycubic", "sku": "X"}
        assert P.slot_tag(st, 1) is None       # still identifying
        assert P.slot_tag(st, 2) is None       # nothing useful

    def test_rgb(self):
        assert P.rgb_to_hex([300, -4, 15]) == "ff000f"
        assert P.rgb_to_hex("red") is None

    def test_dryer(self):
        assert P.dryer_of({"dryer_status": {"status": "drying"}}) == {"status": "drying"}
        assert P.dryer_of({}) == {}

    def test_clamp_speed(self):
        assert P.clamp_speed(500) == P.MAX_SPEED
        assert P.clamp_speed(0) == 1.0
