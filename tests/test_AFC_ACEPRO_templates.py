"""The example config parses, names only options the code reads, and wires
lanes to the unit."""
from __future__ import annotations

import configparser
import os
import re

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(HERE)
TEMPLATE = os.path.join(ROOT, "templates", "AFC_ACEPRO_1.cfg")
UNIT_SRC = os.path.join(ROOT, "extras", "AFC_ACEPRO.py")


def _parse():
    cp = configparser.RawConfigParser(inline_comment_prefixes=("#",), strict=False)
    with open(TEMPLATE) as f:
        cp.read_string(f.read())
    return cp


def test_template_parses():
    cp = _parse()
    assert cp.has_section("AFC_ACEPRO ACE_1")
    assert cp.has_section("AFC_hub ACE_1")
    lanes = [s for s in cp.sections() if s.startswith("AFC_ACEPRO_lane ")]
    assert len(lanes) == 4
    assert [cp.get(s, "unit") for s in lanes] == ["ACE_1:{}".format(i) for i in range(1, 5)]


def test_every_option_in_template_is_read_by_the_code():
    src = open(UNIT_SRC).read()
    read = set(re.findall(r'config\.get\w*\(\s*"(\w+)"', src))
    afc_unit_options = {"hub", "extruder"}
    with open(TEMPLATE) as f:
        text = f.read()
    section = text.split("[AFC_ACEPRO ACE_1]")[1].split("[AFC_hub")[0]
    named = set(re.findall(r"^#?(\w+):", section, flags=re.M))
    unknown = named - read - afc_unit_options
    assert not unknown, "template names options the unit never reads: {}".format(unknown)
