import math

import pytest

from vision_core import (
    estimate_red_line,
    hamming_distance,
    rank_templates,
    vote_labels,
    crc8,
    encode_packet,
)


def test_estimate_red_line_uses_multiple_bands_and_reports_offsets():
    bands = [
        (40, 128, 20),
        (80, 138, 18),
        (120, 148, 24),
        (160, 158, 21),
        (200, 168, 19),
    ]

    result = estimate_red_line(bands, image_width=320, image_height=240)

    assert result.valid is True
    assert result.used_bands == 5
    assert result.near_x == pytest.approx(178.0, abs=1.0)
    assert result.far_x == pytest.approx(128.0, abs=1.0)
    assert result.near_offset == pytest.approx(18.0, abs=1.0)
    assert result.far_offset == pytest.approx(-32.0, abs=1.0)
    assert result.angle_deg == pytest.approx(math.degrees(math.atan2(50, 200)), abs=0.5)


def test_estimate_red_line_rejects_insufficient_bands():
    result = estimate_red_line([(100, 140, 20)], image_width=320, image_height=240)
    assert result.valid is False
    assert result.used_bands == 1


def test_hamming_and_template_ranking_are_deterministic():
    observed = "10110010"
    templates = {"1": "10110011", "2": "00110010", "3": "11111111"}

    assert hamming_distance(observed, templates["1"]) == 1
    ranked = rank_templates(observed, templates)
    assert ranked[0] == ("1", 1)
    assert ranked[1] == ("2", 1)
    assert ranked[2] == ("3", 4)


def test_vote_labels_requires_margin_and_returns_stable_winner():
    assert vote_labels(["1", "1", "2", "1", "2"], min_votes=3, min_margin=1) == ("1", 3, 1)
    assert vote_labels(["1", "2", "1", "2"], min_votes=3, min_margin=1) == (None, 2, 0)
    assert vote_labels([None, "1", None], min_votes=2, min_margin=1) == (None, 1, 1)


def test_packet_has_crc8_and_machine_readable_fields():
    packet = encode_packet("RED", {"valid": 1, "near": 178})
    assert packet.startswith("$RED,valid=1,near=178,")
    assert packet.endswith("\r\n")
    body = packet[1:packet.index(",crc=")]
    expected_crc = crc8(body.encode("ascii"))
    assert packet.split(",crc=")[1].split("*")[0] == f"{expected_crc:02X}"
