"""Pure-Python algorithms shared by the OpenMV validation script and tests.

The board script uses only small integer operations where possible.  This file
keeps the geometry and decision logic independent from OpenMV's image API so
it can be regression-tested on a desktop.
"""

from collections import Counter
from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class RedLineEstimate:
    valid: bool
    slope: float
    intercept: float
    far_x: float
    near_x: float
    far_offset: float
    near_offset: float
    angle_deg: float
    used_bands: int


def estimate_red_line(
    bands: Iterable[Tuple[float, float, int]],
    image_width: int,
    image_height: int,
    min_pixels: int = 1,
) -> RedLineEstimate:
    """Fit ``x = slope*y + intercept`` to red centers from horizontal bands.

    A band tuple is ``(y, center_x, red_pixel_count)``.  The line is projected
    to the top-most observed band and to the bottom image edge.  Offsets are
    relative to the image center; a positive slope/angle means the line moves
    right toward the near (bottom) side.
    """

    points = [(float(y), float(x)) for y, x, pixels in bands if pixels >= min_pixels]
    if len(points) < 2:
        return RedLineEstimate(False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, len(points))

    mean_y = sum(y for y, _ in points) / len(points)
    mean_x = sum(x for _, x in points) / len(points)
    denom = sum((y - mean_y) ** 2 for y, _ in points)
    if denom <= 1e-9:
        return RedLineEstimate(False, 0.0, mean_x, mean_x, mean_x, 0.0, 0.0, 0.0, len(points))

    slope = sum((y - mean_y) * (x - mean_x) for y, x in points) / denom
    intercept = mean_x - slope * mean_y
    far_y = min(y for y, _ in points)
    far_x = slope * far_y + intercept
    near_x = slope * float(image_height) + intercept
    center_x = (image_width - 1) / 2.0
    angle_deg = math.degrees(math.atan2(near_x - far_x, float(image_height) - far_y))
    return RedLineEstimate(
        True,
        slope,
        intercept,
        far_x,
        near_x,
        far_x - center_x,
        near_x - center_x,
        angle_deg,
        len(points),
    )


def hamming_distance(left: str, right: str) -> int:
    """Return bit mismatch count; reject signatures with different lengths."""

    if len(left) != len(right):
        raise ValueError("signatures must have equal length")
    return sum(a != b for a, b in zip(left, right))


def rank_templates(observed: str, templates: Dict[str, str]) -> List[Tuple[str, int]]:
    """Rank template labels by Hamming distance, preserving insertion order on ties."""

    ranked = [(label, hamming_distance(observed, signature)) for label, signature in templates.items()]
    ranked.sort(key=lambda item: item[1])
    return ranked


def vote_labels(
    labels: Sequence[Optional[str]],
    min_votes: int = 3,
    min_margin: int = 1,
) -> Tuple[Optional[str], int, int]:
    """Return ``(winner, winner_votes, top_minus_second)``.

    ``winner`` is ``None`` until both the vote count and the separation from
    the runner-up pass their thresholds.  ``None`` observations are ignored.
    """

    counts = Counter(label for label in labels if label is not None)
    if not counts:
        return None, 0, 0
    ordered = counts.most_common()
    top_label, top_count = ordered[0]
    second_count = ordered[1][1] if len(ordered) > 1 else 0
    margin = top_count - second_count
    if top_count < min_votes or margin < min_margin:
        return None, top_count, margin
    return top_label, top_count, margin


def crc8(data: bytes, polynomial: int = 0x07) -> int:
    """CRC-8/ATM used for short OpenMV UART debug frames."""

    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ polynomial) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_packet(kind: str, fields: Dict[str, object]) -> str:
    """Encode a deterministic ASCII frame: ``$KIND,k=v,...,crc=XX*\\r\\n``."""

    body = ",".join([kind] + [f"{key}={value}" for key, value in fields.items()])
    return f"${body},crc={crc8(body.encode('ascii')):02X}*\r\n"
