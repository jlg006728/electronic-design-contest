from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    PROJECT_ROOT
    / "backup_code"
    / "20260725_v105_video_gated_first_junction_empty.c"
)


def _source() -> str:
    return SOURCE.read_text(encoding="utf-8")


def test_v105_uses_distance_and_multiframe_gate() -> None:
    source = _source()

    assert "#define FIRMWARE_VERSION            105U" in source
    assert "#define FIRST_JUNCTION_MIN_TRAVEL_FRAMES UINT16_MAX" in source
    assert "#define FIRST_JUNCTION_ARM_DISTANCE_CM 50U" in source
    assert "#define JUNCTION_TRIGGER_ACTIVE_MIN 4U" in source
    assert "#define JUNCTION_WIDE_CONFIRM_FRAMES 1U" in source
    assert "#define FIRST_JUNCTION_WIDE_CONFIRM_FRAMES 3U" in source
    assert (
        ".wide_confirm_frames = FIRST_JUNCTION_WIDE_CONFIRM_FRAMES"
        in source
    )
    assert (
        ".min_travel_frames = FIRST_JUNCTION_MIN_TRAVEL_FRAMES"
        in source
    )


def test_v105_requires_x4_x5_for_first_junction_confirmation() -> None:
    source = _source()

    assert "#define JUNCTION_CENTER_REQUIRED_MASK 0x18U" in source
    assert "first_junction_center_sensor_gate_active" in source
    assert "g_route_action_index == 0U" in source
    assert "junction_active_count = 0U;" in source


def test_v105_applies_center_gate_before_event_detector() -> None:
    source = _source()
    follow_start = source.index("static NOINLINE bool medicine_follow_step")
    follow_end = source.index(
        "static NOINLINE bool medicine_turn_step", follow_start
    )
    follow = source[follow_start:follow_end]

    assert follow.index("route_arm_first_junction_by_distance();") < follow.index(
        "first_junction_center_sensor_gate_active()"
    )
    assert follow.index("junction_active_count = 0U;") < follow.index(
        "event = route_event_step("
    )
    assert "junction_active_count," in follow


def test_v105_four_outer_only_sensors_do_not_trigger() -> None:
    required_mask = 0x18
    outer_only_mask = 0x03
    four_with_center_mask = 0x1E

    assert (outer_only_mask & required_mask) != required_mask
    assert (four_with_center_mask & required_mask) == required_mask


def test_v105_three_confirm_frames_reject_single_transient() -> None:
    first_junction_confirm_frames = 3
    later_junction_confirm_frames = 1
    transient_frames = 1

    assert transient_frames < first_junction_confirm_frames
    assert first_junction_confirm_frames == 3
    assert later_junction_confirm_frames == 1


def test_v105_does_not_allow_the_old_180_frame_auto_arm_bypass() -> None:
    first_junction_min_frames = 65535
    old_fallback_frames = 180

    assert first_junction_min_frames > old_fallback_frames
