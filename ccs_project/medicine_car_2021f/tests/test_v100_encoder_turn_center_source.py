from pathlib import Path


BACKUP_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "backup_code"
    / "20260723_v100_encoder_turn_center_17p5cm_empty.c"
)


def test_v100_converts_measured_geometry_to_encoder_target() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "#define FIRMWARE_VERSION            100U" in source
    assert "#define GRAY_TO_TURN_CENTER_MM      175U" in source
    assert "#define ENCODER_RISING_EDGES_PER_CM 664U" not in source
    assert "GRAY_TO_TURN_CENTER_MM * ENCODER_RISING_EDGES_PER_CM" in source
    assert "JUNCTION_CENTER_TARGET_EDGES" in source
    assert "volatile uint32_t g_turn_center_target_edges" in source


def test_v100_centers_only_ninety_degree_junction_turns() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "medicine_action_needs_turn_centering" in source
    assert "kind == MEDICINE_ACTION_TURN_LEFT_90" in source
    assert "kind == MEDICINE_ACTION_TURN_RIGHT_90" in source
    assert "MEDICINE_ACTION_TURN_AROUND_180" not in source[
        source.index("static NOINLINE bool medicine_action_needs_turn_centering"):
        source.index("static NOINLINE void turn_center_publish_progress")
    ]
    assert "turn_center_begin();" in source
    assert "const uint16_t center_forward_ms =" in source
    assert "? UINT16_MAX" in source
    assert ": 0U;" in source


def test_v100_uses_encoder_progress_before_gyro_rotation() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")
    turn_start = source.index("static NOINLINE bool medicine_turn_step")
    turn_end = source.index("static NOINLINE bool medicine_ward_stop_step")
    turn_source = source[turn_start:turn_end]

    assert "route_progress_begin(" in source
    assert "route_progress_step(" in source
    assert "g_turn_center_average_edges >= JUNCTION_CENTER_TARGET_EDGES" in source
    assert "g_turn_controller.phase = GYRO_TURN_PHASE_SETTLE;" in source
    assert "turn_center_step()" in turn_source
    assert turn_source.index("turn_center_step()") < turn_source.index(
        "gyro_turn_step("
    )


def test_v100_preserves_v99_safety_and_single_turn_test() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "#define FIRST_JUNCTION_MIN_TRAVEL_FRAMES 180U" in source
    assert "(g_route_action_index == 0U)" in source
    assert "#define LINE_SENSOR_ERROR_SIGN      (-1)" in source
    assert "#define GRAY_FIXED_WHITE_BASELINE_MASK 0xFFU" in source
    assert "#define SINGLE_TURN_TEST_ENABLE     1U" in source
    assert "g_single_turn_test_complete = 1U;" in source
    assert "medicine_fail(STOP_REASON_ENCODER_FAULT);" in source
