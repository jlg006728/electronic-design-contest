from pathlib import Path


BACKUP_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "backup_code"
    / "20260723_v101_junction_latch_heading_hold_empty.c"
)


def test_v101_uses_one_frame_for_all_junction_events() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "#define FIRMWARE_VERSION            101U" in source
    assert "#define JUNCTION_WIDE_CONFIRM_FRAMES 1U" in source
    assert source.count(
        ".wide_confirm_frames = JUNCTION_WIDE_CONFIRM_FRAMES"
    ) == 2


def test_v101_latches_encoder_at_event_before_active_brake() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")
    event_start = source.index("if (event != ROUTE_EVENT_NONE)")
    event_end = source.index(
        "if (g_gray_changed_count >= event_config->wide_active_min)",
        event_start,
    )
    event_source = source[event_start:event_end]

    latch_index = event_source.index("turn_center_latch_junction_event();")
    brake_index = event_source.index("motors_brake();")
    mission_index = event_source.index("medicine_mission_step(&g_mission, &input);")

    assert latch_index < brake_index < mission_index
    assert "g_turn_center_event_latched = 1U;" in source
    assert "g_turn_center_event_left_count" in source
    assert "g_turn_center_event_right_count" in source


def test_v101_consumes_event_counts_for_the_17p5cm_target() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")
    begin_start = source.index("static NOINLINE void turn_center_begin")
    begin_end = source.index("static NOINLINE bool turn_center_step", begin_start)
    begin_source = source[begin_start:begin_end]

    assert "if (g_turn_center_event_latched != 0U)" in begin_source
    assert "left_count = g_turn_center_event_left_count;" in begin_source
    assert "right_count = g_turn_center_event_right_count;" in begin_source
    assert "g_turn_center_event_latched = 0U;" in begin_source
    assert "JUNCTION_CENTER_TARGET_EDGES" in begin_source


def test_v101_holds_heading_during_encoder_centering() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")
    turn_start = source.index("static NOINLINE bool medicine_turn_step")
    turn_end = source.index("static NOINLINE bool medicine_ward_stop_step")
    turn_source = source[turn_start:turn_end]

    assert "#define JUNCTION_CENTER_HEADING_TRIM_DIVISOR 250" in source
    assert "#define JUNCTION_CENTER_HEADING_TRIM_MAX 60" in source
    assert "static NOINLINE int32_t turn_center_heading_trim" in source
    assert "static NOINLINE void apply_turn_center_forward" in source
    assert "g_turn_controller.phase == GYRO_TURN_PHASE_CENTER" in turn_source
    assert "g_turn_yaw_mdeg +=" in turn_source
    assert "output.motion == GYRO_TURN_MOTION_FORWARD" in turn_source
    assert "apply_turn_center_forward();" in turn_source


def test_v101_preserves_v100_geometry_and_single_turn_stop() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "#define GRAY_TO_TURN_CENTER_MM      175U" in source
    assert "JUNCTION_CENTER_TARGET_EDGES" in source
    assert "#define FIRST_JUNCTION_MIN_TRAVEL_FRAMES 180U" in source
    assert "#define LINE_SENSOR_ERROR_SIGN      (-1)" in source
    assert "#define SINGLE_TURN_TEST_ENABLE     1U" in source
    assert "g_single_turn_test_complete = 1U;" in source


def test_v101_latched_progress_counts_post_event_coast() -> None:
    event_left_count = 0
    event_right_count = 0
    post_coast_left_count = -120
    post_coast_right_count = 100

    left_forward_edges = event_left_count - post_coast_left_count
    right_forward_edges = post_coast_right_count - event_right_count
    average_forward_edges = (
        left_forward_edges + right_forward_edges
    ) // 2

    assert left_forward_edges == 120
    assert right_forward_edges == 100
    assert average_forward_edges == 110


def test_v101_heading_trim_corrects_yaw_and_wheel_error_direction() -> None:
    def heading_trim(yaw_mdeg: int, left_edges: int, right_edges: int) -> int:
        trim = int(yaw_mdeg / 250)
        trim += int((right_edges - left_edges) / 64)
        return max(-60, min(60, trim))

    assert heading_trim(5000, 1000, 1000) == 20
    assert heading_trim(-5000, 1000, 1000) == -20
    assert heading_trim(0, 1000, 1640) == 10
    assert heading_trim(0, 1640, 1000) == -10
    assert heading_trim(50000, 0, 50000) == 60
