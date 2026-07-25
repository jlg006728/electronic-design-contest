from pathlib import Path


BACKUP_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "backup_code"
    / "20260723_v99_first_junction_gate_empty.c"
)


def test_v99_delays_first_junction_rearm() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "#define FIRMWARE_VERSION            99U" in source
    assert "#define FIRST_JUNCTION_MIN_TRAVEL_FRAMES 180U" in source
    assert "ROUTE_EVENT_CONFIG_INITIALIZER;" in source
    assert "FIRST_JUNCTION_ROUTE_EVENT_CONFIG" in source
    assert "(g_route_action_index == 0U)" in source
    assert "event_config->min_travel_frames" not in source
    assert "route_event_step(\n        event_config," in source


def test_v99_keeps_v98_feedback_fix_and_single_turn_stop() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "#define LINE_SENSOR_ERROR_SIGN      (-1)" in source
    assert (
        "LINE_SENSOR_ERROR_SIGN * (int32_t) g_gray_line_error"
        in source
    )
    assert (
        "LINE_SENSOR_ERROR_SIGN * (int32_t) g_line_last_nonzero_error"
        in source
    )
    assert "#define SINGLE_TURN_TEST_ENABLE     1U" in source
    assert "g_single_turn_test_complete = 1U;" in source
    assert "motors_stop();" in source
