from pathlib import Path


BACKUP_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "backup_code"
    / "20260723_v98_line_feedback_sign_fix_single_turn_empty.c"
)
V97_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "backup_code"
    / "20260723_v97_single_turn_auto_stop_empty.c"
)


def test_v98_reverses_current_and_lost_line_error_at_adapter() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")
    follow_start = source.index("static NOINLINE bool simple_line_follow_update")
    follow_end = source.index("static NOINLINE void gray_update_led", follow_start)
    follow_source = source[follow_start:follow_end]

    assert "#define FIRMWARE_VERSION            98U" in source
    assert "#define LINE_SENSOR_ERROR_SIGN      (-1)" in source
    assert (
        "LINE_SENSOR_ERROR_SIGN * (int32_t) g_gray_line_error"
        in follow_source
    )
    assert (
        "LINE_SENSOR_ERROR_SIGN * (int32_t) g_line_last_nonzero_error"
        in follow_source
    )


def test_v98_keeps_v97_route_start_and_single_turn_stop() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "#define GRAY_FIXED_WHITE_BASELINE_MASK 0xFFU" in source
    assert "#define SINGLE_TURN_TEST_ENABLE     1U" in source
    assert "volatile uint8_t g_single_turn_test_complete = 0U;" in source
    assert "(g_turn_count == 1U)" in source
    assert "g_single_turn_test_complete = 1U;" in source
    assert "if (g_mission.fault) {" in source
    assert "if (g_single_turn_test_complete != 0U)" in source
    assert "g_stop_reason = STOP_REASON_TEST_COMPLETE;" in source
    assert "motors_stop();" in source


def test_v98_diff_is_limited_to_version_and_feedback_adapter() -> None:
    v97_source = V97_SOURCE.read_text(encoding="utf-8")
    v98_source = BACKUP_SOURCE.read_text(encoding="utf-8")
    expected = v97_source.replace(
        "#define FIRMWARE_VERSION            97U\n",
        "#define FIRMWARE_VERSION            98U\n",
        1,
    ).replace(
        "#define SINGLE_TURN_TEST_ENABLE     1U\n",
        "#define SINGLE_TURN_TEST_ENABLE     1U\n"
        "#define LINE_SENSOR_ERROR_SIGN      (-1)\n",
        1,
    ).replace(
        "        g_gray_line_error,\n"
        "        g_line_last_nonzero_error,\n",
        "        LINE_SENSOR_ERROR_SIGN * (int32_t) g_gray_line_error,\n"
        "        LINE_SENSOR_ERROR_SIGN * (int32_t) g_line_last_nonzero_error,\n",
        1,
    )

    assert v98_source == expected
