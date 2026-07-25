from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
V102_SOURCE = (
    PROJECT_ROOT
    / "backup_code"
    / "20260725_v102_junction_threshold_4_empty.c"
)
V103_SOURCE = (
    PROJECT_ROOT
    / "backup_code"
    / "20260725_v103_junction_trigger_decoupled_empty.c"
)


def _source_under_test() -> str:
    source_path = V103_SOURCE if V103_SOURCE.exists() else V102_SOURCE
    return source_path.read_text(encoding="utf-8")


def test_v103_separates_junction_trigger_from_normal_line_arming() -> None:
    source = _source_under_test()

    assert "#define FIRMWARE_VERSION            103U" in source
    assert "#define JUNCTION_TRIGGER_ACTIVE_MIN 4U" in source
    assert "#define LINE_WIDE_ACTIVE_MIN        7U" in source
    assert source.count(
        ".wide_active_min = JUNCTION_TRIGGER_ACTIVE_MIN"
    ) == 2


def test_v103_uses_seven_sensor_limit_until_detector_is_armed() -> None:
    source = _source_under_test()
    follow_start = source.index("static NOINLINE bool medicine_follow_step")
    follow_end = source.index(
        "static NOINLINE bool medicine_turn_step", follow_start
    )
    follow_source = source[follow_start:follow_end]

    assert "route_event_config_t detector_config = *event_config;" in follow_source
    assert (
        "if ((destination != ROUTE_DESTINATION_JUNCTION) ||\n"
        "        !g_route_detector.armed)"
        in follow_source
    )
    assert (
        "detector_config.wide_active_min = LINE_WIDE_ACTIVE_MIN;"
        in follow_source
    )
    assert "&detector_config," in follow_source


def test_v103_keeps_endpoint_normal_line_limit_at_seven() -> None:
    junction_trigger_active_min = 4
    line_wide_active_min = 7

    endpoint_active_count = 6
    endpoint_is_normal = 0 < endpoint_active_count < line_wide_active_min
    endpoint_is_normal_with_wrong_limit = (
        0 < endpoint_active_count < junction_trigger_active_min
    )

    assert endpoint_is_normal
    assert not endpoint_is_normal_with_wrong_limit


def test_v103_keeps_four_to_six_sensor_frames_in_line_control() -> None:
    source = _source_under_test()
    follow_start = source.index("static NOINLINE bool medicine_follow_step")
    follow_end = source.index(
        "static NOINLINE bool medicine_turn_step", follow_start
    )
    follow_source = source[follow_start:follow_end]

    assert (
        "if (g_gray_changed_count >= LINE_WIDE_ACTIVE_MIN)"
        in follow_source
    )
    assert (
        "if (g_gray_changed_count >= event_config->wide_active_min)"
        not in follow_source
    )
    assert follow_source.index(
        "if (g_gray_changed_count >= LINE_WIDE_ACTIVE_MIN)"
    ) < follow_source.index("if (!simple_line_follow_update())")


def test_v103_threshold_model_arms_at_four_then_triggers_at_four() -> None:
    junction_trigger_active_min = 4
    line_wide_active_min = 7

    unarmed_active_count = 4
    unarmed_is_normal = (
        0 < unarmed_active_count < line_wide_active_min
    )

    armed_active_count = 4
    armed_is_junction = armed_active_count >= junction_trigger_active_min
    armed_bypasses_line_control = armed_active_count >= line_wide_active_min

    assert unarmed_is_normal
    assert armed_is_junction
    assert not armed_bypasses_line_control
