from pathlib import Path


BACKUP_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "backup_code"
    / "20260723_v97_single_turn_auto_stop_empty.c"
)


def test_first_turn_completion_exits_the_mission_loop() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")
    turn_start = source.index("static NOINLINE bool medicine_turn_step")
    turn_end = source.index(
        "static NOINLINE bool medicine_ward_stop_step", turn_start
    )
    turn_source = source[turn_start:turn_end]
    follow_start = source.index("static NOINLINE bool medicine_follow_step")
    follow_end = source.index("static NOINLINE void apply_turn_motion", follow_start)
    follow_source = source[follow_start:follow_end]

    assert "#define FIRMWARE_VERSION            97U" in source
    assert "#define SINGLE_TURN_TEST_ENABLE     1U" in source
    assert "volatile uint8_t g_single_turn_test_complete = 0U;" in source
    assert "(g_turn_count == 1U)" in source
    assert "g_single_turn_test_complete = 1U;" in source
    assert "(g_turn_count == 1U)" in turn_source
    assert "g_single_turn_test_complete = 1U;" in turn_source
    assert turn_source.index("medicine_mission_step(&g_mission, &input);") < (
        turn_source.index("g_single_turn_test_complete = 1U;")
    )
    assert "g_single_turn_test_complete = 1U;" not in follow_source
    assert "(g_single_turn_test_complete == 0U)" in source
    assert "if (g_single_turn_test_complete != 0U)" in source
    assert "g_stop_reason = STOP_REASON_TEST_COMPLETE;" in source


def test_single_turn_version_keeps_pharmacy_start_and_motor_stop() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")
    main_start = source.index("int main(void)")
    loop_exit_anchor = (
        "g_mission_elapsed_ms += MISSION_TICK_MS;\n"
        "    }\n\n"
    )
    post_loop_start = source.index(loop_exit_anchor, main_start) + len(
        loop_exit_anchor
    )
    idle_loop_start = source.index("    while (1) {", post_loop_start)
    post_loop_source = source[post_loop_start:idle_loop_start]

    assert "#define GRAY_FIXED_WHITE_BASELINE_MASK 0xFFU" in source
    assert "gray_use_fixed_white_baseline();" in source
    assert "while (g_gray_baseline_ready == 0U)" not in source
    assert "medicine_mission_step(&g_mission, &input);" in source
    assert post_loop_source.startswith("    if (g_mission.fault) {")
    assert post_loop_source.index("if (g_mission.fault)") < (
        post_loop_source.index("if (g_single_turn_test_complete != 0U)")
    )
    assert "motors_stop();" in post_loop_source
