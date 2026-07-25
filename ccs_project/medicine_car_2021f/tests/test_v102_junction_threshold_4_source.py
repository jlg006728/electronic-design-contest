from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
V101_SOURCE = (
    PROJECT_ROOT
    / "backup_code"
    / "20260723_v101_junction_latch_heading_hold_empty.c"
)
V102_SOURCE = (
    PROJECT_ROOT
    / "backup_code"
    / "20260725_v102_junction_threshold_4_empty.c"
)


def _source_under_test() -> str:
    source_path = V102_SOURCE if V102_SOURCE.exists() else V101_SOURCE
    return source_path.read_text(encoding="utf-8")


def test_v102_triggers_junction_at_four_active_sensors() -> None:
    source = _source_under_test()

    assert "#define FIRMWARE_VERSION            102U" in source
    assert source.count(".wide_active_min = 4U") == 2
    assert ".wide_active_min = 7U" not in source


def test_v102_preserves_one_frame_confirmation_and_configured_check() -> None:
    source = _source_under_test()

    assert "#define JUNCTION_WIDE_CONFIRM_FRAMES 1U" in source
    assert (
        "if (g_gray_changed_count >= event_config->wide_active_min)"
        in source
    )
