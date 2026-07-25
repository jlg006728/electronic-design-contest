from pathlib import Path


BACKUP_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "backup_code"
    / "20260723_v96_pharmacy_red_line_start_empty.c"
)


def test_pharmacy_start_uses_fixed_white_baseline() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert "#define FIRMWARE_VERSION            96U" in source
    assert "#define GRAY_FIXED_WHITE_BASELINE_MASK 0xFFU" in source
    assert "static NOINLINE void gray_use_fixed_white_baseline(void)" in source
    assert (
        "g_gray_white_baseline_mask = GRAY_FIXED_WHITE_BASELINE_MASK;"
        in source
    )
    assert "g_gray_baseline_uniform = 1U;" in source
    assert "g_gray_white_level_code = 1U;" in source
    assert "g_gray_baseline_ready = 1U;" in source
    assert "gray_use_fixed_white_baseline();" in source
    assert "while (g_gray_baseline_ready == 0U)" not in source


def test_pharmacy_start_keeps_route_and_encoder_calibration() -> None:
    source = BACKUP_SOURCE.read_text(encoding="utf-8")

    assert '#include "encoder_calibration_profile.h"' in source
    assert "medicine_mission_init(&g_mission);" in source
    assert "encoder_init();" in source
    assert "mpu_calibrate_gyro_z()" in source
