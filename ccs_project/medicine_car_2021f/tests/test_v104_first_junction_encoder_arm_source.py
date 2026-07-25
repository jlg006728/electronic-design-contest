from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
V103_SOURCE = (
    PROJECT_ROOT
    / "backup_code"
    / "20260725_v103_junction_trigger_decoupled_empty.c"
)
V104_SOURCE = (
    PROJECT_ROOT
    / "backup_code"
    / "20260725_v104_first_junction_encoder_arm_empty.c"
)


def _source_under_test() -> str:
    source_path = V104_SOURCE if V104_SOURCE.exists() else V103_SOURCE
    return source_path.read_text(encoding="utf-8")


def test_v104_uses_distance_gate_for_first_junction_arm() -> None:
    source = _source_under_test()

    assert "#define FIRMWARE_VERSION            104U" in source
    assert "#define FIRST_JUNCTION_ARM_DISTANCE_CM 25U" in source
    assert "FIRST_JUNCTION_ARM_DISTANCE_CM * ENCODER_RISING_EDGES_PER_CM" in source
    assert "g_route_progress.average_forward_edges <" in source
    assert "FIRST_JUNCTION_ARM_DISTANCE_EDGES" in source


def test_v104_requires_normal_frames_before_encoder_arm() -> None:
    source = _source_under_test()

    assert "g_route_detector.normal_frames < 5U" in source
    assert "g_route_detector.armed = true;" in source
    assert "route_arm_first_junction_by_distance();" in source


def test_v104_rejects_pharmacy_start_distance_but_accepts_first_junction_distance() -> None:
    encoder_edges_per_cm = 664
    arm_distance_edges = 25 * encoder_edges_per_cm

    assert 10 * encoder_edges_per_cm < arm_distance_edges
    assert 60 * encoder_edges_per_cm > arm_distance_edges
