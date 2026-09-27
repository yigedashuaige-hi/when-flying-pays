import ast
from pathlib import Path


COORDINATOR = (
    Path(__file__).resolve().parents[1] / "scripts" /
    "rotors_handoff_coordinator.py"
)


def function_source(name):
    source = COORDINATOR.read_text()
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return ast.get_source_segment(source, node)
    raise AssertionError("missing function: %s" % name)


def test_persistent_memory_registration_is_guarded_by_current_stable_evidence():
    text = function_source("remember_safe_site")
    assert "history_eligible" in text
    assert "site_preflight_ready" in text
    assert "site_memory.upsert" in text


def test_persistent_target_arrival_still_requires_fresh_preflight_revalidation():
    text = function_source("reposition_update")
    assert "site_preflight_ready" in text
    assert "mark_reached" in text


def test_failed_reposition_can_resume_only_from_current_revalidated_site():
    text = function_source("update")
    assert "TAKEOFF_REPOSITION_FAILED" in text
    assert "site_preflight_ready" in text
    assert "takeoff_approved:current_site_revalidated" in text


def test_persistent_navigation_does_not_add_a_cmd_vel_publisher():
    text = COORDINATOR.read_text()
    assert 'Publisher("/mecanum/cmd_vel"' not in text
    assert "/gazebo/set_model_state" not in text
