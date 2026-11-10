from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "windows_demo_launcher.sh"
SESSION = ROOT / "scripts" / "windows_demo_session.sh"


def body(source, name):
    match = re.search(r"(?ms)^" + re.escape(name) + r"\(\) \{\n(.*?)^\}", source)
    if not match:
        raise AssertionError(f"function not found: {name}")
    return match.group(1)


class WindowsDemoResourcePolicyTests(unittest.TestCase):
    def setUp(self):
        self.launcher = LAUNCHER.read_text()
        self.session = SESSION.read_text()

    def test_active_codex_and_user_apps_are_never_closed_automatically(self):
        offer = body(self.launcher, "offer_close_apps")
        self.assertNotIn("close_approved_apps", self.launcher)
        self.assertNotIn("CloseMainWindow", self.launcher)
        self.assertNotIn("Stop-Process", self.launcher)
        self.assertIn("Preserving the active ChatGPT/Codex execution", offer)
        self.assertIn("No desktop app is closed automatically", offer)
        self.assertNotIn("read -r", offer)

    def test_project_preflight_orders_snapshot_cleanup_recheck_and_gate(self):
        prepare = body(self.launcher, "prepare_launch")
        snapshot_positions = [m.start() for m in re.finditer("show_resource_snapshot", prepare)]
        self.assertGreaterEqual(len(snapshot_positions), 2)
        positions = [
            snapshot_positions[0],
            prepare.index("offer_close_apps"),
            prepare.index('bash "$SESSION_SCRIPT" stop'),
            prepare.index("print_preflight_state"),
            snapshot_positions[1],
            prepare.index('WINDOWS_AVAILABLE_MIB" -lt 2560'),
        ]
        self.assertEqual(positions, sorted(positions))

    def test_diagnostics_uses_the_protected_process_inventory(self):
        diagnostics = body(self.launcher, "diagnostics")
        self.assertIn("protected_codex_inventory", diagnostics)
        self.assertNotIn("host_app_candidates", diagnostics)

    def test_gui_rviz_and_disk_resource_limits_remain_enforced(self):
        prepare = body(self.launcher, "prepare_launch")
        self.assertIn('WSL_AVAILABLE_MIB" -lt 1536', prepare)
        self.assertIn('WINDOWS_AVAILABLE_MIB" -lt 3328', prepare)
        self.assertIn('DISK_AVAILABLE_MIB" -lt 3072', prepare)

    def test_tracked_project_stop_uses_graceful_interrupt_only(self):
        stop = body(self.session, "stop_tracked_session")
        self.assertIn('kill -INT -- "-$pgid"', stop)
        self.assertNotIn("SIGTERM", stop)
        self.assertNotIn("SIGKILL", stop)
        self.assertIn("live PID/group/command identity", stop)


if __name__ == "__main__":
    unittest.main(verbosity=2)
