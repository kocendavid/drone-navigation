"""Reset reload handoff preserves evidence without executing flight commands."""

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from navigation.runtime import RuntimeVersion, prepare_reload, restore_reload
from navigation.history import RunHistory
from test_history import snapshot


class RuntimeTests(unittest.TestCase):
    def test_running_version_does_not_claim_disk_changes_are_already_loaded(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "navigation").mkdir()
            (root / "config").mkdir()
            for name in (
                "ros2_navigate.py",
                "ros2_hover.py",
                "ros2_step.py",
                "config/navigation_settings.json",
            ):
                (root / name).write_text("original")
            version = RuntimeVersion(root)
            self.assertFalse(version.report()["reload_pending"])
            (root / "ros2_navigate.py").write_text("changed")
            version.next_check = 0
            self.assertTrue(version.report()["reload_pending"])
            self.assertNotEqual(version.report()["loaded"], version.report()["disk"])
            self.assertFalse(RuntimeVersion(root).report()["reload_pending"])

    def test_completed_reset_handoff_restores_full_frame_and_previous_run(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = root / "logs" / "experiment_test"
            archive.mkdir(parents=True)
            history = RunHistory(root / "logs" / "history.jsonl")
            history.observe(
                dict(phase="HOLD", reason="Recorded run", goal=[1, 1], delivered_simulation_s=5),
                snapshot(),
                100,
            )
            history.flush()
            summary = history.report()
            state = dict(phase="READY", busy=False, archive=str(archive))
            prepare_reload(root, state, history, "reset-123")
            hub = SimpleNamespace()
            context = restore_reload(root, hub)
            restored = hub.previous_history
            self.assertEqual(context["request_id"], "reset-123")
            self.assertEqual(restored.run_id, summary["run_id"])
            self.assertEqual(restored.report()["targets"], summary["targets"])
            self.assertEqual(restored.report()["trail"], summary["trail"])
            self.assertEqual(restored.frame(summary["frame_ids"][0])["grid"], "." * 400)
            self.assertEqual(restored.path, archive / "history.jsonl")
            self.assertTrue(restored.closed)
            self.assertIsNone(restore_reload(root, hub))
            self.assertTrue((archive / "history.jsonl").exists())

    def test_handoff_refuses_failed_reset_and_outside_archive(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "logs").mkdir()
            for state in (
                dict(phase="FAILED", busy=False, archive=str(root / "logs")),
                dict(phase="READY", busy=False, archive=str(root)),
            ):
                with self.assertRaises(ValueError):
                    prepare_reload(root, state, RunHistory(), None)
            self.assertFalse((root / "logs" / "navigator_reload.json").exists())
