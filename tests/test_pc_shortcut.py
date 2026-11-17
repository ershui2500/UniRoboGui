#!/usr/bin/env python3
"""Desktop generation and lifecycle tests without touching the real desktop or robots."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shlex
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("shortcut", ROOT / "scripts/pc_shortcut.py")
shortcut = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shortcut)


class ShortcutTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="PC 客户 ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.args = argparse.Namespace(build_dir=self.root / "build with spaces", port=18080,
                                       mock=True, a2_joint_gains=None)
        self.environment = patch.dict(os.environ, XDG_DATA_HOME=str(self.root / "data"),
                                      XDG_CACHE_HOME=str(self.root / "cache"))
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def test_create_repeat_and_argument_roundtrip(self):
        desktop = self.root / "桌面"
        desktop.mkdir()
        gains = self.root / 'operator $x`cmd`"% gains.json'
        gains.write_text("{}")
        self.args.a2_joint_gains = gains
        with patch.object(shortcut.shutil, "which", return_value="/usr/bin/fake"), \
             patch.object(shortcut.subprocess, "run", return_value=Mock(returncode=0, stdout=str(desktop) + "\n")):
            shortcut.create(self.args)
            first = (desktop / "unirobogui-pc.desktop").read_text()
            shortcut.create(self.args)
        self.assertEqual(first, (desktop / "unirobogui-pc.desktop").read_text())
        self.assertEqual(first, (self.root / "data/applications/unirobogui-pc.desktop").read_text())
        self.assertIn("Terminal=true", first)
        line = next(line[5:] for line in first.splitlines() if line.startswith("Exec="))
        # Apply Desktop Entry string decoding, then its quoted Exec argument decoding.
        decoded = line.replace("\\\\", "\\").replace("%%", "%")
        values = shlex.split(decoded)
        values = [value.replace('\\$', '$').replace('\\`', '`') for value in values]
        self.assertEqual(values[-1], str(gains))
        self.assertIn(str(self.args.build_dir), values)
        self.assertIn("--mock", values)
        self.assertIn("launch", values)
        for value in ('a\\b', 'a"b', '$x`cmd`%', '中文 with spaces'):
            encoded = shortcut.desktop_argument(value).replace('\\\\', '\\').replace('%%', '%')
            actual = shlex.split(encoded)[0].replace('\\$', '$').replace('\\`', '`')
            self.assertEqual(actual, value)
        with self.assertRaises(ValueError):
            shortcut.desktop_argument("line\nbreak")

    def test_no_desktop_directory_is_created(self):
        with patch.object(shortcut.shutil, "which", return_value=None):
            with self.assertRaises(ValueError):
                shortcut.create(self.args)
        with patch.object(shortcut.shutil, "which", side_effect=lambda name: "/usr/bin/xdg-open" if name == "xdg-open" else None):
            shortcut.create(self.args)
        self.assertTrue((self.root / "data/applications/unirobogui-pc.desktop").exists())

    def test_existing_workbench_or_conflict_never_spawns(self):
        with patch.object(shortcut, "probe", return_value="ready"), \
             patch.object(shortcut, "open_browser") as browser, \
             patch.object(shortcut.subprocess, "Popen") as spawn:
            self.assertEqual(shortcut.launch(self.args), 0)
            browser.assert_called_once_with(18080)
            spawn.assert_not_called()
        with patch.object(shortcut, "probe", return_value="occupied"), \
             patch.object(shortcut.subprocess, "Popen") as spawn:
            with self.assertRaises(ValueError):
                shortcut.launch(self.args)
            spawn.assert_not_called()

    def test_successful_start_and_failure(self):
        child = Mock()
        child.poll.return_value = None
        child.wait.return_value = 0
        with patch.object(shortcut, "probe", side_effect=["absent", "ready"]), \
             patch.object(shortcut, "open_browser") as browser, \
             patch.object(shortcut.time, "sleep"), \
             patch.object(shortcut.subprocess, "Popen", return_value=child) as spawn:
            self.assertEqual(shortcut.launch(self.args), 0)
            self.assertEqual(spawn.call_count, 1)
            self.assertIn("--mock", spawn.call_args.args[0])
            browser.assert_called_once_with(18080)
            child.wait.assert_called_once()
        child.poll.return_value = 1
        child.wait.return_value = 1
        with patch.object(shortcut, "probe", return_value="absent"), \
             patch.object(shortcut, "open_browser") as browser, \
             patch.object(shortcut.subprocess, "Popen", return_value=child):
            self.assertEqual(shortcut.launch(self.args), 1)
            browser.assert_not_called()

    def test_duplicate_start_is_locked(self):
        with patch.object(shortcut.fcntl, "flock", side_effect=BlockingIOError), \
             patch.object(shortcut.subprocess, "Popen") as spawn:
            with self.assertRaises(ValueError):
                shortcut.launch(self.args)
            spawn.assert_not_called()

    def test_shutdown_refusal_keeps_terminal_attached(self):
        child = Mock()
        child.poll.return_value = None
        child.wait.side_effect = [KeyboardInterrupt, KeyboardInterrupt, 0]
        self.assertEqual(shortcut.wait_for_exit(child), 0)
        self.assertEqual(child.send_signal.call_count, 2)

    def test_probe_bypasses_proxy_and_checks_identity(self):
        response = Mock()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.read.return_value = json.dumps({"runtime_mode": "pc_sdk2"}).encode()
        connection = Mock()
        connection.__enter__ = Mock()
        connection.__exit__ = Mock()
        opener = Mock()
        opener.open.return_value = response
        with patch.object(shortcut.socket, "create_connection", return_value=connection), \
             patch.object(shortcut.urllib.request, "build_opener", return_value=opener), \
             patch.object(shortcut.urllib.request, "ProxyHandler") as proxy:
            self.assertEqual(shortcut.probe(18080), "ready")
            proxy.assert_called_once_with({})
            for invalid in (b'{}', b'[]', b'null', b'not json'):
                response.read.return_value = invalid
                self.assertEqual(shortcut.probe(18080), "occupied")
        with patch.object(shortcut.socket, "create_connection", side_effect=ConnectionRefusedError):
            self.assertEqual(shortcut.probe(18080), "absent")


if __name__ == "__main__":
    unittest.main()
