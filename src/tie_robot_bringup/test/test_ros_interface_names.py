#!/usr/bin/env python3
import pathlib
import subprocess
import sys
import tempfile
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
CHECKER = REPO_ROOT / "scripts" / "check_ros_interface_names.py"


class RosInterfaceNamesTest(unittest.TestCase):
    def run_checker(self, *args):
        return subprocess.run(
            [sys.executable, str(CHECKER), *args],
            cwd=str(REPO_ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

    def test_migration_map_covers_current_ros_interface_literals(self):
        result = self.run_checker("--quiet")
        self.assertEqual(result.returncode, 0, result.stdout)

    def test_checker_rejects_unmapped_legacy_interface_literals(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            sample = pathlib.Path(tmpdir) / "bad_node.py"
            sample.write_text('BAD_TOPIC = "/web/moduan/new_bad_legacy_name"\\n')

            result = self.run_checker("--quiet", "--roots", tmpdir)

        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("/web/moduan/new_bad_legacy_name", result.stdout)


if __name__ == "__main__":
    unittest.main()
