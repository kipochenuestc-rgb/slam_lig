import unittest,tempfile,pathlib
from unittest.mock import patch
from launch_rviz import graphics_state,main

class GraphicsStateTests(unittest.TestCase):
    def test_failed_shared_memory_is_detected(self):
        self.assertEqual(graphics_state('Input/output error\nRDP backend: use_gfxredir = 0\n'),0)
    def test_latest_restart_wins_over_historical_error(self):
        self.assertEqual(graphics_state('RDP backend: use_gfxredir = 0\nRDP backend: use_gfxredir = 1\n'),1)
    def test_unrelated_setting_does_not_report_failure(self):
        self.assertIsNone(graphics_state('RDP backend: enable_copy_warning_title = 1\n'))
    def test_copy_mode_is_reported_without_launching_an_invisible_window(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch('launch_rviz.ROOT',pathlib.Path(directory)),patch('launch_rviz.current_state',return_value=0),patch('launch_rviz.subprocess.Popen') as process,patch('sys.argv',['launch_rviz.py']):
                self.assertEqual(main(),75);process.assert_not_called()
