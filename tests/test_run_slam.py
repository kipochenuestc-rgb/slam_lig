"""验证建图入口的参数校验和 ROS 命令组装。"""

import tempfile
import unittest
from pathlib import Path

import yaml
import run_slam


def make_lio_params():
    """返回足以覆盖运行参数改写的最小 LIO-SAM 配置。"""
    return {
        '/**': {
            'ros__parameters': {
                'gpsTopic': '/gnss/fix_fixed',
                'useImuOdometryPrediction': True,
            }
        }
    }


class RunSlamCommandTests(unittest.TestCase):
    """检查不同运行模式对应的节点和回放参数。"""

    def test_live_mode_uses_wall_clock_without_player(self):
        """实时建图不启动 bag 播放器，并关闭仿真时钟。"""
        parser, args = run_slam.parse_arguments(['--mode', 'live', '--run', 'live', '--no-rviz'])
        mode = run_slam.resolve_mode(args, parser)
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            commands = run_slam.build_commands(args, None, '/gnss/fix_fixed', make_lio_params(), run)
        self.assertEqual(mode, 'live')
        self.assertNotIn('player', commands)
        self.assertIn('use_sim_time:=false', commands['launch'])
        self.assertIn('fusion', commands)

    def test_raw_replay_preserves_sensor_selection_and_timing(self):
        """原始 bag 回放保留 GNSS、四路雷达和时间参数。"""
        parser, args = run_slam.parse_arguments([
            '--mode', 'replay', '--run', 'raw_replay', '--rate', '1.5',
            '--start-offset', '2', '--duration', '30', '--gnss-topic', '/gnss/custom', '--no-rviz',
        ])
        run_slam.resolve_mode(args, parser)
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            commands = run_slam.build_commands(
                args, Path('/recording/bag'), '/gnss/custom', make_lio_params(), run
            )
        player = commands['player']
        self.assertEqual(player[player.index('--rate') + 1], '1.5')
        self.assertEqual(player[player.index('--start-offset') + 1], '2.0')
        self.assertEqual(player[player.index('--playback-duration') + 1], '30.0')
        topics = player[player.index('--topics') + 1:]
        self.assertEqual(
            topics,
            ['/imu/data', '/gnss/custom'] + [f'/point_cloud_hasco{index}' for index in range(1, 5)],
        )

    def test_fused_replay_applies_gnss_and_imu_overrides(self):
        """融合点云回放应用 GNSS 与 IMU 预测覆盖且不启动拼接节点。"""
        parser, args = run_slam.parse_arguments([
            '--mode', 'replay', '--run', 'fused_replay', '--input', 'fused',
            '--no-gnss', '--no-imu-prediction', '--no-rviz',
        ])
        mode = run_slam.resolve_mode(args, parser)
        params = make_lio_params()
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            commands = run_slam.build_commands(args, Path('/recording/bag'), '/gnss/fix_fixed', params, run)
            status = run_slam.build_initial_status(
                args, mode, Path('/recording/bag'), '/gnss/fix_fixed', params, run
            )
            written_params = yaml.safe_load((run / 'slam_params_no_gnss.yaml').read_text())
        self.assertNotIn('fusion', commands)
        self.assertNotIn('--topics', commands['player'])
        self.assertIn('gnss_enabled:=false', commands['bridge'])
        self.assertEqual(written_params['/**']['ros__parameters']['gpsTopic'], '/slam_lig/disabled_gnss/fused_replay')
        self.assertFalse(written_params['/**']['ros__parameters']['useImuOdometryPrediction'])
        self.assertFalse(status['gnss_enabled'])
        self.assertFalse(status['imu_odometry_prediction'])

    def test_live_flag_rejects_conflicting_mode(self):
        """显式实时开关与回放模式冲突时保留命令行错误。"""
        parser, args = run_slam.parse_arguments(['--live', '--mode', 'replay'])
        with self.assertRaises(SystemExit):
            run_slam.resolve_mode(args, parser)
