#!/usr/bin/env python3
"""Select live SLAM, raw-data recording, or mapping from a recorded bag."""

import argparse
import datetime
import functools
import json
import os
import pathlib
import shutil
import signal
import subprocess
import time
import yaml
from zoneinfo import ZoneInfo

ROOT = pathlib.Path(__file__).resolve().parents[1]


def build_parser():
    """构造统一的命令行解析器，供入口和测试复用。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=pathlib.Path)
    parser.add_argument('--mode', choices=['live', 'record', 'replay'], help='Default from config/runtime.yaml')
    parser.add_argument('--live', action='store_true', help='Receive real sensor topics; do not launch a bag player')
    parser.add_argument('--gnss-topic', help='NavSatFix input topic (default from config/bridge.yaml)')
    parser.add_argument('--no-gnss', action='store_true', help='LiDAR + IMU only; disable GNSS input and constraints')
    parser.add_argument('--no-imu-prediction', action='store_true', help='Keep gyro deskew but do not use preintegrated IMU odometry as the registration initial guess')
    parser.add_argument('--calibration-dir', type=pathlib.Path, help='Calibration snapshot for this dataset')
    parser.add_argument('--params-file', type=pathlib.Path, help='LIO-SAM parameters for this dataset')
    parser.add_argument('--imu-bias-file', type=pathlib.Path, help='Stationary IMU bias measured for this dataset')
    parser.add_argument('--input', choices=['raw', 'fused'], default='raw')
    parser.add_argument('--run', default=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('run_%Y%m%d_%H%M%S'))
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--start-offset', type=float, default=0., help='Replay offset in seconds; live/record start immediately')
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--no-save', action='store_true')
    parser.add_argument('--leave-running', action='store_true')
    return parser


def parse_arguments(argv=None):
    """解析命令行参数并返回解析器与参数对象。"""
    parser = build_parser()
    return parser, parser.parse_args(argv)


def resolve_mode(args, parser):
    """根据显式参数或运行配置确定模式，并校验模式相关选项。"""
    mode = args.mode
    if mode is None:
        mode = 'live' if args.live else yaml.safe_load(
            (ROOT / 'config/runtime.yaml').read_text()
        )['mode']
    if mode not in ['live', 'record', 'replay']:
        parser.error('Invalid mode in config/runtime.yaml')
    if args.live and args.mode and args.mode != 'live':
        parser.error('--live conflicts with --mode')
    args.live = mode == 'live'
    if args.live and args.bag:
        parser.error('--bag is only valid in replay mode')
    if pathlib.Path(args.run).name != args.run:
        parser.error('--run must be a folder name')
    if mode == 'record':
        if args.bag:
            parser.error('Recording writes to runs/<run>/bag; --bag is for replay')
        if args.duration < 0:
            parser.error('Invalid duration')
        return mode
    if args.rate <= 0 or args.start_offset < 0 or args.duration < 0:
        parser.error('Invalid replay timing')
    return mode


def close_saved_viewers():
    """只向本工程启动的静态地图查看器和 RViz 发送中断信号。"""
    viewer_pids = []
    for entry in pathlib.Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / 'cmdline').read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if b'view_saved_map.py' in command and str(ROOT / 'scripts/view_saved_map.py').encode() in command:
            viewer_pids.append(int(entry.name))
        elif b'rviz2' in command and str(ROOT / 'config/four_lidar.rviz').encode() in command:
            viewer_pids.append(int(entry.name))
    # 切换到实时建图前关闭旧地图，避免两个发布源交替显示。
    for pid in viewer_pids:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass


def resolve_bag(args, dataset):
    """选择回放 bag，转换 Windows 路径并确认元数据存在。"""
    bag = None if args.live else (args.bag or pathlib.Path(dataset[args.input + '_bag']))
    if bag is not None and len(str(bag)) > 2 and str(bag)[1] == ':':
        converted = subprocess.run(
            ['wslpath', '-u', str(bag)], check=True, capture_output=True, text=True
        ).stdout.strip()
        bag = pathlib.Path(converted)
    if bag is not None and not (bag / 'metadata.yaml').is_file():
        raise FileNotFoundError(bag / 'metadata.yaml')
    return bag


def load_mapping_config(args):
    """加载本次运行的 bag、GNSS 话题和 LIO-SAM 参数。"""
    dataset = json.loads((ROOT / 'config/dataset.json').read_text())
    bag = resolve_bag(args, dataset)
    gnss_topic = args.gnss_topic
    if not gnss_topic:
        bridge = yaml.safe_load((ROOT / 'config/bridge.yaml').read_text())
        gnss_topic = bridge['/**']['ros__parameters']['gnss_topic']
    params_path = args.params_file or ROOT / 'config/params.yaml'
    lio_params = yaml.safe_load(params_path.read_text())
    return bag, gnss_topic, lio_params


def build_commands(args, bag, gnss_topic, lio_params, run):
    """组装 ROS 子进程命令，并写出本次运行需要的参数覆盖文件。"""
    sim_time = 'false' if args.live else 'true'
    commands = {
        'launch': ['ros2', 'launch', str(ROOT / 'launch/slam.launch.py'), 'use_sim_time:=' + sim_time],
        'bridge': [
            'python3', '-u', str(ROOT / 'scripts/gnss_bridge.py'), '--ros-args',
            '--params-file', str(ROOT / 'config/bridge.yaml'), '-p', 'gnss_topic:=' + gnss_topic,
            '-p', 'use_sim_time:=' + sim_time,
        ],
        'monitor': ['python3', '-u', str(ROOT / 'scripts/monitor.py'), str(run)],
        'filter': ['python3', '-u', str(ROOT / 'scripts/filter_node.py')],
    }
    if args.input == 'raw':
        commands['fusion'] = ['python3', '-u', str(ROOT / 'scripts/fusion_node.py')]
        if args.calibration_dir:
            commands['fusion'] += ['--ros-args', '-p', 'calibration_dir:=' + str(args.calibration_dir)]
    if args.params_file:
        commands['launch'] += ['params_file:=' + str(args.params_file)]
    if args.no_gnss or args.no_imu_prediction:
        params = lio_params
        if args.no_gnss:
            params['/**']['ros__parameters']['gpsTopic'] = '/slam_lig/disabled_gnss/' + args.run
        if args.no_imu_prediction:
            params['/**']['ros__parameters']['useImuOdometryPrediction'] = False
        params_path = run / 'slam_params_no_gnss.yaml'
        params_path.write_text(yaml.safe_dump(params))
        commands['launch'] = [
            item for item in commands['launch'] if not item.startswith('params_file:=')
        ] + ['params_file:=' + str(params_path)]
        if args.no_gnss:
            commands['bridge'] += ['-p', 'gnss_enabled:=false']
    if args.imu_bias_file:
        commands['bridge'] += ['-p', 'imu_bias_file:=' + str(args.imu_bias_file)]
    if not args.no_rviz:
        commands['rviz'] = ['python3', '-u', str(ROOT / 'scripts/launch_rviz.py')]
    if not args.live:
        player = [
            'ros2', 'bag', 'play', str(bag), '--clock', '--start-offset', str(args.start_offset),
            '--delay', '8', '--rate', str(args.rate), '--read-ahead-queue-size', '1000',
            '--qos-profile-overrides-path', str(ROOT / 'config/replay_qos.yaml'),
        ]
        if args.duration:
            player += ['--playback-duration', str(args.duration)]
        if args.input == 'raw':
            player += ['--topics', '/imu/data']
            if not args.no_gnss:
                player.append(gnss_topic)
            player += [f'/point_cloud_hasco{index}' for index in range(1, 5)]
        commands['player'] = player
    return commands


def build_initial_status(args, mode, bag, gnss_topic, lio_params, run):
    """构造运行状态文件的初始字段，记录本次启用的功能。"""
    params = lio_params['/**']['ros__parameters']
    return {
        'failed': False,
        'interrupted': False,
        'mode': mode,
        'bag': str(bag) if bag is not None else None,
        'live_sensors': args.live,
        'gnss_topic': gnss_topic,
        'input': args.input,
        'live_fusion': args.input == 'raw',
        'live_filter': True,
        'run': str(run),
        'gnss_enabled': not args.no_gnss,
        'imu_odometry_prediction': bool(params.get('useImuOdometryPrediction', True)),
        'solid_state_surface_only': bool(params.get('solidStateSurfaceOnly', False)),
    }


def request_stop(stop_requested, signum, frame):
    """记录系统中断请求，留给主循环按正常流程收尾。"""
    stop_requested[0] = True


def install_signal_handlers(stop_requested):
    """安装协作式退出处理器并返回原处理器供后续恢复。"""
    handler = functools.partial(request_stop, stop_requested)
    return {sig: signal.signal(sig, handler) for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]}


def restore_signal_handlers(handlers):
    """恢复当前进程启动映射前的信号处理器。"""
    for signum, handler in handlers.items():
        signal.signal(signum, handler)


def start_processes(commands, run, environment, processes, logs):
    """按命令表启动节点，为每个子进程创建独立日志文件。"""
    for name, argv in commands.items():
        log = (run / (name + '.log')).open('w')
        logs.append(log)
        processes[name] = subprocess.Popen(
            ['bash', str(ROOT / 'scripts/ros_env.sh')] + argv,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env=environment,
        )
    process_records = {
        name: {'pid': process.pid, 'args': commands[name]}
        for name, process in processes.items()
    }
    (run / 'processes.json').write_text(json.dumps(process_records, indent=2))


def stop_processes(processes):
    """先中断所有子进程，超时后升级为终止信号。"""
    for process in processes.values():
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    for process in processes.values():
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)


def report_progress(run):
    """打印已有的质量、滤波和点云拼接状态文件。"""
    for filename in ['quality.json', 'live_filter_status.json', 'fusion_status.json']:
        path = run / filename
        if path.exists():
            try:
                print(filename, path.read_text().replace('\n', ''), flush=True)
            except OSError:
                pass


def monitor_run(args, run, processes, stop_requested):
    """监控建图健康状态、外部停止请求和实时运行时长。"""
    last_report = started = time.monotonic()
    rviz_exit_reported = False
    while not stop_requested[0] and (args.live or processes['player'].poll() is None):
        if (run / 'stop_request.json').exists():
            stop_requested[0] = True
            break
        if args.live and args.duration and time.monotonic() - started >= args.duration:
            break
        if 'rviz' in processes and processes['rviz'].poll() is not None and not rviz_exit_reported:
            print(
                'RVIZ_WINDOW_CLOSED: mapping is still running. For COPY MODE errors, '
                'use repair_rviz.cmd. Details: ' + str(run / 'rviz.log'),
                flush=True,
            )
            rviz_exit_reported = True
        required = ['launch', 'bridge', 'monitor', 'filter']
        if args.input == 'raw':
            required.append('fusion')
        for name in required:
            if processes[name].poll() is not None:
                raise RuntimeError(f'{name} exited: {processes[name].returncode}')
        launch_log = (run / 'launch.log').read_text(errors='replace')
        if launch_log.count('reset IMU-preintegration') > 4:
            raise RuntimeError('Repeated IMU resets')
        if time.monotonic() - last_report > 20:
            report_progress(run)
            last_report = time.monotonic()
        time.sleep(1)


def finish_playback(args, processes, status, stop_requested):
    """等待 bag 播放器退出，并检查非中断情况下的退出码。"""
    if args.live:
        return
    player = processes['player']
    if stop_requested[0] and player.poll() is None:
        player.send_signal(signal.SIGINT)
    player.wait(timeout=15)
    status['player_exit_code'] = player.returncode
    if status['player_exit_code'] != 0 and not stop_requested[0]:
        raise RuntimeError('Bag playback failed')


def save_final_map(args, run, environment, valid, already_saved, status):
    """通过 ROS 服务保存有效地图并复制到本次运行目录。"""
    if already_saved:
        status.update(map_saved=True, saved_map=already_saved)
    if args.no_save or not valid or already_saved:
        return
    destination = '/slam_lig_maps/' + run.name
    request = '{resolution: 0.2, destination: ' + json.dumps(destination) + '}'
    saved = subprocess.run(
        ['bash', str(ROOT / 'scripts/ros_env.sh'), 'ros2', 'service', 'call',
         '/lio_sam/save_map', 'lio_sam/srv/SaveMap', request],
        capture_output=True,
        text=True,
        timeout=180,
        env=environment,
    )
    (run / 'save_map.log').write_text(saved.stdout + saved.stderr)
    source = pathlib.Path.home() / destination.lstrip('/')
    if saved.returncode or 'success=True' not in saved.stdout or not source.exists():
        raise RuntimeError('SaveMap failed')
    shutil.copytree(source, run / 'map')
    status['map_saved'] = True
    print('MAP_SAVED', str(run / 'map'), flush=True)


def complete_run(args, run, environment, processes, stop_requested, status):
    """结束回放、确认里程计质量并按需保存最终地图。"""
    status['interrupted'] = stop_requested[0]
    finish_playback(args, processes, status, stop_requested)
    # 硬件仍可能发布数据，先让 SLAM 完成最后一轮优化和地图保存。
    time.sleep(2 if args.live else 12)
    quality_path = run / 'quality.json'
    quality = json.loads(quality_path.read_text()) if quality_path.exists() else {}
    valid = quality.get('odom_count', 0) >= 10 and quality.get('finite', False)
    if not valid and not args.live and not stop_requested[0]:
        raise RuntimeError('Missing or invalid SLAM output')
    request_path = run / 'stop_request.json'
    stop_request = json.loads(request_path.read_text()) if request_path.exists() else {}
    save_final_map(args, run, environment, valid, stop_request.get('saved_map'), status)


def audit_imu_relay(args, bag, run, status):
    """对完整回放的 IMU 转发数量与 bag 元数据进行核对。"""
    bridge_status = run / 'bridge_status.json'
    should_audit = (
        not args.live and args.start_offset == 0 and args.duration == 0
        and not status['interrupted'] and not status['failed'] and bridge_status.exists()
    )
    if not should_audit:
        return
    try:
        metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
        topics = metadata['topics_with_message_count']
        expected = next(
            (topic['message_count'] for topic in topics
             if topic['topic_metadata']['name'] == '/imu/data'),
            0,
        )
        published = json.loads(bridge_status.read_text())['counts'].get('imu_published', 0)
        if expected:
            fraction = published / expected
            status['imu_relay'] = {
                'expected': expected,
                'published': published,
                'fraction': fraction,
                'ok': published >= .99 * expected,
            }
            if published < .99 * expected:
                print(
                    f'IMU_RELAY_WARNING: {published}/{expected} samples; map quality requires review',
                    flush=True,
                )
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        status['imu_relay'] = {'ok': False, 'audit_error': str(exc)}
        print('IMU_RELAY_WARNING: audit unavailable: ' + str(exc), flush=True)


def run_mapping(args, mode):
    """启动建图子进程，执行监控、收尾和运行状态审计。"""
    from session_control import active_mapping

    existing = active_mapping()
    if existing:
        print(
            '已有建图任务正在运行（PID ' + str(existing[0]['pid'])
            + '）。请先运行 stop.cmd 保存并停止旧任务，再启动新任务。',
            flush=True,
        )
        return 2
    close_saved_viewers()
    bag, gnss_topic, lio_params = load_mapping_config(args)
    run = ROOT / 'runs' / args.run
    run.mkdir(exist_ok=False)
    environment = {**os.environ, 'SLAM_RUN_DIR': str(run)}
    commands = build_commands(args, bag, gnss_topic, lio_params, run)
    status = build_initial_status(args, mode, bag, gnss_topic, lio_params, run)
    processes = {}
    logs = []
    stop_requested = [False]
    handlers = install_signal_handlers(stop_requested)
    try:
        start_processes(commands, run, environment, processes, logs)
        print(
            'STARTED', str(run), 'live_filter=True live_fusion=' + str(args.input == 'raw'),
            flush=True,
        )
        monitor_run(args, run, processes, stop_requested)
        complete_run(args, run, environment, processes, stop_requested, status)
    except KeyboardInterrupt:
        status['interrupted'] = True
    except Exception as exc:
        status.update(failed=True, error=str(exc))
        print('FAILED', str(exc), flush=True)
    finally:
        if not args.leave_running or status['failed'] or status['interrupted']:
            stop_processes(processes)
        restore_signal_handlers(handlers)
        audit_imu_relay(args, bag, run, status)
        (run / 'run_status.json').write_text(json.dumps(status, indent=2))
        for log in logs:
            log.close()
    print('RUN_COMPLETE', json.dumps(status), flush=True)
    if status['failed']:
        raise SystemExit(1)


def main(argv=None):
    """解析模式后进入录制流程或统一的建图生命周期。"""
    parser, args = parse_arguments(argv)
    mode = resolve_mode(args, parser)
    if mode == 'record':
        from record_data import record

        record(args.run, args.duration, args.gnss_topic)
        return None
    return run_mapping(args, mode)


if __name__ == '__main__':
    raise SystemExit(main())
