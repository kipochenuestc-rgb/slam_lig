#!/usr/bin/env python3
"""Select live SLAM, raw-data recording, or mapping from a recorded bag."""
import argparse,datetime,json,os,pathlib,shutil,signal,subprocess,time,yaml
from zoneinfo import ZoneInfo

ROOT=pathlib.Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag',type=pathlib.Path)
    ap.add_argument('--mode',choices=['live','record','replay'],help='Default from config/runtime.yaml')
    ap.add_argument('--live',action='store_true',help='Receive real sensor topics; do not launch a bag player')
    ap.add_argument('--gnss-topic',help='NavSatFix input topic (default from config/bridge.yaml)')
    ap.add_argument('--no-gnss',action='store_true',help='LiDAR + IMU only; disable GNSS input and constraints')
    ap.add_argument('--no-imu-prediction',action='store_true',help='Keep gyro deskew but do not use preintegrated IMU odometry as the registration initial guess')
    ap.add_argument('--calibration-dir',type=pathlib.Path,help='Calibration snapshot for this dataset')
    ap.add_argument('--params-file',type=pathlib.Path,help='LIO-SAM parameters for this dataset')
    ap.add_argument('--imu-bias-file',type=pathlib.Path,help='Stationary IMU bias measured for this dataset')
    ap.add_argument('--input',choices=['raw','fused'],default='raw')
    ap.add_argument('--run',default=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('run_%Y%m%d_%H%M%S'))
    ap.add_argument('--rate',type=float,default=1.)
    ap.add_argument('--start-offset',type=float,default=0.,help='Replay offset in seconds; live/record start immediately')
    ap.add_argument('--duration',type=float,default=0.)
    ap.add_argument('--no-rviz',action='store_true')
    ap.add_argument('--no-save',action='store_true')
    ap.add_argument('--leave-running',action='store_true')
    args=ap.parse_args()
    mode=args.mode or ('live' if args.live else yaml.safe_load((ROOT/'config/runtime.yaml').read_text())['mode'])
    if mode not in ['live','record','replay']:ap.error('Invalid mode in config/runtime.yaml')
    if args.live and args.mode and args.mode!='live':ap.error('--live conflicts with --mode')
    args.live=mode=='live'
    if args.live and args.bag:ap.error('--bag is only valid in replay mode')
    if pathlib.Path(args.run).name!=args.run:ap.error('--run must be a folder name')
    if mode=='record':
        from record_data import record
        if args.bag:ap.error('Recording writes to runs/<run>/bag; --bag is for replay')
        if args.duration<0:ap.error('Invalid duration')
        record(args.run,args.duration,args.gnss_topic);return
    if args.rate<=0 or args.start_offset<0 or args.duration<0:ap.error('Invalid replay timing')
    from session_control import active_mapping
    existing=active_mapping()
    if existing:
        print('已有建图任务正在运行（PID '+str(existing[0]['pid'])+'）。请先运行 stop.cmd 保存并停止旧任务，再启动新任务。',flush=True)
        return 2
    saved_viewers=[]
    for entry in pathlib.Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:command=(entry/'cmdline').read_bytes()
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if b'view_saved_map.py' in command and str(ROOT/'scripts/view_saved_map.py').encode() in command:
            saved_viewers.append(int(entry.name))
        elif b'rviz2' in command and str(ROOT/'config/four_lidar.rviz').encode() in command:
            saved_viewers.append(int(entry.name))
    # Switch away from the saved-map viewer to avoid alternating old/new maps.
    for pid in saved_viewers:
        try:os.kill(pid,signal.SIGINT)
        except ProcessLookupError:pass
    dataset=json.loads((ROOT/'config/dataset.json').read_text())
    bag=None if args.live else (args.bag or pathlib.Path(dataset[args.input+'_bag']))
    if bag is not None and len(str(bag))>2 and str(bag)[1]==':':
        bag=pathlib.Path(subprocess.run(['wslpath','-u',str(bag)],check=True,capture_output=True,text=True).stdout.strip())
    if bag is not None and not (bag/'metadata.yaml').is_file():raise FileNotFoundError(bag/'metadata.yaml')
    gnss_topic=args.gnss_topic or yaml.safe_load((ROOT/'config/bridge.yaml').read_text())['/**']['ros__parameters']['gnss_topic']
    lio_params=yaml.safe_load((args.params_file or ROOT/'config/params.yaml').read_text())
    sim_time='false' if args.live else 'true'
    if pathlib.Path(args.run).name!=args.run:ap.error('--run must be a folder name')
    run=ROOT/'runs'/args.run;run.mkdir(exist_ok=False)
    environment={**os.environ,'SLAM_RUN_DIR':str(run)}
    commands={'launch':['ros2','launch',str(ROOT/'launch/slam.launch.py'),'use_sim_time:='+sim_time],
              'bridge':['python3','-u',str(ROOT/'scripts/gnss_bridge.py'),'--ros-args','--params-file',str(ROOT/'config/bridge.yaml'),'-p','gnss_topic:='+gnss_topic,'-p','use_sim_time:='+sim_time],
              'monitor':['python3','-u',str(ROOT/'scripts/monitor.py'),str(run)],
              'filter':['python3','-u',str(ROOT/'scripts/filter_node.py')]}
    if args.input=='raw':commands['fusion']=['python3','-u',str(ROOT/'scripts/fusion_node.py')]
    if args.calibration_dir and args.input=='raw':commands['fusion']+=['--ros-args','-p','calibration_dir:='+str(args.calibration_dir)]
    if args.params_file:commands['launch']+=['params_file:='+str(args.params_file)]
    if args.no_gnss or args.no_imu_prediction:
        params=lio_params
        if args.no_gnss:params['/**']['ros__parameters']['gpsTopic']='/slam_lig/disabled_gnss/'+args.run
        if args.no_imu_prediction:params['/**']['ros__parameters']['useImuOdometryPrediction']=False
        params_path=run/'slam_params_no_gnss.yaml';params_path.write_text(yaml.safe_dump(params))
        commands['launch']=[x for x in commands['launch'] if not x.startswith('params_file:=')]+['params_file:='+str(params_path)]
        if args.no_gnss:commands['bridge']+=['-p','gnss_enabled:=false']
    if args.imu_bias_file:commands['bridge']+=['-p','imu_bias_file:='+str(args.imu_bias_file)]
    if not args.no_rviz:commands['rviz']=['python3','-u',str(ROOT/'scripts/launch_rviz.py')]
    if not args.live:
        player=['ros2','bag','play',str(bag),'--clock','--start-offset',str(args.start_offset),'--delay','8','--rate',str(args.rate),'--read-ahead-queue-size','1000']
        player+=['--qos-profile-overrides-path',str(ROOT/'config/replay_qos.yaml')]
        if args.duration:player+=['--playback-duration',str(args.duration)]
        if args.input=='raw':player+=['--topics','/imu/data']+([] if args.no_gnss else [gnss_topic])+[f'/point_cloud_hasco{i}' for i in range(1,5)]
        commands['player']=player
    processes={};logs=[];status={'failed':False,'interrupted':False,'mode':mode,'bag':str(bag) if bag is not None else None,'live_sensors':args.live,'gnss_topic':gnss_topic,'input':args.input,'live_fusion':args.input=='raw','live_filter':True,'run':str(run)}
    status['gnss_enabled']=not args.no_gnss
    status['imu_odometry_prediction']=bool(lio_params['/**']['ros__parameters'].get('useImuOdometryPrediction',True))
    status['solid_state_surface_only']=bool(lio_params['/**']['ros__parameters'].get('solidStateSurfaceOnly',False))
    def stop_children():
        for p in processes.values():
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGINT)
                except ProcessLookupError:pass
        for p in processes.values():
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
    stop_requested=[False]
    def request_stop(signum,frame):stop_requested[0]=True
    handlers={sig:signal.signal(sig,request_stop) for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]}
    try:
        for name,argv in commands.items():
            log=open(run/(name+'.log'),'w');logs.append(log)
            processes[name]=subprocess.Popen(['bash',str(ROOT/'scripts/ros_env.sh')]+argv,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True,env=environment)
        (run/'processes.json').write_text(json.dumps({name:{'pid':p.pid,'args':commands[name]} for name,p in processes.items()},indent=2))
        print('STARTED',str(run),'live_filter=True live_fusion='+str(args.input=='raw'),flush=True)
        last=started=time.monotonic();rviz_exit_reported=False
        while not stop_requested[0] and (args.live or processes['player'].poll() is None):
            if (run/'stop_request.json').exists():stop_requested[0]=True;break
            if args.live and args.duration and time.monotonic()-started>=args.duration:break
            if 'rviz' in processes and processes['rviz'].poll() is not None and not rviz_exit_reported:
                print('RVIZ_WINDOW_CLOSED: mapping is still running. For COPY MODE errors, use repair_rviz.cmd. Details: '+str(run/'rviz.log'),flush=True)
                rviz_exit_reported=True
            for name in ['launch','bridge','monitor','filter']+(['fusion'] if args.input=='raw' else []):
                if processes[name].poll() is not None:raise RuntimeError(f'{name} exited: {processes[name].returncode}')
            if (run/'launch.log').read_text(errors='replace').count('reset IMU-preintegration')>4:raise RuntimeError('Repeated IMU resets')
            if time.monotonic()-last>20:
                for name in ['quality.json','live_filter_status.json','fusion_status.json']:
                    path=run/name
                    if path.exists():
                        try:print(name,path.read_text().replace('\n',''),flush=True)
                        except OSError:pass
                last=time.monotonic()
            time.sleep(1)
        status['interrupted']=stop_requested[0]
        if not args.live:
            if stop_requested[0] and processes['player'].poll() is None:processes['player'].send_signal(signal.SIGINT)
            processes['player'].wait(timeout=15)
            status['player_exit_code']=processes['player'].returncode
            if status['player_exit_code']!=0 and not stop_requested[0]:raise RuntimeError('Bag playback failed')
        # Hardware publishers may still be active, so save while SLAM is alive.
        time.sleep(2 if args.live else 12)
        quality_path=run/'quality.json'
        quality=json.loads(quality_path.read_text()) if quality_path.exists() else {}
        valid=quality.get('odom_count',0)>=10 and quality.get('finite',False)
        if not valid and not args.live and not stop_requested[0]:raise RuntimeError('Missing or invalid SLAM output')
        stop_request=json.loads((run/'stop_request.json').read_text()) if (run/'stop_request.json').exists() else {}
        already_saved=stop_request.get('saved_map')
        if already_saved:status.update(map_saved=True,saved_map=already_saved)
        if not args.no_save and valid and not already_saved:
            destination='/slam_lig_maps/'+run.name
            request='{resolution: 0.2, destination: '+json.dumps(destination)+'}'
            saved=subprocess.run(['bash',str(ROOT/'scripts/ros_env.sh'),'ros2','service','call','/lio_sam/save_map','lio_sam/srv/SaveMap',request],capture_output=True,text=True,timeout=180,env=environment)
            (run/'save_map.log').write_text(saved.stdout+saved.stderr)
            source=pathlib.Path.home()/destination.lstrip('/')
            if saved.returncode or 'success=True' not in saved.stdout or not source.exists():raise RuntimeError('SaveMap failed')
            shutil.copytree(source,run/'map')
            status['map_saved']=True
            print('MAP_SAVED',str(run/'map'),flush=True)
    except KeyboardInterrupt:
        status['interrupted']=True
    except Exception as exc:
        status.update(failed=True,error=str(exc));print('FAILED',str(exc),flush=True)
    finally:
        if not args.leave_running or status['failed'] or status['interrupted']:stop_children()
        for sig,handler in handlers.items():signal.signal(sig,handler)
        # Audit complete replay only; partial/live runs have no metadata total.
        bridge_status=run/'bridge_status.json'
        if not args.live and args.start_offset==0 and args.duration==0 and not status['interrupted'] and not status['failed'] and bridge_status.exists():
            try:
                metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
                expected=next((t['message_count'] for t in metadata['topics_with_message_count'] if t['topic_metadata']['name']=='/imu/data'),0)
                relayed=json.loads(bridge_status.read_text())['counts'].get('imu_published',0)
                if expected:
                    status['imu_relay']={'expected':expected,'published':relayed,'fraction':relayed/expected,'ok':relayed>=.99*expected}
                    if relayed<.99*expected:print(f'IMU_RELAY_WARNING: {relayed}/{expected} samples; map quality requires review',flush=True)
            except (OSError,ValueError,KeyError,yaml.YAMLError) as exc:
                status['imu_relay']={'ok':False,'audit_error':str(exc)}
                print('IMU_RELAY_WARNING: audit unavailable: '+str(exc),flush=True)
        (run/'run_status.json').write_text(json.dumps(status,indent=2))
        for log in logs:log.close()
    print('RUN_COMPLETE',json.dumps(status),flush=True)
    if status['failed']:raise SystemExit(1)


if __name__=='__main__':raise SystemExit(main())
