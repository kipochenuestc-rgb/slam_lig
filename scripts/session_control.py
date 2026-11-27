#!/usr/bin/env python3
"""Save and stop only this project's mapping/recording processes, including orphans."""
import argparse,datetime,json,os,pathlib,shutil,signal,subprocess,time
from zoneinfo import ZoneInfo

ROOT=pathlib.Path(__file__).resolve().parents[1]

def process_info(pid):
    try:
        proc=pathlib.Path('/proc')/str(pid);fields=(proc/'stat').read_text().rsplit(')',1)[1].split()
        if fields[0]=='Z':return None
        command=[os.fsdecode(x) for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
        environment=dict(os.fsdecode(x).split('=',1) for x in (proc/'environ').read_bytes().split(b'\0') if b'=' in x)
        return {'pid':int(pid),'ppid':int(fields[1]),'start_ticks':fields[19],'command':command,'run':environment.get('SLAM_RUN_DIR')}
    except (FileNotFoundError,ProcessLookupError,PermissionError,ValueError):return None

def all_processes():
    return [info for entry in pathlib.Path('/proc').iterdir() if entry.name.isdigit() and (info:=process_info(entry.name))]

def is_mapper(info):
    command=info['command']
    if not command or pathlib.Path(command[0]).name != 'lio_sam_mapOptimization':
        return False
    run = owned_run(info.get('run'))
    return str(ROOT/'config/params.yaml') in command or bool(run and str(run/'params_from_tf.yaml') in command)

def active_mapping():return [p for p in all_processes() if is_mapper(p)]

def same_process(expected):
    actual=process_info(expected['pid'])
    return actual is not None and actual['start_ticks']==expected['start_ticks'] and actual['command']==expected['command']

def send(info,signum):
    if same_process(info):
        try:os.kill(info['pid'],signum)
        except ProcessLookupError:pass

def owned_run(value):
    if not value:return None
    run=pathlib.Path(value).resolve()
    return run if run.parent==(ROOT/'runs').resolve() else None

def save_map(run):
    suffix=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    destination='/slam_lig_maps/stop_'+suffix;target=run/('map_stop_'+suffix)
    request='{resolution: 0.2, destination: '+json.dumps(destination)+'}'
    print('正在保存当前地图到 '+str(target),flush=True)
    result=subprocess.run(['bash',str(ROOT/'scripts/ros_env.sh'),'ros2','service','call','/lio_sam/save_map','lio_sam/srv/SaveMap',request],capture_output=True,text=True,timeout=180)
    (run/'stop_save_map.log').write_text(result.stdout+result.stderr)
    source=pathlib.Path.home()/destination.lstrip('/')
    if result.returncode or 'success=True' not in result.stdout or not source.is_dir():raise RuntimeError('地图保存失败，旧任务保持运行。详情：'+str(run/'stop_save_map.log'))
    shutil.copytree(source,target);return str(target)

def controller_for(mapper):
    process=mapper
    for _ in range(12):
        process=process_info(process['ppid'])
        if not process:return None
        if str(ROOT/'scripts/run_slam.py') in process['command']:return process
    return None

def is_viewer(info):
    c=info['command']
    if not c:return False
    if pathlib.Path(c[0]).name=='rviz2':return any(str(ROOT/'config')+'/' in arg for arg in c)
    return any(str(ROOT/'scripts'/name) in c for name in ['view_saved_map.py','launch_rviz.py'])

def is_recorder(info):
    c=info['command']
    if 'bag' not in c or 'record' not in c or '--output' not in c:return False
    index=c.index('--output')+1
    if index>=len(c):return False
    bag=pathlib.Path(c[index]).resolve()
    return bag.name=='bag' and bag.parent.parent==(ROOT/'runs').resolve()

def stop_members(run):
    members=[p for p in all_processes() if owned_run(p['run'])==run]
    for p in members:send(p,signal.SIGINT)
    deadline=time.monotonic()+15
    while any(same_process(p) for p in members) and time.monotonic()<deadline:time.sleep(.2)
    for p in members:
        if same_process(p):send(p,signal.SIGTERM)
    deadline=time.monotonic()+5
    while any(same_process(p) for p in members) and time.monotonic()<deadline:time.sleep(.1)
    if any(same_process(p) for p in members):raise RuntimeError('仍有本项目节点未退出，请查看进程状态。')

def stop_all(save=True):
    maps=active_mapping();summary={'saved_maps':[],'stopped_runs':[]}
    if len(maps)>1:raise RuntimeError('检测到多个建图服务，无法确定地图归属；请先检查各运行日志。')
    for mapper in maps:
        run=owned_run(mapper['run'])
        if run is None:raise RuntimeError('无法确认建图进程所属运行目录，未停止该进程。')
        saved=save_map(run) if save else None
        if saved:summary['saved_maps'].append(saved)
        request={'saved_map':saved,'requested_at':time.time()}
        temporary=run/'stop_request.tmp';temporary.write_text(json.dumps(request));temporary.replace(run/'stop_request.json')
        controller=controller_for(mapper)
        if controller:
            print('正在通知当前控制进程正常退出……',flush=True);send(controller,signal.SIGINT)
            deadline=time.monotonic()+25
            while same_process(controller) and time.monotonic()<deadline:time.sleep(.2)
        # If a terminal was closed, the controller may be gone while its nodes
        # still run. The per-run environment identifies those nodes/workers.
        stop_members(run)
        (run/'stop_status.json').write_text(json.dumps({'stopped':True,'saved_map':saved,'controller_was_alive':controller is not None},indent=2))
        summary['stopped_runs'].append(str(run))
    # Also clean partially started runs whose mapper has already exited.
    remaining={run for p in all_processes() if (run:=owned_run(p['run'])) is not None and str(run) not in summary['stopped_runs']}
    for run in remaining:
        stop_members(run)
        (run/'stop_status.json').write_text(json.dumps({'stopped':True,'saved_map':None,'mapper_not_running':True},indent=2))
        summary['stopped_runs'].append(str(run))
    recorders=[p for p in all_processes() if is_recorder(p)]
    for p in recorders:send(p,signal.SIGINT)
    deadline=time.monotonic()+30
    while any(same_process(p) for p in recorders) and time.monotonic()<deadline:time.sleep(.2)
    if any(same_process(p) for p in recorders):raise RuntimeError('录制器仍在写入 bag，请稍后重试，未强制终止。')
    for p in all_processes():
        if is_viewer(p):send(p,signal.SIGINT)
    print('已停止本项目任务。'+('当前没有运行中的建图任务。' if not maps else ''),flush=True)
    print(json.dumps(summary,ensure_ascii=False),flush=True);return summary

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('action',choices=['status','stop']);ap.add_argument('--no-save',action='store_true');args=ap.parse_args()
    if args.action=='status':print(json.dumps(active_mapping(),ensure_ascii=False,indent=2));return 0
    try:stop_all(not args.no_save);return 0
    except Exception as exc:print('停止失败：'+str(exc),flush=True);return 1

if __name__=='__main__':raise SystemExit(main())
