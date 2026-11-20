"""Record raw sensor messages for later mapping; no SLAM/filter/CSV input."""
import json,os,pathlib,signal,subprocess,time,yaml

ROOT=pathlib.Path(__file__).resolve().parents[1]

def record(run_name,duration=0.,gnss_topic=None):
    run=ROOT/'runs'/run_name;run.mkdir(exist_ok=False);bag=run/'bag'
    config=yaml.safe_load((ROOT/'config/runtime.yaml').read_text());topics=config['record_topics']
    configured_gnss=yaml.safe_load((ROOT/'config/bridge.yaml').read_text())['/**']['ros__parameters']['gnss_topic']
    topics=[t for t in topics if t!='/gnss/fix_fixed']+[gnss_topic or configured_gnss];topics=list(dict.fromkeys(topics))
    command=['bash',str(ROOT/'scripts/ros_env.sh'),'ros2','bag','record','--storage','mcap','--output',str(bag),'--topics']+topics
    stop=[False];handler=signal.signal(signal.SIGINT,lambda *_:stop.__setitem__(0,True))
    result={'mode':'record','bag':str(bag),'topics':topics,'failed':False};process=None
    try:
        with (run/'recorder.log').open('w') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,stdin=subprocess.DEVNULL)
            (run/'processes.json').write_text(json.dumps({'recorder':{'pid':process.pid,'args':command}},indent=2))
            print('RECORDING',str(bag),'Stop with Ctrl+C',flush=True);started=time.monotonic()
            while process.poll() is None and not stop[0] and (duration<=0 or time.monotonic()-started<duration):time.sleep(.2)
            requested=stop[0] or (duration>0 and time.monotonic()-started>=duration)
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            process.wait(timeout=30)
            if process.returncode!=0 and not(requested and process.returncode==-signal.SIGINT):raise RuntimeError(f'Recorder exited {process.returncode}')
        metadata=bag/'metadata.yaml'
        if not metadata.exists():raise RuntimeError('Recorder did not finalize metadata.yaml')
        info=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information'];result['message_count']=info['message_count']
    except Exception as exc:result.update(failed=True,error=str(exc))
    finally:
        if process is not None and process.poll() is None:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        signal.signal(signal.SIGINT,handler);(run/'run_status.json').write_text(json.dumps(result,indent=2))
    print('RECORD_COMPLETE',json.dumps(result),flush=True)
    if result['failed']:raise SystemExit(1)
