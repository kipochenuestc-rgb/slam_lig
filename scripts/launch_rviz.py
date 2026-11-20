#!/usr/bin/env python3
"""Check WSLg before RViz; optionally repair graphics without stopping SLAM."""
import argparse,json,os,pathlib,re,signal,subprocess,time

ROOT=pathlib.Path(__file__).resolve().parents[1]

def graphics_state(log):
    values=re.findall(r'RDP backend: use_gfxredir = ([01])',log)
    return int(values[-1]) if values else None

def current_state():
    log=pathlib.Path('/mnt/wslg/weston.log')
    return graphics_state(log.read_text(errors='replace')) if log.exists() else None

def repair():
    print('正在重启本发行版的 WSLg 图形后台；WSL 图形窗口会关闭，建图节点继续运行。',flush=True)
    wsl='/mnt/c/Windows/System32/wsl.exe'
    distro=os.environ.get('WSL_DISTRO_NAME','Ubuntu-24.04')
    log_path=pathlib.Path('/mnt/wslg/weston.log')
    before=[line for line in log_path.read_text(errors='replace').splitlines() if 'RDP backend: use_gfxredir =' in line]
    previous=before[-1] if before else None
    # Target the current distro explicitly: --system alone targets the default
    # distro, which may be a different Ubuntu installation.
    result=subprocess.run([wsl,'-d',distro,'--system','-u','root','--exec','sh','-c','pidof weston | xargs -r kill -TERM'],capture_output=True,timeout=15)
    if result.returncode:raise RuntimeError(result.stderr.decode('utf-8',errors='replace'))
    time.sleep(2)
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        lines=[line for line in log_path.read_text(errors='replace').splitlines() if 'RDP backend: use_gfxredir =' in line]
        if lines and lines[-1]!=previous and current_state()==1 and pathlib.Path('/tmp/.X11-unix/X0').exists():return
        time.sleep(1)
    raise RuntimeError('WSLg did not recover; save WSL work before a full wsl --shutdown')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repair',action='store_true')
    ap.add_argument('--check',action='store_true',help='Only check graphics readiness')
    ap.add_argument('--config',default=str(ROOT/'config/live_mapping.rviz'))
    args=ap.parse_args()
    status_path=ROOT/'runs/rviz_gui_status.json';status_path.parent.mkdir(exist_ok=True)
    try:
        if args.repair and current_state()!=1:repair()
        if current_state()==0:
            raise RuntimeError('WSLg shared-memory graphics failed (COPY MODE). Run repair_rviz.cmd to restart graphics and reopen RViz; mapping can continue.')
        if args.check:return 0
        config=pathlib.Path(args.config)
        if not config.is_file():raise FileNotFoundError(config)
        # Only replace RViz windows owned by this project, not other ROS apps.
        for entry in pathlib.Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:command=(entry/'cmdline').read_bytes().split(b'\0')
            except (FileNotFoundError,PermissionError,ProcessLookupError):continue
            if command and pathlib.Path(os.fsdecode(command[0])).name=='rviz2' and any(str(ROOT/'config').encode() in value for value in command):
                try:os.kill(int(entry.name),signal.SIGINT)
                except ProcessLookupError:pass
        process=subprocess.Popen(['rviz2','-d',str(config)])
        status_path.write_text(json.dumps({'graphics_backend':current_state(),'rviz_pid':process.pid,'config':str(config),'error':None},indent=2))
        return process.wait()
    except KeyboardInterrupt:return 0
    except Exception as exc:
        status_path.write_text(json.dumps({'graphics_backend':current_state(),'error':str(exc)},indent=2))
        print('RVIZ_DISPLAY_ERROR: '+str(exc),flush=True);return 75

if __name__=='__main__':raise SystemExit(main())
