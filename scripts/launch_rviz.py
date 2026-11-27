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

def graphics_idle():
    """Avoid automatic recovery while other X11 or native Wayland clients are connected."""
    try:
        x11 = subprocess.run(['xlsclients', '-a'], capture_output=True, text=True, timeout=3)
        sockets = subprocess.run(['ss', '-xHn'], capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return (x11.returncode == sockets.returncode == 0 and not x11.stdout.strip()
            and not any('wayland-' in line and 'ESTAB' in line for line in sockets.stdout.splitlines()))


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

def x11(*args):
    result = subprocess.run(args, capture_output=True, text=True, timeout=3,
                            env={**os.environ, 'LC_ALL':'C'})
    return result.stdout if result.returncode == 0 else ''


def visible_window(pid):
    """Match a mapped RViz main window to this process, ignoring Qt helper windows."""
    try:
        tree = x11('xwininfo','-root','-tree')
        for line in tree.splitlines():
            match = re.match(r'\s*(0x[0-9a-fA-F]+) "[^"]* - RViz": +\("rviz2" "rviz2"\)',line)
            if not match:
                continue
            wid = match[1]
            owner = x11('xprop','-id',wid,'_NET_WM_PID')
            if owner.strip().endswith('= '+str(pid)) and 'Map State: IsViewable' in x11('xwininfo','-id',wid):
                return wid
    except (OSError,subprocess.TimeoutExpired):
        pass
    return None


def stop_child(process):
    if process and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repair',action='store_true')
    ap.add_argument('--repair-if-idle',action='store_true')
    ap.add_argument('--check',action='store_true',help='Only check graphics readiness')
    ap.add_argument('--config',default=str(ROOT/'config/live_mapping.rviz'))
    args=ap.parse_args()
    status_file=pathlib.Path(os.environ.get('SLAM_RUN_DIR',ROOT/'runs'))/'rviz_gui_status.json'
    status_file.parent.mkdir(exist_ok=True)
    def status(state,message,**fields):
        tmp=status_file.with_suffix('.'+str(os.getpid())+'.tmp')
        tmp.write_text(json.dumps(dict(state=state,message=message,**fields),ensure_ascii=False))
        tmp.replace(status_file)

    process=None
    try:
        config=pathlib.Path(args.config)
        if not config.is_file() and not args.check and current_state()!=0:
            raise FileNotFoundError(config)
        if not args.check:
            for entry in pathlib.Path('/proc').iterdir():
                if not entry.name.isdigit():continue
                try:command=(entry/'cmdline').read_bytes().split(b'\0')
                except (FileNotFoundError,PermissionError,ProcessLookupError):continue
                if (command and pathlib.Path(os.fsdecode(command[0])).name=='rviz2'
                        and any(value.startswith(os.fsencode(ROOT/'config')+b'/') for value in command)):
                    try:os.kill(int(entry.name),signal.SIGINT)
                    except ProcessLookupError:pass
        last_error=''
        for attempt in range(1,4):
            try:
                status('starting',f'正在启动 RViz（第 {attempt} 次）')
                state=current_state()
                if state==0 and (args.repair or (args.repair_if_idle and graphics_idle())):
                    status('repairing','正在恢复 WSLg 图形服务')
                    repair()
                if args.check:
                    if current_state()==0:raise RuntimeError('WSLg 处于 COPY MODE')
                    return 0
                if current_state()==0 and not args.repair_if_idle:
                    raise RuntimeError('WSLg 处于 COPY MODE，请运行 repair_rviz.cmd')
                # When other GUI clients are active, try launching without resetting WSLg.
                process=subprocess.Popen(['rviz2','-d',str(config)])
                deadline=time.monotonic()+22
                while process.poll() is None and time.monotonic()<deadline:
                    window=visible_window(process.pid)
                    if window:
                        status('visible','RViz 窗口已打开',rviz_pid=process.pid,window=window)
                        code=process.wait()
                        if code in (0,-signal.SIGINT,-signal.SIGTERM,130):
                            status('closed','RViz 窗口已关闭')
                            return 0
                        break
                    time.sleep(.35)
                raise RuntimeError('RViz 提前退出或未检测到可见窗口')
            except (OSError,RuntimeError,subprocess.SubprocessError) as error:
                last_error=str(error)
                stop_child(process)
                if args.check or not args.repair_if_idle or attempt==3:break
                time.sleep(2)
        raise RuntimeError(last_error)
    except KeyboardInterrupt:return 0
    except Exception as exc:
        status('failed',str(exc)+'；可运行 repair_rviz.cmd 重开窗口')
        print('RVIZ_DISPLAY_ERROR: '+str(exc),flush=True)
        return 75
    finally:
        stop_child(process)

if __name__=='__main__':raise SystemExit(main())
