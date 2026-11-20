#!/usr/bin/env python3
"""Persistent mode menu keeps the WSL session alive between mapping tasks."""
import argparse,json,pathlib,signal,subprocess,sys

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run',action='store_true',help='Show selection without launching nodes')
    parser.add_argument('--start-mode',choices=['live','record','replay'],help='Start this mode first, then return to the persistent menu')
    args=parser.parse_args()
    initial=args.start_mode
    try:
        while True:
            print('1. 实时建图（接收传感器话题，不播放 bag）',flush=True)
            print('2. 只录制数据（之后再建图）',flush=True)
            print('3. 离线建图（自动播放已有 bag）',flush=True)
            print('0. 退出',flush=True)
            if initial:
                choice={'live':'1','record':'2','replay':'3'}[initial];initial=None
            else:
                while True:
                    choice=input('请选择 [1/2/3/0]: ').strip()
                    if choice=='0':return 0
                    if choice in ['1','2','3']:break
                    print('请输入 1、2、3 或 0。',flush=True)
            mode={'1':'live','2':'record','3':'replay'}[choice]
            command=[sys.executable,str(pathlib.Path(__file__).resolve().with_name('run_slam.py')),'--mode',mode]
            if mode=='replay':
                bag=input('Bag 文件夹路径（直接回车使用默认数据）: ').strip()
                if len(bag)>=2 and bag[0]==bag[-1]=='"':bag=bag[1:-1]
                if bag:command+=['--bag',bag]
            if args.dry_run:
                print('COMMAND_JSON='+json.dumps(command,ensure_ascii=False),flush=True);return 0
            # Ctrl+C goes to run_slam, which saves/stops its children cleanly.
            previous=signal.signal(signal.SIGINT,signal.SIG_IGN)
            try:result=subprocess.call(command)
            finally:signal.signal(signal.SIGINT,previous)
            print(f'任务已结束（退出码 {result}）。保留本窗口可保持 WSL 会话。',flush=True)
            if input('按回车返回菜单，输入 0 退出: ').strip()=='0':return result
    except (EOFError,KeyboardInterrupt):
        print('\n已取消。',flush=True);return 0

if __name__=='__main__':raise SystemExit(main())
