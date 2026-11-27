"""Export an LIO-SAM run as a local PCD or an Autoware LocalCartesian map."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def export_points(source, target, alignment):
    r = np.asarray(alignment['enu_to_map_rotation'], dtype=float)
    t = np.asarray(alignment['enu_to_map_translation'], dtype=float)
    lon, lat, height = alignment['reference_wgs84']
    if (r.shape != (3, 3) or t.shape != (3,) or not np.isfinite(t).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(r), 1)
            or not (-180 <= lon <= 180 and -90 < lat < 90 and np.isfinite(height))):
        raise ValueError('GNSS 对齐参数无效')
    lower = np.full(2, np.inf)
    upper = -lower
    with source.open('rb') as src, (target/'pointcloud_map.pcd').open('wb') as out:
        header = {}
        for _ in range(64):
            line = src.readline(2048)
            if not line:
                raise ValueError('缺少 PCD 头')
            out.write(line)
            fields = line.decode('ascii').split()
            if fields:
                header[fields[0]] = fields[1:]
                if fields[0] == 'DATA':
                    break
        if (header.get('FIELDS') != ['x','y','z','intensity'] or header.get('DATA') != ['binary']
                or header.get('TYPE') != ['F']*4 or header.get('SIZE') != ['4']*4
                or header.get('COUNT',['1']*4) != ['1']*4):
            raise ValueError('需要 LIO-SAM 的 binary XYZI PCD')
        remaining = int(header['POINTS'][0])
        if remaining <= 0:
            raise ValueError('点云为空')
        while remaining:
            n = min(remaining, 200_000)
            buf = src.read(n * 16)
            if len(buf) != n * 16:
                raise ValueError('PCD 长度与点数不符')
            points = np.frombuffer(buf, dtype='<f4').reshape(-1,4).copy()
            xyz = (points[:, :3] - t) @ r
            # Autoware loads LocalCartesian with origin altitude 0 (WGS84).
            xyz[:, 2] += height
            points[:, :3] = xyz
            if not np.isfinite(points[:, :3]).all():
                raise ValueError('点云含无效坐标')
            lower = np.minimum(lower, points[:, :2].min(axis=0))
            upper = np.maximum(upper, points[:, :2].max(axis=0))
            out.write(points.tobytes())
            remaining -= n
        if src.read(1):
            raise ValueError('PCD 含多余数据')
    corner = np.floor(lower)
    size = np.floor(upper - corner) + 1
    metadata = {'x_resolution':float(size[0]), 'y_resolution':float(size[1]),
                'pointcloud_map.pcd':corner.tolist()}
    projector = {'projector_type':'LocalCartesian','vertical_datum':'WGS84',
                 'map_origin':{'latitude':lat,'longitude':lon}}
    for filename, data in [('map_projector_info.yaml',projector), ('pointcloud_map_metadata.yaml',metadata)]:
        (target/filename).write_text(yaml.safe_dump(data,sort_keys=False))


def snapshot(run, directory):
    from session_control import active_mapping
    if not any(p.get('run') == str(run) for p in active_mapping()):
        raise ValueError('该任务没有可保存的实时地图')
    quality = json.loads((run/'quality.json').read_text())
    if not quality.get('finite') or quality.get('odom_count',0) < 10:
        raise ValueError('正在初始化，暂时没有有效地图')
    dest = '/' + str(directory.relative_to(Path.home()))
    cmd = ['bash',str(ROOT/'scripts/ros_env.sh'),'ros2','service','call','/lio_sam/save_map',
           'lio_sam/srv/SaveMap',json.dumps({'resolution':0.2,'destination':dest})]
    result = subprocess.run(cmd,capture_output=True,text=True,timeout=180)
    if result.returncode or 'success=True' not in result.stdout:
        raise ValueError('ROS 地图保存服务未成功返回')
    return directory/'GlobalMap.pcd'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('name')
    parser.add_argument('format',choices=['pcd','autoware'])
    args = parser.parse_args()
    run = args.run.resolve()
    if run.parent != (ROOT/'runs').resolve() or not run.is_dir():
        raise ValueError('运行目录无效')
    if (not re.fullmatch(r'[\w -]{1,64}',args.name) or args.name != args.name.strip()
            or re.fullmatch(r'(?i:con|prn|aux|nul|com[0-9]|lpt[0-9])',args.name)):
        raise ValueError('地图名称限 1–64 个文字、数字、空格、下划线或短横线')
    base = run/'exports'
    base.mkdir(exist_ok=True)
    dest = base/args.name
    if dest.exists():
        raise ValueError('该名称已存在，请更换地图名称')
    alignment = None
    if args.format == 'autoware':
        file = run/'gnss_alignment.json'
        if not file.is_file():
            raise ValueError('GNSS 尚未完成地理对齐，可先保存本地 PCD')
        alignment = json.loads(file.read_text())
    with tempfile.TemporaryDirectory(dir=base,prefix='.export-') as stage, tempfile.TemporaryDirectory(dir=Path.home()) as tmp:
        output = Path(stage)/'map'
        output.mkdir()
        source = run/'map/GlobalMap.pcd'
        if not source.is_file():
            source = snapshot(run,Path(tmp))
        if alignment:
            export_points(source,output,alignment)
        else:
            shutil.copyfile(source,output/'pointcloud_map.pcd')
        os.rename(output,dest)
    print(json.dumps({'files':sorted(p.name for p in dest.iterdir())}))


if __name__ == '__main__':
    try:
        main()
    except (OSError,ValueError,KeyError,subprocess.TimeoutExpired) as error:
        raise SystemExit(str(error))
