"""Resolve rigid sensor poses from a bag's recorded frame IDs and static TF tree."""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

from fusion_core import TOPICS

ROOT = Path(__file__).resolve().parents[1]


def inspect_bag(bag, gnss_topic):
    args = ['bash', str(ROOT / 'scripts/ros_env.sh'), 'python3', __file__, str(bag), gnss_topic]
    proc = subprocess.run(args, capture_output=True, text=True)
    if proc.returncode:
        raise ValueError(proc.stderr.strip() or '无法读取 bag TF')
    return json.loads(proc.stdout.strip().splitlines()[-1])


def describe(bag, gnss_topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from scipy.spatial.transform import Rotation

    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if not info['relative_file_paths'] or any(not (bag / f).is_file() for f in info['relative_file_paths']):
        raise ValueError('bag 数据文件缺失')
    counts = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
    topics = [*TOPICS, '/imu/data', gnss_topic]
    frames, edges = {}, {}
    remaining = counts.get('/tf_static', 0)
    needed = {t for t in topics if counts.get(t)}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=info['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[*needed, '/tf_static']))
    while (remaining or needed) and reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in needed and topic != '/tf_static':
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic in needed:
            frames[topic] = msg.header.frame_id.strip('/')
            needed.remove(topic)
            continue
        remaining -= 1
        for tf in msg.transforms:
            parent, child = tf.header.frame_id.strip('/'), tf.child_frame_id.strip('/')
            tr, q = tf.transform.translation, tf.transform.rotation
            m = np.eye(4)
            m[:3, 3] = [tr.x, tr.y, tr.z]
            m[:3, :3] = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            if child in edges and (parent != edges[child][0] or not np.allclose(m,edges[child][1],atol=1e-7)):
                raise ValueError('冲突的静态 TF: ' + child)
            edges[child] = parent, m

    if remaining:
        raise ValueError('未能读取元数据列出的全部 /tf_static 消息')

    def root_pose(frame):
        m = np.eye(4)
        visited = set()
        while frame in edges:
            if frame in visited:
                raise ValueError('静态 TF 存在环: ' + frame)
            visited.add(frame)
            frame, transform = edges[frame]
            m = transform @ m
        return frame, m

    known_frames = set(edges) | {parent for parent, _ in edges.values()}
    root, root_from_base = root_pose('base_link')
    reference = np.linalg.inv(root_from_base) if 'base_link' in known_frames else None
    rows = []
    for name, topic in zip(['HASCO 1','HASCO 2','HASCO 3','HASCO 4','IMU','GNSS'],topics):
        frame = frames.get(topic, '')
        ancestor, pose = root_pose(frame)
        error = ''
        if not counts.get(topic): error = '无输入消息'
        elif not frame: error = '消息缺少 frame_id'
        elif topic in TOPICS and sum(frames.get(t)==frame for t in TOPICS)>1: error = '多路雷达共用 frame_id'
        elif frame not in known_frames: error = '静态 TF 树中缺少该帧'
        elif reference is None: error = '静态 TF 树中缺少 base_link'
        elif ancestor != root: error = '与参考系缺少静态 TF 链'
        rows.append(dict(name=name, topic=topic, count=counts.get(topic), frame=frame, error=error,
                         pose=None if error else (reference @ pose).tolist()))
    # Re-root the rigid rig at the moving SLAM body; never publish world/odometry edges.
    neighbours = {}
    for child, (parent, matrix) in edges.items():
        if {parent, child} & {'map', 'odom'}:
            continue
        neighbours.setdefault(parent, []).append((child, matrix))
        neighbours.setdefault(child, []).append((parent, np.linalg.inv(matrix)))
    static_tf = []
    pending = ['base_link']
    visited = set(pending)
    while pending:
        parent = pending.pop()
        for child, matrix in neighbours.get(parent, []):
            if child in visited:
                continue
            visited.add(child)
            pending.append(child)
            static_tf.append(dict(parent=parent, child=child, matrix=matrix.tolist()))
    return dict(rows=rows, static_tf=static_tf, reference_frame='base_link', source=str(bag) + ' · /tf_static', filesOk=True)


if __name__ == '__main__':
    print(json.dumps(describe(Path(sys.argv[1]), sys.argv[2]),allow_nan=False))
