"""固定版原始点云注册、时间补偿和文件接口。"""

import json
from pathlib import Path
import cv2
import numpy as np
import yaml
import rosbag2_py as bagapi
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation, Slerp
from rgb_core import load_rig, cloud_arrays, transform, Poses

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT.parent / "session_20261124_100923_5cbed2"
PIC = ROOT.parent / "pic"
OUTPUT = ROOT / "runs/sam3_dense_road_120_150"
POSE_FILE = ROOT / "runs/ndt_ekf_full_v3/ekf_trajectory.csv"
PACK = np.dtype([("xyz", "<f4", 3), ("rgb", "u1", 3)])
TOPICS = [f"/point_cloud_hasco{i}" for i in range(1, 5)]
META = yaml.safe_load((DATA / "bag/metadata.yaml").read_text())["rosbag2_bagfile_information"]


class FullPoses(Poses):
    def __init__(self):
        """用全程 EKF 四元数轨迹复用逐点旋转插值，坐标和时间均来自同一次采集。"""
        values = np.loadtxt(POSE_FILE, delimiter=",", skiprows=1)
        self.times = values[:, 0]
        self.xyz = values[:, 1:4]
        self.start, self.end = self.times[[0, -1]]
        self.slerp = Slerp(self.times - self.start, Rotation.from_quat(values[:, 4:8]))


def messages(path, topics, start=None, end=None):
    """按记录时间筛选分片，只反序列化需要的话题。"""
    reader = bagapi.SequentialReader()
    reader.open(
        bagapi.StorageOptions(uri=str(path), storage_id="mcap"), bagapi.ConverterOptions("", "")
    )
    reader.set_filter(bagapi.StorageFilter(topics=topics))
    if start is not None:
        reader.seek(start)
    while reader.has_next():
        topic, raw, receipt = reader.read_next()
        if end is not None and receipt > end:
            break
        yield topic, raw


def color_scan(message, image, camera, lidar, poses, scan_time, image_time):
    """逐点去畸变并投到原始 KB4 图像，同一像素只取最近的可见测点。"""
    xyz, offsets = cloud_arrays(message)
    distance = np.linalg.norm(xyz, axis=1)
    keep = np.isfinite(offsets) & (distance > 1.5) & (distance < 100)
    world = poses.points_to_world(
        transform(lidar["reference_from_lidar"], xyz[keep]), scan_time + offsets[keep]
    )
    view = poses.at(image_time) @ camera["reference_from_camera"]
    optical = transform(np.linalg.inv(view), world)
    front = optical[:, 2] > 0.05
    optical, world = optical[front], world[front]
    uv, _ = cv2.fisheye.projectPoints(
        optical.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), camera["K"], camera["D"]
    )
    x, y = np.rint(uv.reshape(-1, 2)).astype(int).T
    h, w = image.shape[:2]
    valid = np.where((x >= 0) & (x < w) & (y >= 0) & (y < h))[0]
    ordered = valid[np.argsort(optical[valid, 2])]
    _, first = np.unique(y[ordered] * w + x[ordered], return_index=True)
    idx = ordered[first]
    return world[idx], image[y[idx], x[idx], ::-1], x[idx], y[idx], optical[idx, 2]


def read_map(path):
    """读取原始 XYZRGB 记录以保留坐标、颜色和体素密度。"""
    with path.open("rb") as f:
        header = []
        for line in f:
            header.append(line)
            if line == b"end_header\n":
                break
        return np.memmap(path, mode="r", dtype=PACK, offset=f.tell()), header


def draw(xyz, rgb, bounds, width, height):
    """按相同世界范围绘制顶视图，以便核查道路上方的残留。"""
    low, high = bounds
    scale = min((width - 8) / (high[0] - low[0]), (height - 8) / (high[1] - low[1]))
    x = np.rint((xyz[:, 0] - low[0]) * scale + 4).astype(int)
    y = np.rint((high[1] - xyz[:, 1]) * scale + 4).astype(int)
    visible = (x >= 0) & (x < width) & (y >= 0) & (y < height)
    ix = np.flatnonzero(visible)
    ordered = ix[np.argsort(xyz[ix, 2])]
    img = np.full((height, width, 3), 32, np.uint8)
    img[y[ordered], x[ordered]] = rgb[ordered, ::-1]
    return img
