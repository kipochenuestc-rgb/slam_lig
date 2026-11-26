"""同一段原始雷达构建 SAM3 过滤前后两份地图。"""

import json, sys, time
from pathlib import Path
import cv2, numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import core as b
from core import read_map, draw

s = b
OUT = b.OUTPUT
RUNS = b.ROOT / "runs"
VOXEL = 0.075
ZERO = 1795486169.500616
START, END = ZERO + 120, ZERO + 150


def label_for(g, cam):
    """等待 SAM3 对同一原始图的掩码写入完成，读取原像素尺寸。"""
    p = OUT / f"masks/{g:06d}_{cam}.png"
    while not p.exists():
        time.sleep(1)
    for _ in range(10):
        v = cv2.imread(str(p), 0)
        if v is not None and v.shape == (640, 800):
            return v
        time.sleep(0.5)
    raise ValueError(str(p))


def reject(labels, x, y, z):
    """去天空、人车像素，并以实例深度向轮廓外扩展少量边界。"""
    reason = labels[y, x].copy()
    for kind in [2, 3]:
        n, comp, boxes, _ = cv2.connectedComponentsWithStats((labels == kind).astype("u1"))
        pixel_comp = comp[y, x]
        for c in range(1, n):
            seeds = z[pixel_comp == c]
            if len(seeds) < 4:
                continue
            left, top, w, h, area = boxes[c]
            pad = int(np.clip(max(w, h) * 0.07, 3, 10))
            q = (
                (x >= left - pad)
                & (x < left + w + pad)
                & (y >= top - pad)
                & (y < top + h + pad)
                & (reason == 0)
            )
            if not q.any():
                continue
            lo, hi = np.percentile(seeds, [10, 85])
            same_depth = (z >= lo - 0.25) & (z <= hi + 0.3)
            dist = cv2.distanceTransform((comp != c).astype("u1"), cv2.DIST_L2, 3)
            reason[q & same_depth & (dist[y, x] <= pad)] = kind
    return reason


def point_keys(xyz):
    """按 7.5 厘米体素编码位置，保留原测量值。"""
    v = np.floor(xyz / VOXEL).astype("int64") + (1 << 20)
    assert v.min() >= 0 and v.max() < 1 << 21
    return (v[:, 0] << 42) | (v[:, 1] << 21) | v[:, 2]


def outliers(data):
    """复用工程 20 近邻统计筛选标准，仅返回实际记录的子集。"""
    n = len(data)
    tree = cKDTree(data["xyz"])
    means = np.empty(n, "f4")
    nearest = np.empty(n, "f4")
    for i in range(0, n, 50000):
        dist, _ = tree.query(data["xyz"][i : i + 50000], k=min(21, n), workers=6)
        means[i : i + len(dist)] = dist[:, 1:].mean(1)
        nearest[i : i + len(dist)] = dist[:, 1]
    threshold = float(means.mean() + 2 * means.std())
    keep = means <= threshold
    report = {
        "method": "20-NN mean distance <= global mean + 2 std",
        "threshold_m": threshold,
        "before": n,
        "after": int(keep.sum()),
        "removed": int((~keep).sum()),
        "nearest_neighbor_p50_before": float(np.median(nearest)),
        "nearest_neighbor_p50_after": float(np.median(nearest[keep])),
    }
    return data[keep], data[~keep], report


def voxel(data):
    """全段每 7.5 厘米体素保留一个原始 XYZRGB 点。"""
    _, idx = np.unique(point_keys(data["xyz"]), return_index=True)
    return data[idx]


def write(name, data):
    """写入独立 PLY，并验证读取的每个坐标和颜色。"""
    header = f"ply\nformat binary_little_endian 1.0\nelement vertex {len(data)}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n".encode()
    path = OUT / name
    with path.open("wb") as f:
        f.write(header)
        data.tofile(f)
    assert np.array_equal(read_map(path)[0], data)


def collect_scans():
    """按原始时间匹配相机并收集语义过滤前后的点观测。"""
    cams, lids, _, _ = s.load_rig(s.DATA / "calibration")
    poses = b.FullPoses()
    a = np.load(s.PIC / "poses.npz")
    sel = a["camera_id"] == 0
    stamps, groups = a["stamp_ns"][sel] * 1e-9, a["group_index"][sel]
    before = []
    after = []
    removed = []
    stat = np.zeros(4, np.int64)
    scans = 0
    for part in b.META["files"]:
        t = part["starting_time"]["nanoseconds_since_epoch"] * 1e-9
        if t > END + 1 or t + part["duration"]["nanoseconds"] * 1e-9 < START - 1:
            continue
        for topic, raw in s.messages(
            s.DATA / "bag" / part["path"], s.TOPICS, int((START - 1) * 1e9), int((END + 1) * 1e9)
        ):
            msg = s.deserialize_message(raw, s.PointCloud2)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if not START <= ts < END:
                continue
            gi = np.argmin(abs(stamps - ts))
            if abs(stamps[gi] - ts) > 0.02:
                continue
            lid = s.TOPICS.index(topic) + 1
            cam = lids[lid]["camera_id"]
            g = groups[gi]
            im = cv2.imread(str(s.PIC / f"images/cam{cam}/{g:06d}.png"))
            xyz, rgb, x, y, z = s.color_scan(msg, im, cams[cam], lids[lid], poses, ts, stamps[gi])
            labels = label_for(g, cam)
            why = reject(labels, x, y, z)
            data = np.empty(len(xyz), dtype=b.PACK)
            data["xyz"], data["rgb"] = xyz, rgb
            stat += np.bincount(why, minlength=4)
            before.append(data)
            after.append(data[why == 0])
            removed.append(data[why != 0])
            assert (labels[y[why == 0], x[why == 0]] == 0).all()
            if int(g) == 864:
                paint = im.copy()
                for xx, yy in zip(x[why != 0][::8], y[why != 0][::8]):
                    cv2.circle(paint, (xx, yy), 1, (0, 0, 255), -1)
                cv2.imwrite(str(OUT / f"projected_removed_cam{cam}.jpg"), paint)
            scans += 1
            # 每 40 帧压实体素，控制长片段重建时的内存占用。
            if scans % 40 == 0:
                before = [voxel(np.concatenate(before))]
                after = [voxel(np.concatenate(after))]
                removed = [voxel(np.concatenate(removed))]
    before = voxel(np.concatenate(before))
    after = voxel(np.concatenate(after))
    removed = voxel(np.concatenate(removed))
    return poses, before, after, removed, stat, scans


def main():
    """导出固定版本点云、离群点和统计报告。"""
    poses, before, after, removed, stat, scans = collect_scans()
    write("before_rgb.ply", before)
    write("sam3_before_outlier_filter.ply", after)
    after, isolated, filter_report = outliers(after)
    write("sam3_clean_rgb.ply", after)
    write("outliers_removed.ply", isolated)
    write("removed_observations.ply", removed)
    # 保持同一区域透视边界，用最高点作为顶视可见点。
    poses_xy = poses.at([START, END])[:, :2, 3]
    lo = poses_xy.min(0) - [12, 8]
    hi = poses_xy.max(0) + [12, 8]
    panels = []
    for data, name in [(before, "Before"), (after, "SAM 3")]:
        im = draw(data["xyz"], data["rgb"], (lo, hi), 850, 1050)
        cv2.putText(im, name, (12, 35), 0, 1, (70, 225, 255), 2)
        panels.append(im)
    cv2.imwrite(str(OUT / "comparison.jpg"), np.hstack(panels))
    # 通过同一地图高度范围统计此前人体拖影所在的低空区域，不据此删除。
    c = cKDTree(poses.xyz[:, :2])
    test = []
    for data in [before, after]:
        xyz = data["xyz"]
        dist, ix = c.query(xyz[:, :2], workers=4)
        h = xyz[:, 2] - poses.xyz[ix, 2]
        test.append(int(((dist < 3.5) & (h > 0.1) & (h < 1.8)).sum()))
    summary = dict(
        start_offset_s=120,
        duration_s=30,
        scans=scans,
        points_before=len(before),
        points_after=len(after),
        voxel_m=VOXEL,
        outlier_filter=filter_report,
        observations=dict(zip(["kept", "sky", "person", "vehicle"], stat.tolist())),
        low_air_near_trajectory_before_after=test,
        trajectory=str(b.POSE_FILE),
        calibration_status="candidate_not_deployed",
        mask_overlap_after=0,
    )
    (OUT / "report.json").write_text(json.dumps(summary, indent=2))
    print(summary, flush=True)


if __name__ == "__main__":
    main()
