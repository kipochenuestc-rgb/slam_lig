"""Generate a web preview of a completed, geographically aligned mapping run."""
import hashlib
import csv
import json
import tempfile
from pathlib import Path
from threading import Lock

import numpy as np
from pyproj import Transformer
import yaml

lock = Lock()
GEOGRAPHIC_FROM_ENU = """+proj=pipeline +step +inv +proj=topocentric +ellps=WGS84
    +lat_0={lat} +lon_0={lon} +h_0=0 +step +inv +proj=cart +ellps=WGS84"""


def _pcd_sample(source: Path):
    with source.open("rb") as f:
        header = {}
        for _ in range(64):
            fields = f.readline(2048).decode("ascii").split()
            if fields:
                header[fields[0]] = fields[1:]
                if fields[0] == "DATA":
                    break
        if (header.get("DATA") != ["binary"] or header.get("FIELDS") != ["x","y","z","intensity"]
                or header.get("TYPE") != ["F"] * 4 or header.get("SIZE") != ["4"] * 4
                or header.get("COUNT", ["1"] * 4) != ["1"] * 4):
            raise ValueError("需要 binary XYZI PCD")
        count = int(header["POINTS"][0])
        if count <= 0 or source.stat().st_size != f.tell() + count * 16:
            raise ValueError("点云长度无效")
        cloud = np.memmap(source, mode="r", dtype="<f4", offset=f.tell(), shape=(count, 4))
        points = cloud[np.linspace(0, count - 1, min(120000, count), dtype=int)].copy()
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError("点云没有有效坐标")
    return points, count


def project_autoware(folder: Path) -> Path:
    source = folder / "pointcloud_map.pcd"
    projector_file = folder / "map_projector_info.yaml"
    signature = [str(folder.resolve()), source.stat().st_size, source.stat().st_mtime_ns,
                 projector_file.stat().st_mtime_ns]
    cache = Path(tempfile.gettempdir()) / "slam-lig-atlas"
    cache.mkdir(exist_ok=True)
    dest = cache / hashlib.sha256(json.dumps(signature).encode()).hexdigest()
    with lock:
        if dest.is_dir():
            return dest
        projector = yaml.safe_load(projector_file.read_text(encoding="utf-8"))
        if projector.get("projector_type") != "LocalCartesian" or projector.get("vertical_datum") != "WGS84":
            raise ValueError("请选择 LocalCartesian / WGS84 的 Autoware 地图")
        origin = projector["map_origin"]
        lat, lon = float(origin["latitude"]), float(origin["longitude"])
        to_wgs = Transformer.from_pipeline(GEOGRAPHIC_FROM_ENU.format(lat=lat, lon=lon))
        points, count = _pcd_sample(source)
        lower, upper = points[:, :3].min(axis=0), points[:, :3].max(axis=0)
        bounds = np.asarray([[lower[0], lower[1], 0], [upper[0], upper[1], 0]])
        corners = np.column_stack(to_wgs.transform(*bounds.T))[:, :2]
        lonlat = np.column_stack(to_wgs.transform(*points[:, :3].T))[:, :2]
        tracks = {"fused": [], "gnss": []}
        run = folder.parent.parent
        alignment_file = run / "gnss_alignment.json"
        if folder.parent.name == "exports" and alignment_file.is_file():
            alignment = json.loads(alignment_file.read_text())
            rotation = np.asarray(alignment["enu_to_map_rotation"])
            translation = np.asarray(alignment["enu_to_map_translation"])
            for key, filename in [("fused", "trajectory.csv"), ("gnss", "gnss_constraints.csv")]:
                track_file = run / filename
                if not track_file.is_file():
                    continue
                with track_file.open(newline="") as f:
                    rows = [[float(row[k]) for k in ("x", "y", "z")] for row in csv.DictReader(f)]
                if rows:
                    tracks[key] = ((np.asarray(rows) - translation) @ rotation).tolist()
        quality_file = run / "quality.json"
        quality = json.loads(quality_file.read_text()) if folder.parent.name == "exports" and quality_file.is_file() else {}
        meta = dict(datasetId=folder.name, datasetLabel="Autoware 地图 · " + folder.name,
                    pointCount=len(points), sourcePointCount=count,
                    boundsENU=[float(lower[0]), float(lower[1]), float(upper[0]), float(upper[1])],
                    boundsWGS84=[*corners[0], *corners[1]],
                    zRange=[float(lower[2]), float(upper[2])],
                    zColorRange=np.percentile(points[:, 2], [5, 95]).tolist(),
                    distanceM=float(quality.get("path_length_m", 0)),
                    durationS=float(quality.get("elapsed_sensor_s", 0)),
                    gnssFactors=len(tracks["gnss"]), qualityNote="根据 map_projector_info.yaml 地理原点投影；点云为抽样预览。")
        with tempfile.TemporaryDirectory(dir=cache) as temp:
            stage = Path(temp) / "preview"
            stage.mkdir()
            points.tofile(stage / "points.bin")
            stride = max(1, len(points) // 20000)
            feature = dict(type="Feature", properties={},
                           geometry=dict(type="MultiPoint", coordinates=lonlat[::stride].tolist()))
            payloads = {"metadata.json": meta, "tracks.json": tracks,
                        "pointcloud_2d.geojson": dict(type="FeatureCollection", features=[feature])}
            for name, data in payloads.items():
                (stage / name).write_text(json.dumps(data, default=float, allow_nan=False), encoding="utf-8")
            stage.rename(dest)
        return dest


def project(run: Path) -> Path:
    dest = run / "atlas_preview"
    with lock:
        if dest.is_dir():
            return dest
        alignment = json.loads((run / "gnss_alignment.json").read_text())
        rotation = np.array(alignment["enu_to_map_rotation"])
        translation = np.array(alignment["enu_to_map_translation"])
        source = run / "map/GlobalMap.pcd"
        points, count = _pcd_sample(source)
        points[:, :3] = (points[:, :3] - translation) @ rotation
        lo, hi = points[:, :3].min(axis=0), points[:, :3].max(axis=0)
        to_wgs = Transformer.from_crs(4978, 4979, always_xy=True)

        def lonlat(enu):
            ecef = enu @ np.array(alignment["ecef_to_enu"]) + alignment["ecef_origin"]
            lon, lat, _ = to_wgs.transform(*ecef.T)
            return np.column_stack((lon, lat))

        corners = lonlat(np.array([[*lo[:2], 0], [*hi[:2], 0]]))
        tracks = {}
        for key, filename in [("fused","trajectory.csv"), ("gnss","gnss_constraints.csv")]:
            path = run / filename
            rows = []
            if path.exists():
                with path.open(newline="") as f:
                    for row in csv.DictReader(f):
                        rows.append([float(row[k]) for k in ("x","y","z")])
            tracks[key] = ((np.asarray(rows) - translation) @ rotation).tolist() if rows else []
        quality = json.loads((run / "quality.json").read_text())
        meta = dict(datasetId=run.name, datasetLabel="离线建图 · " + run.name,
                    pointCount=len(points), sourcePointCount=count, boundsENU=[*lo[:2], *hi[:2]],
                    boundsWGS84=corners.ravel().tolist(), zRange=[lo[2], hi[2]],
                    zColorRange=np.percentile(points[:,2], [5,95]).tolist(),
                    distanceM=quality["path_length_m"], durationS=quality["elapsed_sensor_s"],
                    gnssFactors=len(tracks["gnss"]), qualityNote="按本次建图的 GNSS 对齐结果投影，点云为抽样预览。")
        with tempfile.TemporaryDirectory(dir=run) as tmp:
            stage = Path(tmp) / "preview"
            stage.mkdir()
            points.tofile(stage / "points.bin")
            xy = lonlat(points[::max(1,len(points)//20000), :3])
            feature = dict(type="Feature", properties={}, geometry=dict(type="MultiPoint", coordinates=xy.tolist()))
            for name, data in [("metadata.json",meta), ("tracks.json",tracks),
                               ("pointcloud_2d.geojson",dict(type="FeatureCollection", features=[feature]))]:
                (stage / name).write_text(json.dumps(data, default=float, allow_nan=False), encoding="utf-8")
            stage.rename(dest)
        return dest
