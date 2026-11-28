"""Read time-indexed GNSS fixes without replaying messages into the ROS graph."""
from functools import lru_cache
from math import isfinite
from pathlib import Path

import yaml
from fastapi import HTTPException
from pyproj import Transformer
from pyproj.enums import TransformDirection
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from projection import GEOGRAPHIC_FROM_ENU

FIX = "sensor_msgs/msg/NavSatFix"
STORE = get_typestore(Stores.ROS2_JAZZY)


def bag_info(directory):
    bag = Path(directory)
    if not bag.is_absolute() or not (bag / "metadata.yaml").is_file():
        raise HTTPException(400, "请选择含 metadata.yaml 的 ROS 2 bag 目录")
    with Reader(bag) as reader:
        topics = sorted({c.topic for c in reader.connections if c.msgtype == FIX})
        return {"duration": reader.duration / 1e9, "topics": topics}


@lru_cache(maxsize=4)
def read_fixes(bag, topic, revision):
    with Reader(bag) as reader:
        connections = [c for c in reader.connections if c.topic == topic and c.msgtype == FIX]
        if not connections:
            raise HTTPException(400, "此 bag 中找不到指定的 NavSatFix topic")
        rows = []
        for _, receipt, raw in reader.messages(connections=connections):
            fix = STORE.deserialize_cdr(raw, FIX)
            llh = (fix.longitude, fix.latitude, fix.altitude)
            valid = (fix.status.status >= 0 and all(map(isfinite, llh))
                     and abs(llh[0]) <= 180 and abs(llh[1]) <= 90)
            rows.append(((receipt-reader.start_time)/1e9, fix.header.frame_id, llh if valid else None))
        return sorted(rows, key=lambda row: row[0])


def map_positions(directory, topic, folder):
    bag = Path(directory)
    if not bag.is_absolute() or not (bag / "metadata.yaml").is_file():
        raise HTTPException(400, "bag 目录无效")
    projector = yaml.safe_load((folder / "map_projector_info.yaml").read_text(encoding="utf-8"))
    if projector.get("projector_type") != "LocalCartesian" or projector.get("vertical_datum") != "WGS84":
        raise HTTPException(400, "此定位页支持 LocalCartesian / WGS84 地图")
    origin = projector["map_origin"]
    transform = Transformer.from_pipeline(GEOGRAPHIC_FROM_ENU.format(lat=float(origin["latitude"]),
                                                                     lon=float(origin["longitude"])))
    revision = tuple((p.name, p.stat().st_size, p.stat().st_mtime_ns)
                     for p in sorted(bag.iterdir()) if p.suffix in {".yaml", ".mcap", ".db3"})
    rows = read_fixes(bag, topic, revision)
    return {"origin": origin, "samples": [
        [t, frame, transform.transform(*llh, direction=TransformDirection.INVERSE) if llh else None]
        for t, frame, llh in rows]}
