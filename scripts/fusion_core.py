"""Four-lidar rigid fusion, referenced to lidar 1's header and origin.

This stage does not claim motion deskewing: individual point times are retained
for LIO-SAM's subsequent IMU deskew. Synthetic elevation bins are not real rings.
"""
from collections import Counter, deque
import json
from pathlib import Path
import numpy as np

TOPICS = [f'/point_cloud_hasco{i}' for i in range(1, 5)]
OUTPUT_TOPIC = '/lidars/fused_points'
OUTPUT_FRAME = 'hasco1_lio'
DTYPE = np.dtype({'names': ['x','y','z','intensity','ring','time','lidar_id','source_time_us'],
    'formats': ['<f4','<f4','<f4','<f4','<u2','<f4','u1','<u4'],
    'offsets': [0,4,8,12,16,20,24,28], 'itemsize': 32})

def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

def load_transforms(calibration_dir):
    p = Path(calibration_dir)
    calibration = json.loads((p/'results/rig_cam0_with_lidars_paired_1203.json').read_text())
    mount = json.loads((p/'config/imu_mount.json').read_text())
    imu_from_camera = np.eye(4)
    imu_from_camera[:3,:3] = np.column_stack([mount['camera0_axes_in_imu'][k] for k in ['x','y','z']])
    imu_from_camera[:3,3] = mount['camera0_origin_in_imu_m']
    imu_from_lidars = [imu_from_camera @ np.array(calibration['lidars'][str(i)]['T_reference_from_lidar']) for i in range(4)]
    # Output axes are IMU-aligned, but the origin is lidar 1's physical center.
    origin = imu_from_lidars[0][:3,3].copy()
    result = [m.copy() for m in imu_from_lidars]
    for m in result:
        m[:3,3] -= origin
    return result

class FrameMatcher:
    """Wait for per-sensor timestamp watermarks, then pick nearest frames.

    A complete set is mandatory; a frame is never reused. Missing sensors do not
    cause a frame from a different 100 ms cycle to be silently substituted.
    """
    def __init__(self, tolerance_ms=20.0, max_wait_s=1.0):
        self.tolerance_ns = round(tolerance_ms * 1e6)
        self.max_wait_ns = round(max_wait_s * 1e9)
        self.buffers = [deque() for _ in range(4)]
        self.latest = [None] * 4
        self.stats = Counter()

    def push(self, sensor, message):
        timestamp = stamp_ns(message)
        self.stats[f'received_{sensor+1}'] += 1
        if self.latest[sensor] is not None and timestamp <= self.latest[sensor]:
            self.stats[f'rejected_nonmonotonic_{sensor+1}'] += 1
            return []
        self.latest[sensor] = timestamp
        self.buffers[sensor].append(message)
        return self.drain()

    def drain(self, flush=False):
        result = []
        while self.buffers[0]:
            reference = self.buffers[0][0]
            timestamp = stamp_ns(reference)
            ready = all(s is not None and s >= timestamp + self.tolerance_ns for s in self.latest[1:])
            expired = self.latest[0] - timestamp >= self.max_wait_ns
            if not (ready or expired or flush):
                break
            self.buffers[0].popleft()
            matched = [reference]
            missing = []
            for sensor in range(1, 4):
                buffer = self.buffers[sensor]
                while buffer and stamp_ns(buffer[0]) < timestamp-self.tolerance_ns:
                    buffer.popleft()
                    self.stats[f'unused_{sensor+1}'] += 1
                candidate = min(buffer, key=lambda m: abs(stamp_ns(m)-timestamp), default=None)
                if candidate is None or abs(stamp_ns(candidate)-timestamp) > self.tolerance_ns:
                    missing.append(sensor+1)
                matched.append(candidate)
            if missing:
                self.stats['dropped_incomplete_reference_frames'] += 1
                for sensor in missing:
                    self.stats[f'missing_{sensor}'] += 1
                continue
            for sensor in range(1, 4):
                candidate = matched[sensor]
                while self.buffers[sensor]:
                    popped = self.buffers[sensor].popleft()
                    if popped is candidate:
                        break
                    self.stats[f'unused_{sensor+1}'] += 1
            self.stats['matched_complete_sets'] += 1
            result.append(matched)
        # Sensors may start before lidar 1; keep bounded timestamp histories.
        if self.latest[0] is not None:
            oldest = stamp_ns(self.buffers[0][0]) if self.buffers[0] else self.latest[0]
            for sensor in range(1, 4):
                while self.buffers[sensor] and stamp_ns(self.buffers[sensor][0]) < oldest-self.tolerance_ns:
                    self.buffers[sensor].popleft()
        return result

def fuse_messages(messages, transforms):
    from sensor_msgs.msg import PointCloud2, PointField
    reference_ns = stamp_ns(messages[0])
    parts, counts, offsets, invalid = [], [], [], 0
    for sensor, (message, transform) in enumerate(zip(messages, transforms)):
        fields = {f.name:f for f in message.fields}
        for name in ['x','y','z','intensity','timestamp']:
            if name not in fields:
                raise ValueError(f'{TOPICS[sensor]} missing {name}')
        if fields['timestamp'].datatype != PointField.UINT32:
            raise ValueError('Expected hardware-driver timestamp uint32 microseconds')
        endian = '>' if message.is_bigendian else '<'
        formats = {PointField.UINT8:'u1',PointField.UINT16:'u2',PointField.UINT32:'u4',PointField.FLOAT32:'f4',PointField.FLOAT64:'f8'}
        def channel(name):
            f=fields[name]
            return np.ndarray((message.height,message.width),dtype=endian+formats[f.datatype],buffer=message.data,offset=f.offset,strides=(message.row_step,message.point_step)).ravel()
        xyz = np.column_stack([channel(k) for k in ['x','y','z']]) @ transform[:3,:3].T + transform[:3,3]
        micros = channel('timestamp')
        valid = np.isfinite(xyz).all(axis=1) & (micros <= 200_000)
        invalid += int((~valid).sum())
        xyz, micros = xyz[valid], micros[valid]
        offset_ns = stamp_ns(message)-reference_ns
        relative = offset_ns*1e-9 + micros.astype(np.float64)*1e-6
        points = np.zeros(len(xyz),dtype=DTYPE)
        for i,k in enumerate(['x','y','z']):points[k]=xyz[:,i]
        points['intensity']=channel('intensity')[valid]
        points['time']=relative
        points['source_time_us']=micros
        points['lidar_id']=sensor+1
        elevation=np.degrees(np.arctan2(xyz[:,2],np.hypot(xyz[:,0],xyz[:,1])))
        row=np.floor((elevation+50)/.5).astype(np.int32)
        points['ring']=np.where((row>=0)&(row<200),row,65535)
        parts.append(points);counts.append(len(points));offsets.append(offset_ns)
    # np.concatenate can canonicalize structured dtypes and remove explicit
    # padding. Copy into the declared ROS wire layout instead.
    points=np.zeros(sum(len(part) for part in parts),dtype=DTYPE)
    cursor=0
    for part in parts:
        points[cursor:cursor+len(part)]=part
        cursor+=len(part)
    points=points[np.argsort(points['time'],kind='stable')]
    message=PointCloud2();message.header.stamp=messages[0].header.stamp;message.header.frame_id=OUTPUT_FRAME
    message.height=1;message.width=len(points);message.is_bigendian=False;message.is_dense=True;message.point_step=32;message.row_step=32*len(points);message.data=points.tobytes()
    message.fields=[PointField(name=k,offset=offset,datatype=datatype,count=1) for k,offset,datatype in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',20,7),('lidar_id',24,2),('source_time_us',28,6)]]
    if len(message.data) != message.height*message.row_step:
        raise ValueError('PointCloud2 wire layout size mismatch')
    info={'reference_ns':reference_ns,'source_offsets_ms':[x/1e6 for x in offsets],'points_per_lidar':counts,'points':len(points),'invalid_points_removed':invalid,'relative_time_min_s':float(points['time'][0]) if len(points) else 0,'relative_time_max_s':float(points['time'][-1]) if len(points) else 0}
    return message,info
