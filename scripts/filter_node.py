#!/usr/bin/env python3
"""Bounded, ordered online filtering; never rewrites measurement timestamps."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import json,time,pathlib,multiprocessing,signal
from collections import deque
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from fusion_core import DTYPE
from filter_core import filter_points


def worker_init():
    signal.signal(signal.SIGINT,signal.SIG_IGN)


def work(data,options):
    started=time.monotonic()
    points,info=filter_points(np.frombuffer(data,dtype=DTYPE),query_workers=1,**options)
    return points.tobytes(),info,time.monotonic()-started


def main():
    rclpy.init();node=rclpy.create_node('fused_lidar_filter');options={}
    for name,value in [('min_range',1.5),('max_range',100.),('leaf_size',.2),('mean_k',20),('stddev_mul',2.)]:
        node.declare_parameter(name,value);options[name]=node.get_parameter(name).value
    node.declare_parameter('workers',4);node.declare_parameter('max_pending',16)
    workers=node.get_parameter('workers').value;limit=node.get_parameter('max_pending').value
    pool=ProcessPoolExecutor(max_workers=workers,mp_context=multiprocessing.get_context('spawn'),initializer=worker_init)
    pub=node.create_publisher(PointCloud2,'/lidars/filtered_points',qos_profile_sensor_data)
    status=node.create_publisher(String,'/lidars/filter_status',10)
    pending=deque();last={};received=published=dropped=0;started=time.monotonic()
    def callback(message):
        nonlocal received,dropped
        received+=1
        if message.point_step!=32 or len(message.data)!=message.height*message.row_step:raise ValueError('Invalid fused point layout')
        if len(pending)>=limit:
            dropped+=1;return
        pending.append((pool.submit(work,bytes(message.data),options),message.header,message.fields,time.monotonic()))
    def drain():
        nonlocal published,last
        while pending and pending[0][0].done():
            future,header,fields,queued=pending.popleft();data,info,compute_s=future.result()
            m=PointCloud2();m.header=header;m.fields=fields;m.height=1;m.width=len(data)//32;m.point_step=32;m.row_step=len(data);m.is_bigendian=False;m.is_dense=True;m.data=data
            pub.publish(m);published+=1;last={**info,'compute_s':compute_s,'queue_to_publish_s':time.monotonic()-queued,'stamp_s':header.stamp.sec+header.stamp.nanosec*1e-9}
    sub=node.create_subscription(PointCloud2,'/lidars/fused_points',callback,qos_profile_sensor_data)
    def report():
        result={'received':received,'published':published,'dropped_overload':dropped,'pending':len(pending),'workers':workers,'output_rate_hz':published/max(time.monotonic()-started,.01),'last_filter':last}
        msg=String();msg.data=json.dumps(result);status.publish(msg);print(msg.data,flush=True)
        (pathlib.Path(os.environ.get('SLAM_RUN_DIR',str(pathlib.Path(__file__).resolve().parents[1]/'runs')))/'live_filter_status.json').write_text(json.dumps(result,indent=2))
    node.create_timer(.01,drain);node.create_timer(2,report)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        pool.shutdown(wait=True,cancel_futures=True);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
