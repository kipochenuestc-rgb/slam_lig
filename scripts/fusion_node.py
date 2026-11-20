#!/usr/bin/env python3
import json,os
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from fusion_core import FrameMatcher, load_transforms, fuse_messages, TOPICS, OUTPUT_TOPIC

def main():
    rclpy.init();node=rclpy.create_node('four_lidar_fusion')
    node.declare_parameter('tolerance_ms',20.0)
    node.declare_parameter('max_wait_s',1.0)
    node.declare_parameter('calibration_dir',str(Path(__file__).resolve().parents[1]/'config/calibration'))
    matcher=FrameMatcher(node.get_parameter('tolerance_ms').value,node.get_parameter('max_wait_s').value)
    transforms=load_transforms(node.get_parameter('calibration_dir').value)
    publisher=node.create_publisher(PointCloud2,OUTPUT_TOPIC,qos_profile_sensor_data)
    status=node.create_publisher(String,'/lidars/fusion_status',10)
    last={}
    def callback(sensor,message):
        nonlocal last
        for group in matcher.push(sensor,message):
            merged,last=fuse_messages(group,transforms)
            publisher.publish(merged)
    subscriptions=[node.create_subscription(PointCloud2,t,lambda m,s=i:callback(s,m),qos_profile_sensor_data) for i,t in enumerate(TOPICS)]
    def report():
        result={'statistics':dict(matcher.stats),'last_fusion':last}
        message=String();message.data=json.dumps(result);status.publish(message);print(message.data,flush=True)
        (Path(os.environ.get('SLAM_RUN_DIR',str(Path(__file__).resolve().parents[1]/'runs')))/'fusion_status.json').write_text(json.dumps(result,indent=2))
    node.create_timer(2,report)
    node.get_logger().info('Four-lidar fusion ready; lidar 1 reference, 20 ms default tolerance, complete sets required')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
