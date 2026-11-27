"""Publish the saved optimized map and trajectory for RViz, without replay."""
import csv,math,pathlib
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_ros import StaticTransformBroadcaster
from analyze_run import read_pcd

root=pathlib.Path(__import__('sys').argv[1])
parts=[read_pcd(root/'map'/name) for name in ['CornerMap.pcd','SurfMap.pcd']]
a=np.concatenate([np.column_stack([p[k] for k in ['x','y','z','intensity']]) for p in parts])
_,indices=np.unique(np.floor(a[:,:3]/.4).astype(np.int32),axis=0,return_index=True);a=a[indices].astype('<f4')
rclpy.init();node=rclpy.create_node('saved_four_lidar_map')
tf=StaticTransformBroadcaster(node);transform=TransformStamped();transform.header.stamp=node.get_clock().now().to_msg();transform.header.frame_id='map';transform.child_frame_id='odom';transform.transform.rotation.w=1.;tf.sendTransform(transform)
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub=node.create_publisher(PointCloud2,'/lio_sam/mapping/map_global',qos)
pathpub=node.create_publisher(Path,'/lio_sam/mapping/path',qos)
cloud=PointCloud2();cloud.header.frame_id='map';cloud.height=1;cloud.width=len(a);cloud.point_step=16;cloud.row_step=len(a)*16;cloud.is_dense=True;cloud.data=a.tobytes();cloud.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z','intensity'])]
track=Path();track.header.frame_id='map'
trajectory = root/'optimized_trajectory.csv'
if not trajectory.exists():trajectory=root/'trajectory.csv'
for row in csv.DictReader(trajectory.open()) if trajectory.exists() else ():
    p=PoseStamped();p.header.frame_id='map';p.pose.position.x=float(row['x']);p.pose.position.y=float(row['y']);p.pose.position.z=float(row['z'])
    if 'qw' in row:
        q=p.pose.orientation
        q.x,q.y,q.z,q.w=[float(row[k]) for k in ('qx','qy','qz','qw')]
        track.poses.append(p)
        continue
    r,pitch,y=[float(row[k])/2 for k in ['roll','pitch','yaw']];cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(pitch),math.sin(pitch),math.cos(y),math.sin(y)
    p.pose.orientation.x=sr*cp*cy-cr*sp*sy;p.pose.orientation.y=cr*sp*cy+sr*cp*sy;p.pose.orientation.z=cr*cp*sy-sr*sp*cy;p.pose.orientation.w=cr*cp*cy+sr*sp*sy;track.poses.append(p)
def publish():
    stamp=node.get_clock().now().to_msg();cloud.header.stamp=stamp;track.header.stamp=stamp
    for p in track.poses:p.header.stamp=stamp
    pub.publish(cloud);pathpub.publish(track)
node.create_timer(3,publish);publish();print(f'SAVED_MAP_READY points={len(a)} poses={len(track.poses)}',flush=True)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
node.destroy_node();rclpy.shutdown()
