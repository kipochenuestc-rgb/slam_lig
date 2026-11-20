import pathlib,time,json,csv,sys,numpy as np
import rclpy
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import PointCloud2
from lio_sam.msg import CloudInfo
from rclpy.qos import qos_profile_sensor_data
out=pathlib.Path(sys.argv[1]);out.mkdir(exist_ok=True)
rclpy.init();n=rclpy.create_node('lio_quality_monitor');rows=[];features=[];ncloud=[0];last=[time.monotonic()]
def odom(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation;rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
def feat(m):features.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.cloud_corner.width*m.cloud_corner.height,m.cloud_surface.width*m.cloud_surface.height])
def cloud(m):ncloud[0]+=1
subs=[n.create_subscription(Odometry,'/lio_sam/mapping/odometry',odom,qos_profile_sensor_data),n.create_subscription(CloudInfo,'/lio_sam/feature/cloud_info',feat,qos_profile_sensor_data),n.create_subscription(PointCloud2,'/lio_sam/mapping/map_global',cloud,qos_profile_sensor_data)]
try:
 while rclpy.ok():
  rclpy.spin_once(n,timeout_sec=.2)
  if time.monotonic()-last[0]>10:
   result={'odom_count':len(rows),'feature_count':len(features),'global_map_count':ncloud[0],'last_odometry':rows[-1] if rows else None,'last_features':features[-1] if features else None}
   if rows:
    a=np.array(rows);result.update(elapsed_sensor_s=float(a[-1,0]-a[0,0]),position_min=a[:,1:4].min(axis=0).tolist(),position_max=a[:,1:4].max(axis=0).tolist(),path_length_m=float(np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1).sum()),finite=bool(np.isfinite(a).all()))
   (out/'quality.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
   with (out/'trajectory.csv').open('w',newline='') as f:w=csv.writer(f);w.writerow(['stamp_s','x','y','z','qx','qy','qz','qw']);w.writerows(rows)
   with (out/'features.csv').open('w',newline='') as f:w=csv.writer(f);w.writerow(['stamp_s','edge_points','surface_points']);w.writerows(features)
   last[0]=time.monotonic()
except KeyboardInterrupt:pass
n.destroy_node();rclpy.shutdown()
