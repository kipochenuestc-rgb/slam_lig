#!/usr/bin/env python3
"""Live NavSatFix and IMU bridge; no CSV input or replay-clock scheduling."""
import bisect,csv,json,os,pathlib
from collections import Counter,deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,NavSatFix
from nav_msgs.msg import Odometry
from gnss_core import LocalENU,fix_covariance,fit_alignment

ROOT=pathlib.Path(__file__).resolve().parents[1]
def stamp_ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec

class GNSSBridge(Node):
    def __init__(self,**kwargs):
        super().__init__('gnss_position_bridge',**kwargs)
        defaults={'gnss_topic':'/gnss/fix_fixed','imu_topic':'/imu/data','odom_topic':'/lio_sam/mapping/odometry','gnss_output_topic':'/lio_gnss/odometry','imu_output_topic':'/imu/calibrated','map_frame':'map','alignment_mode':'online','georeference_file':str(ROOT/'config/georeference.json'),'imu_bias_file':str(ROOT/'config/imu_initial_bias.json'),'apply_imu_bias':True,'min_alignment_points':12,'min_alignment_baseline_m':15.,'max_alignment_rmse_m':1.5,'alignment_window_s':120.,'max_odom_gap_s':1.,'unknown_horizontal_std_m':1.,'unknown_vertical_std_m':1.5,'max_horizontal_variance_m2':25.}
        for name,value in defaults.items():self.declare_parameter(name,value)
        self.declare_parameter('gnss_enabled',True)
        self.p=lambda name:self.get_parameter(name).value
        self.output=pathlib.Path(os.environ.get('SLAM_RUN_DIR',str(ROOT/'runs/live_bridge')));self.output.mkdir(parents=True,exist_ok=True)
        self.count=Counter();self.local=None;self.rotation=None;self.translation=None;self.metrics={};self.last_fix_ns=-1
        self.odometry=deque(maxlen=3000);self.pending=deque(maxlen=1000);self.pairs=deque(maxlen=1000)
        self.gyro_bias=np.zeros(3);self.accel_bias=np.zeros(3)
        if self.p('apply_imu_bias'):
            bias=json.loads(pathlib.Path(self.p('imu_bias_file')).read_text());self.gyro_bias=np.array(bias['gyro_bias_rad_s']);self.accel_bias=np.array(bias['accel_bias_m_s2'])
        if not self.p('gnss_enabled'):pass
        elif self.p('alignment_mode')=='fixed':
            config=json.loads(pathlib.Path(self.p('georeference_file')).read_text());self.local=LocalENU(*config['reference_wgs84']);self.rotation=np.eye(3);self.rotation[:2,:2]=config['map_to_enu_R_xy'];self.translation=np.r_[-np.array(config['map_to_enu_translation_xy'])@self.rotation[:2,:2].T,-config['map_to_enu_z_offset']];self.metrics={'mode':'fixed'}
        elif self.p('alignment_mode')!='online':raise ValueError('alignment_mode must be online or fixed')
        self.gps_pub=self.create_publisher(Odometry,self.p('gnss_output_topic'),100) if self.p('gnss_enabled') else None
        self.imu_pub=self.create_publisher(Imu,self.p('imu_output_topic'),qos_profile_sensor_data)
        self.input_subscriptions=[self.create_subscription(Imu,self.p('imu_topic'),self.imu,qos_profile_sensor_data)]
        if self.p('gnss_enabled'):self.input_subscriptions += [self.create_subscription(NavSatFix,self.p('gnss_topic'),self.gnss,qos_profile_sensor_data),self.create_subscription(Odometry,self.p('odom_topic'),self.odom,qos_profile_sensor_data)]
        self.csv_file=(self.output/'gnss_constraints.csv').open('w',newline='');self.csv_writer=csv.writer(self.csv_file);self.csv_writer.writerow(['stamp_s','x','y','z']);self.csv_file.flush()
        self.create_timer(2,self.report)
        if self.rotation is not None:self.save_alignment()
        self.get_logger().info(f"GNSS enabled={self.p('gnss_enabled')}; IMU input={self.p('imu_topic')}; measurement stamps preserved")

    def imu(self,m):
        for k,b in zip(['x','y','z'],self.gyro_bias):setattr(m.angular_velocity,k,getattr(m.angular_velocity,k)-b)
        for k,b in zip(['x','y','z'],self.accel_bias):setattr(m.linear_acceleration,k,getattr(m.linear_acceleration,k)-b)
        self.imu_pub.publish(m);self.count['imu_published']+=1

    def gnss(self,m):
        if not self.p('gnss_enabled'):return
        self.count['fix_received']+=1;ns=stamp_ns(m)
        if m.status.status<0 or not np.isfinite([m.longitude,m.latitude,m.altitude]).all() or not -180<=m.longitude<=180 or not -90<m.latitude<90:
            self.count['invalid_fix']+=1;return
        if ns<=0 or ns<=self.last_fix_ns:self.count['invalid_or_out_of_order_stamp']+=1;return
        c=fix_covariance(m.position_covariance,m.position_covariance_type!=NavSatFix.COVARIANCE_TYPE_UNKNOWN,self.p('unknown_horizontal_std_m'),self.p('unknown_vertical_std_m'))
        if max(c[0,0],c[1,1])>self.p('max_horizontal_variance_m2'):self.count['poor_covariance']+=1;return
        self.last_fix_ns=ns
        if self.local is None:self.local=LocalENU(m.longitude,m.latitude,m.altitude)
        enu=self.local.convert(m.longitude,m.latitude,m.altitude)
        if self.rotation is not None:self.publish_fix(m,enu,c);return
        if len(self.pending)==self.pending.maxlen:self.count['alignment_queue_overflow']+=1
        self.pending.append((ns,enu));self.align()

    def odom(self,m):
        if self.rotation is not None:return
        ns=stamp_ns(m);p=m.pose.pose.position;xyz=np.array([p.x,p.y,p.z])
        if not np.isfinite(xyz).all() or ns<=0:return
        if self.odometry and ns<=self.odometry[-1][0]:self.count['nonmonotonic_odometry']+=1;return
        self.odometry.append((ns,xyz));self.align()

    def align(self):
        if self.rotation is not None or len(self.odometry)<2:return
        times=[row[0] for row in self.odometry]
        while self.pending and self.pending[0][0]<=times[-1]:
            ns,enu=self.pending.popleft();index=bisect.bisect_left(times,ns)
            if index==0:
                if ns!=times[0]:self.count['fix_before_odometry']+=1;continue
                position=self.odometry[0][1]
            else:
                ta,a=self.odometry[index-1];tb,b=self.odometry[index]
                if (tb-ta)*1e-9>self.p('max_odom_gap_s'):self.count['alignment_odom_gap']+=1;continue
                position=a+(b-a)*((ns-ta)/(tb-ta))
            self.pairs.append((ns,enu,position))
            while self.pairs and (ns-self.pairs[0][0])*1e-9>self.p('alignment_window_s'):self.pairs.popleft()
        result=fit_alignment([p[1] for p in self.pairs],[p[2] for p in self.pairs],self.p('min_alignment_points'),self.p('min_alignment_baseline_m'),self.p('max_alignment_rmse_m'))
        if result is not None:
            self.rotation,self.translation,self.metrics=result;self.metrics['mode']='online';self.pending.clear();self.save_alignment()
            self.get_logger().info('GNSS alignment ready: '+json.dumps(self.metrics))
            # Past alignment fixes are not published as stale factors. Only new
            # incoming fixes publish immediately, ahead of delayed LIO callbacks.

    def publish_fix(self,m,enu,c):
        position=self.rotation@enu+self.translation;cov=self.rotation@c@self.rotation.T
        result=Odometry();result.header.stamp=m.header.stamp;result.header.frame_id=self.p('map_frame');result.child_frame_id=m.header.frame_id or 'gnss_antenna'
        result.pose.pose.position.x=float(position[0]);result.pose.pose.position.y=float(position[1]);result.pose.pose.position.z=float(position[2]);result.pose.pose.orientation.w=1.
        for i in range(3):
            for j in range(3):result.pose.covariance[i*6+j]=float(cov[i,j])
        for i in [21,28,35]:result.pose.covariance[i]=1e6
        self.gps_pub.publish(result);self.count['gnss_published']+=1
        self.csv_writer.writerow([stamp_ns(m)*1e-9,*position.tolist()]);self.csv_file.flush()

    def save_alignment(self):
        r=self.rotation;t=self.translation
        result={'reference_wgs84':self.local.reference,'ecef_origin':self.local.origin.tolist(),'ecef_to_enu':self.local.rotation.tolist(),'enu_to_map_rotation':r.tolist(),'enu_to_map_translation':t.tolist(),'map_to_enu_R_xy':r[:2,:2].tolist(),'map_to_enu_translation_xy':(-t[:2]@r[:2,:2]).tolist(),'map_to_enu_z_offset':float(-t[2]),'alignment':self.metrics,'note':'Frozen causal alignment; antenna lever arm not compensated'}
        (self.output/'gnss_alignment.json').write_text(json.dumps(result,indent=2))

    def report(self):
        result={'state':'ready' if self.rotation is not None else 'waiting_for_motion_and_matching_odometry','input_topic':self.p('gnss_topic'),'counts':dict(self.count),'alignment_pairs':len(self.pairs),'pending_fixes':len(self.pending),'alignment':self.metrics}
        if not self.p('gnss_enabled'):result['state']='disabled_imu_only'
        (self.output/'bridge_status.json').write_text(json.dumps(result,indent=2))

    def destroy_node(self):
        self.report();self.csv_file.close();return super().destroy_node()

def main():
    rclpy.init();node=GNSSBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
