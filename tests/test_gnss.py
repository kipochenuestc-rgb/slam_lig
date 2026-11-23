import math,os,pathlib,tempfile,time,unittest
import numpy as np
from gnss_core import LocalENU,ecef,fit_alignment,fix_covariance

class CoordinateTests(unittest.TestCase):
    def test_wgs84_origin_and_east(self):
        np.testing.assert_allclose(ecef(0,0,0),[6378137,0,0])
        frame=LocalENU(0,0,10);np.testing.assert_allclose(frame.convert(0,0,10),[0,0,0])
        expected=(6378137+10)*math.radians(.00001)
        self.assertAlmostEqual(frame.convert(.00001,0,10)[0],expected,places=6)

    def test_alignment_preserves_scale_and_recovers_pose(self):
        p=np.column_stack([np.linspace(0,40,20),np.sin(np.linspace(0,3,20))*5,np.linspace(0,1,20)])
        angle=.8;r=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
        t=np.array([7,-4,2]);q=p@r.T+t
        rotation,translation,stats=fit_alignment(p,q)
        np.testing.assert_allclose(rotation,r,atol=1e-10);np.testing.assert_allclose(translation,t,atol=1e-10)
        self.assertLess(stats['horizontal_rmse_m'],1e-10)

    def test_stationary_or_inconsistent_positions_do_not_align(self):
        p=np.zeros((20,3));self.assertIsNone(fit_alignment(p,p))
        p[:,0]=np.linspace(0,30,20);self.assertIsNone(fit_alignment(p,p*2,max_rmse=.1))

    def test_unknown_or_invalid_covariance_is_not_zero(self):
        for values,known in [([0]*9,False),([-1,0,0,0,1,0,0,0,1],True)]:
            np.testing.assert_allclose(fix_covariance(values,known,1,1.5),np.diag([1,1,2.25]))
        c=np.diag([4,9,16]);np.testing.assert_allclose(fix_covariance(c.ravel(),True,1,1.5),c)

class LiveTopicTests(unittest.TestCase):
    def test_disabled_gnss_keeps_imu_without_gps_subscriptions_or_publisher(self):
        import rclpy
        from rclpy.parameter import Parameter
        from sensor_msgs.msg import Imu,NavSatFix
        from gnss_bridge import GNSSBridge
        previous=os.environ.get('SLAM_RUN_DIR');rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as directory:
                os.environ['SLAM_RUN_DIR']=directory
                bridge=GNSSBridge(parameter_overrides=[Parameter('gnss_enabled',value=False),Parameter('apply_imu_bias',value=False)])
                try:
                    self.assertIsNone(bridge.gps_pub)
                    topics=[name for name,_ in bridge.get_subscriber_names_and_types_by_node(bridge.get_name(),bridge.get_namespace())]
                    self.assertIn('/imu/data',topics);self.assertNotIn('/gnss/fix_fixed',topics)
                    bridge.gnss(NavSatFix());self.assertEqual(bridge.count['fix_received'],0)
                    bridge.imu(Imu());self.assertEqual(bridge.count['imu_published'],1)
                    bridge.report();self.assertIn('disabled_imu_only',(pathlib.Path(directory)/'bridge_status.json').read_text())
                    self.assertFalse((pathlib.Path(directory)/'gnss_alignment.json').exists())
                finally:bridge.destroy_node()
        finally:
            rclpy.shutdown()
            if previous is None:os.environ.pop('SLAM_RUN_DIR',None)
            else:os.environ['SLAM_RUN_DIR']=previous

    def test_live_topics_without_clock_preserve_time_and_rotate_covariance(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import NavSatFix,Imu
        from nav_msgs.msg import Odometry
        from gnss_bridge import GNSSBridge
        previous=os.environ.get('SLAM_RUN_DIR');rclpy.init()
        with tempfile.TemporaryDirectory() as directory:
            os.environ['SLAM_RUN_DIR']=directory
            bridge=GNSSBridge(parameter_overrides=[Parameter('apply_imu_bias',value=False),Parameter('min_alignment_points',value=6),Parameter('min_alignment_baseline_m',value=10.)])
            source=rclpy.create_node('test_live_gnss_source');executor=SingleThreadedExecutor();executor.add_node(bridge);executor.add_node(source)
            gps=source.create_publisher(NavSatFix,'/gnss/fix_fixed',20);odom=source.create_publisher(Odometry,'/lio_sam/mapping/odometry',20);imu=source.create_publisher(Imu,'/imu/data',20)
            received=[];imu_received=[]
            gs=source.create_subscription(Odometry,'/lio_gnss/odometry',received.append,20);ims=source.create_subscription(Imu,'/imu/calibrated',imu_received.append,qos_profile_sensor_data)
            def until(predicate,timeout=8):
                end=time.monotonic()+timeout
                while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.03)
                self.assertTrue(predicate())
            try:
                until(lambda:gps.get_subscription_count() and odom.get_subscription_count() and imu.get_subscription_count())
                self.assertFalse(any(info.node_name==bridge.get_name() and info.node_namespace==bridge.get_namespace() for info in bridge.get_subscriptions_info_by_topic('/clock')))
                local=LocalENU(103,30,100);angle=.7;r=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]]);t=np.array([4,-2,.8])
                for i in range(8):
                    fix=NavSatFix();fix.header.stamp.sec=100+i;fix.header.stamp.nanosec=123456789;fix.header.frame_id='antenna';fix.status.status=0;fix.longitude=103+math.degrees(i*4/(6378137*math.cos(math.radians(30))));fix.latitude=30.;fix.altitude=100.;fix.position_covariance_type=3;fix.position_covariance=np.diag([4.,9.,16.]).ravel().tolist()
                    gps.publish(fix);until(lambda:bridge.count['fix_received']>=i+1)
                    position=r@local.convert(fix.longitude,fix.latitude,fix.altitude)+t;m=Odometry();m.header.stamp=fix.header.stamp;m.header.frame_id='map';m.pose.pose.orientation.w=1.;m.pose.pose.position.x=float(position[0]);m.pose.pose.position.y=float(position[1]);m.pose.pose.position.z=float(position[2]);odom.publish(m)
                    until(lambda:bridge.rotation is not None or len(bridge.odometry)>=i+1)
                until(lambda:len(received)==2)
                self.assertEqual([(m.header.stamp.sec,m.header.stamp.nanosec) for m in received],[(106,123456789),(107,123456789)])
                np.testing.assert_allclose(bridge.rotation,r,atol=1e-8);np.testing.assert_allclose(bridge.translation,t,atol=1e-8)
                covariance=np.array(received[-1].pose.covariance).reshape(6,6)[:3,:3];np.testing.assert_allclose(covariance,r@np.diag([4,9,16])@r.T,atol=1e-8)
                np.testing.assert_allclose([received[-1].pose.pose.position.x,received[-1].pose.pose.position.y,received[-1].pose.pose.position.z],position,atol=1e-8)
                fix.header.stamp.sec=108;fix.position_covariance_type=0;fix.position_covariance=[0.]*9;gps.publish(fix);until(lambda:len(received)==3)
                self.assertAlmostEqual(received[-1].pose.covariance[0],1.);self.assertAlmostEqual(received[-1].pose.covariance[14],2.25)
                fix.header.stamp.sec=109;fix.status.status=-1;gps.publish(fix);until(lambda:bridge.count['invalid_fix']==1);self.assertEqual(len(received),3)
                m=Imu();m.header.stamp.sec=105;m.header.stamp.nanosec=987;m.orientation.w=1.;m.angular_velocity.x=.12;m.linear_acceleration.z=9.81;imu.publish(m);until(lambda:len(imu_received)==1)
                self.assertEqual(imu_received[0].header.stamp,m.header.stamp);self.assertAlmostEqual(imu_received[0].angular_velocity.x,.12)
                self.assertTrue((pathlib.Path(directory)/'gnss_alignment.json').is_file())
            finally:
                executor.remove_node(bridge);executor.remove_node(source);bridge.destroy_node();source.destroy_node();executor.shutdown();rclpy.shutdown()
                if previous is None:os.environ.pop('SLAM_RUN_DIR',None)
                else:os.environ['SLAM_RUN_DIR']=previous
