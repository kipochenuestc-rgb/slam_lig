import unittest
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField
from fusion_core import FrameMatcher,fuse_messages,DTYPE,stamp_ns

def cloud(ns,xyz=(2,0,1),micros=123):
    dt=np.dtype({'names':['x','y','z','intensity','timestamp'],'formats':['<f4','<f4','<f4','u1','<u4'],'offsets':[0,4,8,12,16],'itemsize':20});a=np.zeros(1,dt)
    for k,v in zip(['x','y','z'],xyz):a[k]=v
    a['intensity']=7;a['timestamp']=micros;m=PointCloud2();m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9;m.header.frame_id='hasco_lidar';m.height=1;m.width=1;m.point_step=20;m.row_step=20;m.data=a.tobytes();m.fields=[PointField(name=k,offset=o,datatype=t,count=1) for k,o,t in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,2),('timestamp',16,6)]];return m

class FusionTests(unittest.TestCase):
    def test_extrinsics_and_per_point_time(self):
        t=10**10;messages=[cloud(t+i*1_000_000) for i in range(4)];transforms=[np.eye(4) for _ in range(4)];transforms[1][:3,3]=[1,2,3]
        message,info=fuse_messages(messages,transforms);a=np.frombuffer(message.data,DTYPE);second=a[a['lidar_id']==2][0]
        self.assertEqual(len(message.data),message.row_step*message.height)
        from rclpy.serialization import serialize_message,deserialize_message
        restored=deserialize_message(serialize_message(message),PointCloud2)
        np.testing.assert_array_equal(np.frombuffer(restored.data,DTYPE)['source_time_us'],a['source_time_us'])
        np.testing.assert_allclose([second['x'],second['y'],second['z']],[3,2,4]);self.assertAlmostEqual(float(second['time']),.001123,places=8);self.assertEqual(stamp_ns(message),t);self.assertEqual(int(second['source_time_us']),123);self.assertEqual(message.width,4)
    def test_drop_incomplete_instead_of_cross_cycle_match(self):
        t=10**10;m=FrameMatcher();groups=[]
        for i in range(4):groups+=m.push(i,cloud(t+(100_000_000 if i==2 else i*1_000_000)))
        groups+=m.drain(flush=True);self.assertEqual(groups,[]);self.assertEqual(m.stats['dropped_incomplete_reference_frames'],1)
    def test_complete_sets_are_not_reused(self):
        t=10**10;m=FrameMatcher();groups=[]
        for cycle in range(3):
            for i in [2,0,3,1]:groups+=m.push(i,cloud(t+cycle*100_000_000+i*1_000_000))
        groups+=m.drain(flush=True);self.assertEqual(len(groups),3);self.assertEqual(len({stamp_ns(g[2]) for g in groups}),3)
    def test_nonmonotonic_input_is_rejected(self):
        m=FrameMatcher();m.push(1,cloud(10**10));m.push(1,cloud(10**10));self.assertEqual(m.stats['rejected_nonmonotonic_2'],1)
if __name__=='__main__':unittest.main()
