import unittest,numpy as np
from fusion_core import DTYPE
from filter_core import filter_points

class FilterTests(unittest.TestCase):
    def test_voxel_representative_preserves_real_sample_and_time(self):
        p=np.zeros(4,DTYPE);p['x']=[2.01,2.03,2.09,4];p['y']=[.01,.01,.01,0];p['time']=[.001,.002,.003,.004];p['lidar_id']=[1,2,3,4];p['source_time_us']=[100,200,300,400]
        out,info=filter_points(p,leaf_size=.2,mean_k=0)
        self.assertEqual(len(out),2)
        # 2.03 is closest to the first voxel's centroid: keep its actual metadata.
        self.assertAlmostEqual(float(out[0]['x']),2.03,places=5);self.assertAlmostEqual(float(out[0]['time']),.002,places=7);self.assertEqual(int(out[0]['lidar_id']),2);self.assertEqual(int(out[0]['source_time_us']),200);self.assertEqual(out.dtype,DTYPE)
    def test_finite_and_distance_filter(self):
        p=np.zeros(4,DTYPE);p['x']=[.5,3,101,np.nan];out,info=filter_points(p,leaf_size=0,mean_k=0);self.assertEqual(len(out),1);self.assertEqual(float(out[0]['x']),3)
    def test_statistical_isolated_point_is_removed(self):
        p=np.zeros(101,DTYPE);rng=np.random.default_rng(4);p['x'][:100]=3+rng.normal(0,.05,100);p['y'][:100]=rng.normal(0,.05,100);p['x'][100]=50;p['lidar_id']=1
        out,info=filter_points(p,leaf_size=0,mean_k=10,stddev_mul=2.);self.assertLess(len(out),len(p));self.assertLess(float(out['x'].max()),4)
if __name__=='__main__':unittest.main()
