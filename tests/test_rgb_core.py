import pathlib,tempfile,types,unittest
import numpy as np
from rgb_core import decode_mosaic,split_cameras,fish_project,transform,Poses,voxel_rgb


class RGBGeometryTests(unittest.TestCase):
    def test_nv12_mosaic_uses_decoded_height_and_calibration_quadrants(self):
        y=np.block([[np.full((8,8),32),np.full((8,8),64)],[np.full((8,8),128),np.full((8,8),200)]]).astype('u1')
        nv=np.vstack([y,np.full((8,16),128,dtype='u1')]);m=types.SimpleNamespace(data=nv.tobytes(),height=24,width=16,step=16,encoding='nv12')
        bgr=decode_mosaic(m);self.assertEqual(bgr.shape,(16,16,3))
        tiles=split_cameras(bgr,{'tl':3,'tr':1,'bl':2,'br':0},{i:{'size':(8,8)} for i in range(4)})
        self.assertLess(tiles[3].mean(),tiles[1].mean());self.assertLess(tiles[1].mean(),tiles[2].mean());self.assertLess(tiles[2].mean(),tiles[0].mean())

    def test_fisheye_center_and_behind_camera_rejection(self):
        K=np.array([[200.,0,400],[0,200,320],[0,0,1.]])
        uv,valid=fish_project(np.array([[0.,0,10],[1,0,10],[0,0,-10]]),K,np.zeros(4),(800,640))
        np.testing.assert_allclose(uv[0],[400,320]);self.assertGreater(uv[1,0],400);self.assertEqual(valid.tolist(),[True,True,False])

    def test_pose_interpolation_and_transform_direction(self):
        dtype=[(k,'<f4') for k in ['x','y','z','roll','pitch','yaw']]+[('time','<f8')];p=np.zeros(2,dtype=dtype);p['x']=[0,1];p['yaw']=[0,np.pi/2];p['time']=[10,11]
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/'poses.pcd'
            with path.open('wb') as f:f.write(b'FIELDS x y z roll pitch yaw time\nSIZE 4 4 4 4 4 4 8\nTYPE F F F F F F F\nCOUNT 1 1 1 1 1 1 1\nPOINTS 2\nDATA binary\n');f.write(p.tobytes())
            poses=Poses(path);T=poses.at(10.5);np.testing.assert_allclose(T[:3,3],[.5,0,0]);np.testing.assert_allclose(T[:2,0],[np.sqrt(.5),np.sqrt(.5)],atol=1e-6)
            xyz=np.array([[1.,2,3]]);np.testing.assert_allclose(transform(np.linalg.inv(T),transform(T,xyz)),xyz,atol=1e-6)
            points=poses.points_to_world(np.array([[1.,0,0],[1.,0,0]]),np.array([10.,11.]));np.testing.assert_allclose(points,[[1,0,0],[1,1,0]],atol=1e-6)

    def test_rgb_voxel_averages_measured_colors(self):
        p,c=voxel_rgb(np.array([[.01,0,0],[.02,0,0],[1.,0,0]]),np.array([[255,0,0],[0,0,255],[0,255,0]],dtype='u1'),.1)
        self.assertEqual(len(p),2);np.testing.assert_array_equal(c[0],[128,0,128]);np.testing.assert_array_equal(c[1],[0,255,0])
