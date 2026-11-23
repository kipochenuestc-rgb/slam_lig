"""Check that the image-only package uses image tracks and preserves the calibrated camera rig."""
import importlib.util,json,pathlib,subprocess,sys,tempfile,unittest
import cv2,numpy as np


class ImageOnlyPackageTests(unittest.TestCase):
    def test_packaging_copies_image_tracks_instead_of_measured_cloud(self):
        with tempfile.TemporaryDirectory() as folder:
            root=pathlib.Path(folder);source=root/'source';bundle=root/'bundle';masks=root/'semantics';output=root/'output'
            for path in [source/'images',bundle/'colmap',masks/'dynamic']:path.mkdir(parents=True)
            view={'image_path':'images/00000_cam0.jpg','camera_id':0,'stamp_s':100.,'width':32,'height':24,'K':[[10,0,16],[0,10,12],[0,0,1]],'camera_to_world':np.eye(4).tolist(),'world_to_camera':np.eye(4).tolist()}
            cv2.imwrite(str(source/view['image_path']),np.full((24,32,3),100,dtype='u1'))
            cv2.imwrite(str(masks/'dynamic/0000.png'),np.zeros((24,32),dtype='u1'))
            (source/'cameras.json').write_text(json.dumps([view]));(bundle/'cameras.json').write_text(json.dumps([view]));(bundle/'report.json').write_text('{}')
            (source/'scene.json').write_text(json.dumps({'scene_origin_map_m':[1,2,3],'start_s':0,'duration_s':2,'calibration_dir':'candidate','calibration_status':'candidate'}))
            # Deliberately invalid source geometry: a correct image-only package never opens it.
            (source/'points.npz').write_bytes(b'NOT_A_POINT_CLOUD')
            xyz=np.random.default_rng(2).normal(size=(501,3)).astype('f4');rgb=np.full((501,3),120,dtype='u1')
            np.savez(bundle/'feature_points.npz',xyz=xyz,rgb=rgb)
            script=pathlib.Path(__file__).resolve().parents[1]/'scripts/prepare_image_only_scene.py'
            subprocess.run([sys.executable,str(script),'--images-scene',str(source),'--bundle',str(bundle),'--semantics',str(masks),'--output',str(output)],check=True,stdout=subprocess.PIPE)
            actual=np.load(output/'points.npz');np.testing.assert_array_equal(actual['xyz'],xyz)
            metadata=json.loads((output/'scene.json').read_text());self.assertFalse(metadata['lidar_geometry_used']);self.assertFalse(metadata['depth_loss_used'])
            self.assertEqual(metadata['initialization_source'],'image_tracks_bundle_adjusted')
            self.assertEqual(len((output/'camera_poses.csv').read_text().splitlines()),2)
            self.assertTrue((cv2.imread(str(output/'valid_masks/0000.png'),0)==255).all())

    @unittest.skipUnless(all(importlib.util.find_spec(k) is not None for k in ['torch','transformers']),'Optional Torch/Transformers environment')
    def test_semantic_merge_keeps_dark_traffic_light_misclassified_by_old_person_mask(self):
        from prepare_scene_semantics import semantic_masks
        labels=np.full((40,60),2,dtype='u1');rgb=np.full((40,60,3),220,dtype='u1');old=np.full((40,60),255,dtype='u1')
        rgb[5:12,5:12]=20;old[5:12,5:12]=0
        labels[20:32,30:40]=12;rgb[20:32,30:40]=70;old[20:32,30:40]=0
        static,sky,dynamic=semantic_masks(labels,rgb,[12,20,80,83,102],old)
        self.assertEqual(static[8,8],1);self.assertEqual(dynamic[8,8],0);self.assertEqual(sky[8,8],0)
        self.assertEqual(dynamic[25,35],1);self.assertEqual(static[25,35],0)


if __name__=='__main__':unittest.main()
