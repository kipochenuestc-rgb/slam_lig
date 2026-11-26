"""Package image-only 3DGS inputs: camera priors + image-track BA points, never LiDAR points."""
import argparse,hashlib,json,pathlib,shutil
import cv2,numpy as np
import csv
from rgb_core import write_rgb_ply


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--images-scene',type=pathlib.Path,required=True)
    ap.add_argument('--bundle',type=pathlib.Path,required=True)
    ap.add_argument('--semantics',type=pathlib.Path,required=True)
    ap.add_argument('--output',type=pathlib.Path,required=True)
    args=ap.parse_args();out=args.output
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True);(out/'images').mkdir();(out/'valid_masks').mkdir()
    views=json.loads((args.bundle/'cameras.json').read_text());priors=json.loads((args.images_scene/'cameras.json').read_text())
    if len(views)!=len(priors):raise ValueError('Image/prior count differs')
    for i,(v,p) in enumerate(zip(views,priors)):
        if any(v[k]!=p[k] for k in ['image_path','camera_id','stamp_s','K','width','height']):raise ValueError('Bundle image definition differs')
        src=args.images_scene/v['image_path'];shutil.copy2(src,out/v['image_path'])
        dynamic=cv2.imread(str(args.semantics/'dynamic'/f'{i:04d}.png'),0)
        if dynamic is None or dynamic.shape!=(v['height'],v['width']):raise ValueError('Missing/wrong semantic mask')
        # Keep sky photometry. Only dynamic-image pixels are excluded from loss.
        cv2.imwrite(str(out/'valid_masks'/f'{i:04d}.png'),255-dynamic)
    source=args.bundle/'feature_points.npz';data=np.load(source);xyz=data['xyz'];rgb=data['rgb']
    if len(xyz)<500 or not np.isfinite(xyz).all():raise ValueError('Insufficient finite image tracks')
    np.savez_compressed(out/'points.npz',xyz=xyz.astype('f4'),rgb=rgb.astype('u1'));write_rgb_ply(out/'image_sparse_points.ply',xyz,rgb)
    (out/'cameras.json').write_text(json.dumps(views,indent=2));(out/'slam_camera_priors.json').write_text(json.dumps(priors,indent=2));shutil.copytree(args.bundle/'colmap',out/'sparse')
    with (out/'camera_poses.csv').open('w',newline='') as handle:
        writer=csv.writer(handle);writer.writerow(['image','camera_id','stamp_s','x_m','y_m','z_m']+[f'R{a}{b}' for a in range(3) for b in range(3)])
        for v in views:
            T=np.array(v['camera_to_world']);writer.writerow([v['image_path'],v['camera_id'],format(v['stamp_s'],'.9f'),*T[:3,3],*T[:3,:3].ravel()])
    original=json.loads((args.images_scene/'scene.json').read_text());report=json.loads((args.bundle/'report.json').read_text())
    metadata={'initialization_source':'image_tracks_bundle_adjusted','point_source_file':str(source),'point_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
              'images':len(views),'sets':len(views)//4,'image_track_points':len(xyz),'bundle_report':report,
              'scene_origin_map_m':original['scene_origin_map_m'],'start_s':original['start_s'],'duration_s':original['duration_s'],
              'calibration_dir':original['calibration_dir'],'calibration_status':original['calibration_status'],
              'pose_initialization':'SLAM optimized trajectory at image stamp composed with calibrated camera extrinsics',
              'gnss_role':'Already incorporated in source SLAM trajectory; no GNSS residuals in Gaussian training',
              'lidar_geometry_used':False,'depth_loss_used':False,'training_split':'whole time group id modulo 8 == 4 withheld from feature reconstruction and Gaussian optimization',
              'notes':['No raw/accumulated LiDAR points copied or read by the Gaussian trainer.',
                       'Metric first/last rig poses and fixed intra-rig extrinsics retain SLAM pose prior information.',
                       'RGB losses only, with dynamic-image masks; no depth, plane, surface-anchor or point-to-LiDAR losses.']}
    (out/'scene.json').write_text(json.dumps(metadata,indent=2));print('IMAGE_ONLY_SCENE_READY',json.dumps(metadata),flush=True)


if __name__=='__main__':main()
