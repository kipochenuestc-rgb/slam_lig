"""Export synchronized measured RGB points and rectified multi-camera 3DGS input."""
import argparse,bisect,json,pathlib
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation
from rgb_core import load_rig,decode_mosaic,split_cameras,cloud_arrays,transform,Poses,fish_project,visible_pixels,voxel_rgb,write_rgb_ply


def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag',type=pathlib.Path,required=True);ap.add_argument('--run',type=pathlib.Path,required=True);ap.add_argument('--output',type=pathlib.Path,required=True)
    ap.add_argument('--start',type=float,default=275);ap.add_argument('--duration',type=float,default=20);ap.add_argument('--sample-period',type=float,default=.5)
    ap.add_argument('--max-range',type=float,default=35);ap.add_argument('--voxel',type=float,default=.1);ap.add_argument('--width',type=int,default=640);ap.add_argument('--height',type=int,default=512);ap.add_argument('--horizontal-fov',type=float,default=110)
    args=ap.parse_args();out=args.output
    if out.exists():raise FileExistsError('Use a new output directory: '+str(out))
    out.mkdir(parents=True);(out/'images').mkdir();(out/'projection').mkdir();sparse=out/'sparse/0';sparse.mkdir(parents=True)
    cal_dir=args.bag.parent/'calibration';cameras,lidars,quadrants,status=load_rig(cal_dir);poses=Poses(args.run/'map/transformations.pcd')
    metadata=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];t0=metadata['starting_time']['nanoseconds_since_epoch']*1e-9;lo=t0+args.start;hi=lo+args.duration
    if hi>poses.end-.12:raise ValueError('Scene extends beyond optimized moving trajectory')
    origin=poses.at((lo+hi)/2)[:3,3];collections={'/Data_Cam_Global':[]};collections.update({f'/point_cloud_hasco{i}':[] for i in range(1,5)})
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    files=[v for v in metadata['files'] if v['starting_time']['nanoseconds_since_epoch']*1e-9+v['duration']['nanoseconds']*1e-9>=lo-1 and v['starting_time']['nanoseconds_since_epoch']*1e-9<=hi+1]
    for record in files:
        reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(args.bag/record['path']),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
        types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()};reader.set_filter(rosbag2_py.StorageFilter(topics=list(collections)))
        while reader.has_next():
            topic,buf,_=reader.read_next();m=deserialize_message(buf,types[topic]);s=stamp(m)
            if lo-.03<=s<=hi+.03:collections[topic].append((s,m))
        print('READ',record['path'],flush=True)
    for values in collections.values():values.sort(key=lambda v:v[0])
    clouds={i:collections[f'/point_cloud_hasco{i}'] for i in range(1,5)};times={i:[v[0] for v in a] for i,a in clouds.items()};images=collections['/Data_Cam_Global']
    f=args.width/(2*np.tan(np.radians(args.horizontal_fov)/2));Knew=np.array([[f,0,args.width/2],[0,f,args.height/2],[0,0,1.]])
    rect={cid:cv2.fisheye.initUndistortRectifyMap(cam['K'],cam['D'],np.eye(3),Knew,(args.width,args.height),cv2.CV_32FC1) for cid,cam in cameras.items()}
    world_points=[];colors=[];views=[];sync=[];counts={str(i):{'input':0,'colored':0} for i in range(1,5)};next_time=lo;sets=0
    for image_time,message in images:
        if image_time<next_time-.025 or image_time>hi:continue
        group={}
        for i in range(1,5):
            k=bisect.bisect_left(times[i],image_time);candidates=clouds[i][max(0,k-1):k+1]
            if not candidates:break
            candidate=min(candidates,key=lambda v:abs(v[0]-image_time))
            if abs(candidate[0]-image_time)>.02:break
            group[i]=candidate
        if len(group)!=4:continue
        tiles=split_cameras(decode_mosaic(message),quadrants,cameras);reference_pose=poses.at(image_time);camera_poses={cid:reference_pose@v['reference_from_camera'] for cid,v in cameras.items()}
        for cid,tile in tiles.items():
            name=f'{sets:05d}_cam{cid}.jpg';image=cv2.remap(tile,*rect[cid],cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
            cv2.imwrite(str(out/'images'/name),image,[cv2.IMWRITE_JPEG_QUALITY,95]);c2w=camera_poses[cid].copy();c2w[:3,3]-=origin
            views.append({'image_path':'images/'+name,'camera_id':cid,'stamp_s':image_time,'camera_to_world':c2w.tolist(),'world_to_camera':np.linalg.inv(c2w).tolist(),'K':Knew.tolist(),'width':args.width,'height':args.height})
        for i,(cloud_time,cloud) in group.items():
            xyz,relative=cloud_arrays(cloud);radius=np.linalg.norm(xyz,axis=1);good=np.isfinite(xyz).all(1)&np.isfinite(relative)&(radius>=1.5)&(radius<=args.max_range)&(relative>=0)&(relative<=.2)
            xyz,relative=xyz[good],relative[good];counts[str(i)]['input']+=len(xyz);local=transform(lidars[i]['reference_from_lidar'],xyz);world=poses.points_to_world(local,cloud_time+relative)
            cid=lidars[i]['camera_id'];cam=cameras[cid];camera_points=transform(np.linalg.inv(camera_poses[cid]),world);uv,valid=fish_project(camera_points,cam['K'],cam['D'],cam['size']);idx=visible_pixels(camera_points,uv,valid,cam['size'])
            pix=np.rint(uv[idx]).astype(int);rgb=tiles[cid][pix[:,1],pix[:,0],::-1];nonblack=rgb.max(1)>12;idx=idx[nonblack];rgb=rgb[nonblack];counts[str(i)]['colored']+=len(idx)
            p,c=voxel_rgb(world[idx],rgb,args.voxel);world_points.append(p);colors.append(c);sync.append((cloud_time-image_time)*1000)
            if sets==0:
                overlay=tiles[cid].copy();selection=idx[::max(1,len(idx)//4500)];color=cv2.applyColorMap(np.clip(np.linalg.norm(camera_points[selection],axis=1)/args.max_range*255,0,255).astype(np.uint8),cv2.COLORMAP_TURBO).reshape(-1,3)
                for pt,col in zip(np.rint(uv[selection]).astype(int),color):cv2.circle(overlay,tuple(pt),1,tuple(int(v) for v in col),-1)
                cv2.imwrite(str(out/'projection'/f'lidar{i}_camera{cid}.jpg'),np.hstack([tiles[cid],overlay]))
        sets+=1;next_time=image_time+args.sample_period
        if sets%10==0:print('COLORED',sets,'sets',flush=True)
    if sets<3:raise ValueError('Insufficient synchronized image/cloud sets')
    xyz,rgb=voxel_rgb(np.concatenate(world_points),np.concatenate(colors),args.voxel);write_rgb_ply(out/'rgb_map.ply',xyz,rgb);np.savez_compressed(out/'points.npz',xyz=(xyz-origin).astype(np.float32),rgb=rgb)
    (out/'cameras.json').write_text(json.dumps(views,indent=2));summary={'sets':sets,'images':len(views),'colored_points':len(xyz),'per_lidar':counts,'start_s':args.start,'duration_s':args.duration,'scene_origin_map_m':origin.tolist(),'camera_convention':'OpenCV x right, y down, z forward; camera_to_world','point_format':'rgb_map.ply in SLAM map; points.npz and cameras.json in centered local map','calibration_dir':str(cal_dir),'calibration_status':status,'pairing':{str(i):v['camera_id'] for i,v in lidars.items()},'image_storage':'NV12 1600x1920 bytes -> BGR 1600x1280 -> four 800x640 tiles','projection':'KB4 fisheye; native K,D retained; training images rectified to PINHOLE','motion_compensation':'Each measured point uses header + per-point microseconds, interpolated optimized SLAM pose; camera uses image header','pose_source':str(args.run),'camera_lidar_offset_ms_min_max':[min(sync),max(sync)],'native_intrinsics':{str(cid):{'K':v['K'].tolist(),'D':v['D'].tolist(),'size':v['size'],'reference_from_camera':v['reference_from_camera'].tolist()} for cid,v in cameras.items()}}
    (out/'scene.json').write_text(json.dumps(summary,indent=2))
    flip=np.diag([1.,-1.,-1.,1.]);nerf={'camera_model':'PINHOLE','fl_x':f,'fl_y':f,'cx':args.width/2,'cy':args.height/2,'w':args.width,'h':args.height,'frames':[{'file_path':v['image_path'],'transform_matrix':(np.array(v['camera_to_world'])@flip).tolist()} for v in views]};(out/'transforms.json').write_text(json.dumps(nerf,indent=2))
    with (sparse/'cameras.txt').open('w') as handle:
        for cid in cameras:handle.write(f'{cid+1} PINHOLE {args.width} {args.height} {f} {f} {args.width/2} {args.height/2}\n')
    with (sparse/'images.txt').open('w') as handle:
        for iid,v in enumerate(views,1):
            w2c=np.array(v['world_to_camera']);q=Rotation.from_matrix(w2c[:3,:3]).as_quat()[[3,0,1,2]];handle.write(str(iid)+' '+' '.join(map(str,[*q,*w2c[:3,3],v['camera_id']+1]))+' '+pathlib.Path(v['image_path']).name+'\n\n')
    # Text COLMAP initialization is deliberately track-free; SLAM provides poses.
    step=max(1,len(xyz)//100000)
    with (sparse/'points3D.txt').open('w') as handle:
        for pid,k in enumerate(range(0,len(xyz),step),1):handle.write(str(pid)+' '+' '.join(map(str,[*(xyz[k]-origin),*rgb[k],0.]))+'\n')
    print('RGB_SCENE_READY',json.dumps(summary),flush=True)


if __name__=='__main__':main()
