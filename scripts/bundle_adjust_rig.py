"""Static-feature bundle adjustment with a shared four-camera rig and fixed metric endpoints."""
import argparse,json,pathlib
import cv2,numpy as np,pycolmap as pc
from scipy.spatial.transform import Rotation,Slerp


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--scene',type=pathlib.Path,required=True);ap.add_argument('--semantics',type=pathlib.Path,required=True);ap.add_argument('--output',type=pathlib.Path,required=True)
    args=ap.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    args.output.mkdir(parents=True);views=json.loads((args.scene/'cameras.json').read_text())
    train=[i for i,v in enumerate(views) if int(pathlib.Path(v['image_path']).stem.split('_')[0])%8!=4]
    features={};images={};sift=cv2.SIFT_create(nfeatures=6000)
    for i in train:
        image=cv2.imread(str(args.scene/views[i]['image_path']));mask=cv2.imread(str(args.semantics/'static'/f'{i:04d}.png'),0)
        if image is None or mask is None:raise ValueError('Missing image/mask')
        features[i]=sift.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),mask);images[i]=image[:,:,::-1]
    print('FEATURES',sum(len(k) for k,d in features.values()),flush=True)
    groups={cid:[i for i in train if views[i]['camera_id']==cid] for cid in range(4)}
    by_set={}
    for i in train:by_set.setdefault(int(pathlib.Path(views[i]['image_path']).stem.split('_')[0]),{})[views[i]['camera_id']]=i
    pairs=set()
    for indices in groups.values():
        for stride in [1,2,4,8]:pairs.update(zip(indices[:-stride],indices[stride:]))
    for group in by_set.values():
        for a,b in [(0,1),(1,2),(2,3),(3,0)]:pairs.add((group[a],group[b]))
    parent={};members={};nodes={}
    def root(x):
        if x not in parent:parent[x]=x;members[x]={x[0]};nodes[x]=[x]
        while parent[x]!=x:parent[x]=parent[parent[x]];x=parent[x]
        return x
    def union(a,b):
        a,b=root(a),root(b)
        if a==b or members[a]&members[b]:return
        if len(members[a])<len(members[b]):a,b=b,a
        parent[b]=a;members[a]|=members.pop(b);nodes[a].extend(nodes.pop(b))
    matcher=cv2.FlannBasedMatcher(dict(algorithm=1,trees=5),dict(checks=64))
    for number,(a,b) in enumerate(sorted(pairs)):
        ka,da=features[a];kb,db=features[b]
        if da is None or db is None:continue
        matches=matcher.knnMatch(da,db,k=2);good=[m for pair in matches if len(pair)==2 for m,n in [pair] if m.distance<.7*n.distance]
        if len(good)<12:continue
        x=np.float64([ka[m.queryIdx].pt for m in good]);y=np.float64([kb[m.trainIdx].pt for m in good]);_,valid=cv2.findFundamentalMat(x,y,cv2.USAC_MAGSAC,1.,.999)
        if valid is None:continue
        K=np.array(views[a]['K']);T=np.array(views[b]['world_to_camera'])@np.array(views[a]['camera_to_world']);tx,ty,tz=T[:3,3];cross=np.array([[0,-tz,ty],[tz,0,-tx],[-ty,tx,0]]);F=np.linalg.inv(K).T@cross@T[:3,:3]@np.linalg.inv(K)
        line=np.column_stack([x,np.ones(len(x))])@F.T;error=abs((np.column_stack([y,np.ones(len(y))])*line).sum(1))/np.maximum(np.linalg.norm(line[:,:2],axis=1),1e-12)
        for match,keep,e in zip(good,valid.ravel(),error):
            if keep and e<3:union((a,match.queryIdx),(b,match.trainIdx))
        if number%100==0:print('MATCHED_PAIRS',number,'/',len(pairs),flush=True)
    tracks=[v for v in nodes.values() if len(v)>=4];points=[];accepted=[];colors=[]
    for track in tracks:
        positions=np.array([np.array(views[i]['camera_to_world'])[:3,3] for i,k in track]);a,b=np.unravel_index(np.linalg.norm(positions[:,None]-positions[None],axis=2).argmax(),(len(track),len(track)))
        if np.linalg.norm(positions[a]-positions[b])<.2:continue
        projections=[np.array(views[i]['K'])@np.array(views[i]['world_to_camera'])[:3] for i,k in track];xy=np.array([features[i][0][k].pt for i,k in track])
        homogeneous=cv2.triangulatePoints(projections[a],projections[b],xy[a:a+1].T,xy[b:b+1].T).ravel();X=homogeneous[:3]/homogeneous[3]
        if not np.isfinite(X).all():continue
        ray=X-positions;length=np.linalg.norm(ray,axis=1);angle=np.degrees(np.arccos(np.clip(np.dot(ray[a],ray[b])/(length[a]*length[b]),-1,1)))
        q=np.array([P@np.r_[X,1] for P in projections]);residual=np.linalg.norm(q[:,:2]/q[:,2:]-xy,axis=1);keep=(q[:,2]>.5)&(residual<2.)&(length<150)
        if keep.sum()<4 or angle<2:continue
        track=[t for t,k in zip(track,keep) if k];points.append(X);accepted.append(track)
        rgb=[]
        for i,k in track:
            px,py=np.rint(features[i][0][k].pt).astype(int);rgb.append(images[i][py,px])
        colors.append(np.median(rgb,axis=0).astype('u1'))
    if len(points)<500:raise RuntimeError('Insufficient consistent multi-view tracks')
    rec=pc.Reconstruction();first=next(iter(by_set.values()));reference=np.array(views[first[0]]['camera_to_world'])
    rig=pc.Rig(rig_id=1);rig.add_ref_sensor(pc.sensor_t(pc.SensorType.CAMERA,1))
    for cid in range(4):
        v=views[first[cid]];K=np.array(v['K']);rec.add_camera(pc.Camera(camera_id=cid+1,model='PINHOLE',width=v['width'],height=v['height'],params=[K[0,0],K[1,1],K[0,2],K[1,2]]))
        if cid:rig.add_sensor(pc.sensor_t(pc.SensorType.CAMERA,cid+1),pc.Rigid3d((np.array(v['world_to_camera'])@reference)[:3]))
    rec.add_rig(rig)
    for set_id,group in by_set.items():
        frame=pc.Frame(frame_id=set_id+1,rig_id=1,rig_from_world=pc.Rigid3d(np.array(views[group[0]]['world_to_camera'])[:3]))
        for cid,i in group.items():frame.add_data_id(pc.data_t(pc.sensor_t(pc.SensorType.CAMERA,cid+1),i+1))
        rec.add_frame(frame)
        for cid,i in group.items():
            image=pc.Image(name=views[i]['image_path'],keypoints=np.float64([k.pt for k in features[i][0]]),camera_id=cid+1,image_id=i+1);image.frame_id=set_id+1;rec.add_image(image)
        rec.register_frame(set_id+1)
    for X,track,rgb in zip(points,accepted,colors):rec.add_point3D(X,pc.Track([pc.TrackElement(i+1,k) for i,k in track]),rgb)
    rec.update_point_3d_errors();before=rec.compute_mean_reprojection_error();print('BEFORE_BA',rec.summary(),flush=True)
    config=pc.BundleAdjustmentConfig()
    for i in train:config.add_image(i+1)
    config.set_constant_rig_from_world_pose(min(by_set)+1);config.set_constant_rig_from_world_pose(max(by_set)+1)
    options=pc.BundleAdjustmentOptions();options.refine_focal_length=False;options.refine_principal_point=False;options.refine_extra_params=False;options.refine_sensor_from_rig=False;options.ceres.loss_function_type=pc.LossFunctionType.HUBER;options.ceres.loss_function_scale=1.;options.ceres.solver_options.max_num_iterations=300;options.ceres.solver_options.function_tolerance=1e-6;options.ceres.solver_options.parameter_tolerance=1e-8;options.ceres.solver_options.num_threads=8
    solver=pc.create_default_bundle_adjuster(options,config,rec);result=solver.solve();rec.update_point_3d_errors();after=rec.compute_mean_reprojection_error()
    (args.output/'colmap').mkdir();rec.write(str(args.output/'colmap'))
    frame_ids=sorted(by_set);times=np.array([views[by_set[s][0]]['stamp_s'] for s in frame_ids]);original=np.array([np.array(views[by_set[s][0]]['camera_to_world']) for s in frame_ids]);adjusted=np.array([rec.images[by_set[s][0]+1].cam_from_world().inverse().matrix() for s in frame_ids]);fixed=np.tile(np.eye(4),(len(adjusted),1,1));fixed[:,:3]=adjusted
    output=[]
    for i,v in enumerate(views):
        t=v['stamp_s']
        # Apply the shared rig correction in WORLD coordinates to each camera.
        index=np.searchsorted(times,t).clip(0,len(times)-1)
        if t in times:
            correction=fixed[index]@np.linalg.inv(original[index])
        else:
            k=np.searchsorted(times,t);fraction=(t-times[k-1])/(times[k]-times[k-1]);world_a=fixed[k-1]@np.linalg.inv(original[k-1]);world_b=fixed[k]@np.linalg.inv(original[k]);correction=np.eye(4);correction[:3,:3]=Slerp([0,1],Rotation.from_matrix([world_a[:3,:3],world_b[:3,:3]]))(fraction).as_matrix();correction[:3,3]=(1-fraction)*world_a[:3,3]+fraction*world_b[:3,3]
        T=correction@np.array(v['camera_to_world']);output.append({**v,'camera_to_world':T.tolist(),'world_to_camera':np.linalg.inv(T).tolist()})
    valid=[p for p in rec.points3D.values() if p.error<1. and p.track.length()>=4]
    np.savez_compressed(args.output/'feature_points.npz',xyz=np.array([p.xyz for p in valid],dtype='f4'),rgb=np.array([p.color for p in valid],dtype='u1'))
    (args.output/'cameras.json').write_text(json.dumps(output));report={'tracks_before':len(points),'tracks_after':len(valid),'reprojection_before_px':before,'reprojection_after_px':after,'max_center_adjustment_m':float(np.linalg.norm(fixed[:,:3,3]-original[:,:3,3],axis=1).max()),'point_source':'training_image_feature_tracks','lidar_geometry_used':False,'notes':['Static training images only; four-camera extrinsics fixed; first/last training frame fixed to preserve metric gauge.','Validation rig corrections interpolate adjacent training rig corrections.']};(args.output/'report.json').write_text(json.dumps(report,indent=2));print('READY',json.dumps(report),flush=True)


if __name__=='__main__':main()
