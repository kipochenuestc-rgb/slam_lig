"""Add image-supported geometry outside the LiDAR coverage, using training views only."""
import argparse
import json
import pathlib
import cv2
import numpy as np
import torch
from scipy.spatial import cKDTree
from refine_local_gs import pose_delta
from rgb_core import voxel_rgb


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scene',type=pathlib.Path,required=True)
    ap.add_argument('--checkpoint',type=pathlib.Path,required=True)
    ap.add_argument('--masks',type=pathlib.Path,required=True)
    ap.add_argument('--output',type=pathlib.Path,required=True)
    args=ap.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    views=json.loads((args.scene/'cameras.json').read_text())
    ckpt=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    if ckpt['scene']!=str(args.scene):raise ValueError('Scene/checkpoint mismatch')
    poses=ckpt['pose'];sift=cv2.SIFT_create(nfeatures=5000)
    features={};images={};matrices={};groups={i:[] for i in range(4)}
    for i,v in enumerate(views):
        if int(pathlib.Path(v['image_path']).stem.split('_')[0])%8==4:continue
        image=cv2.imread(str(args.scene/v['image_path']));mask=cv2.imread(str(args.masks/f'{i:04d}.png'),0)
        if image is None or mask is None:raise ValueError('Missing image/mask')
        features[i]=sift.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),mask)
        images[i]=image[:,:,::-1]
        with torch.no_grad():matrices[i]=(torch.tensor(v['camera_to_world'],dtype=torch.float32)@pose_delta(poses[i])).numpy().astype(float)
        groups[v['camera_id']].append(i)
    print('FEATURES',sum(len(k) for k,d in features.values()),flush=True)
    matcher=cv2.FlannBasedMatcher(dict(algorithm=1,trees=5),dict(checks=64));points=[];colors=[];errors=[];pair_count=0
    for cid,indices in groups.items():
        for stride in [1,2,4,8]:
            for a,b in zip(indices[:-stride],indices[stride:]):
                ka,da=features[a];kb,db=features[b]
                if da is None or db is None:continue
                matches=matcher.knnMatch(da,db,k=2);good=[m for pair in matches if len(pair)==2 for m,n in [pair] if m.distance<.7*n.distance]
                if len(good)<12:continue
                pa=np.float64([ka[m.queryIdx].pt for m in good]);pb=np.float64([kb[m.trainIdx].pt for m in good])
                _,inliers=cv2.findFundamentalMat(pa,pb,cv2.USAC_MAGSAC,1.,.999)
                if inliers is None:continue
                pa,pb=pa[inliers.ravel()>0],pb[inliers.ravel()>0]
                K=np.array(views[a]['K']);A=np.linalg.inv(matrices[a]);B=np.linalg.inv(matrices[b])
                if np.linalg.norm(matrices[a][:3,3]-matrices[b][:3,3])<.1:continue
                X=cv2.triangulatePoints(K@A[:3],K@B[:3],pa.T,pb.T).T
                X=X[:,:3]/X[:,3:4];xa=X@A[:3,:3].T+A[:3,3];xb=X@B[:3,:3].T+B[:3,3]
                ua=xa@K.T;ub=xb@K.T;ua=ua[:,:2]/ua[:,2:];ub=ub[:,:2]/ub[:,2:]
                error=np.maximum(np.linalg.norm(ua-pa,axis=1),np.linalg.norm(ub-pb,axis=1))
                ra=X-matrices[a][:3,3];rb=X-matrices[b][:3,3];cos=(ra*rb).sum(1)/(np.linalg.norm(ra,axis=1)*np.linalg.norm(rb,axis=1))
                valid=np.isfinite(X).all(1)&(xa[:,2]>.5)&(xb[:,2]>.5)&(error<1.2)&(cos<np.cos(np.deg2rad(1.)))&(np.linalg.norm(ra,axis=1)<150)&(np.linalg.norm(rb,axis=1)<150)
                if not valid.any():continue
                X,pa,pb=X[valid],pa[valid],pb[valid];ia=np.rint(pa).astype(int);ib=np.rint(pb).astype(int)
                rgb=(images[a][ia[:,1],ia[:,0]].astype(float)+images[b][ib[:,1],ib[:,0]].astype(float))/2
                points.append(X);colors.append(rgb);errors.extend(error[valid]);pair_count+=1
        print('TRIANGULATED_CAMERA',cid,'points',sum(len(p) for p in points),flush=True)
    if not points:raise RuntimeError('No triangulated features')
    xyz,rgb=voxel_rgb(np.concatenate(points),np.concatenate(colors),.05)
    measured=np.load(args.scene/'points.npz')['xyz'];distance=cKDTree(measured).query(xyz,workers=4)[0]
    xyz,rgb=xyz[distance>.1],rgb[distance>.1]
    np.savez_compressed(args.output,xyz=xyz,rgb=rgb)
    report={'points':len(xyz),'accepted_observations':len(errors),'median_reprojection_px':float(np.median(errors)),
            'max_reprojection_px':float(np.max(errors)),'pairs':pair_count,'camera_checkpoint_step':ckpt['step'],
            'note':'Image-triangulated initialization only; not LiDAR measurements. Training images only; people masks applied.'}
    args.output.with_suffix('.json').write_text(json.dumps(report,indent=2));print('READY',json.dumps(report),flush=True)


if __name__=='__main__':main()
