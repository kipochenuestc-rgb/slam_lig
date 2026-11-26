"""Audit and conservatively prune Gaussian floaters against measured-map depth."""
import argparse
import copy
import json
import math
import pathlib

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from scipy.spatial import cKDTree
from gsplat import rasterization
from refine_local_gs import pose_delta, ssim_loss
from train_local_gs import export_gaussians


def cleanup_masks(observed, free, foreground, person, near, distance, scale, opacity):
    """Conservative evidence masks; nearby static geometry alone is not removed."""
    free_bad=(observed>=3)&(free/np.maximum(observed,1)>.65)
    person_bad=(person>=2)&(person/np.maximum(foreground,1)>=.5)
    near_person=(near<.35)&(person>=1)
    unsupported_large=(observed<3)&(distance>.75+.01*near)&(scale>.15)
    keep_depth=~free_bad&~person_bad&(opacity>=.015)
    keep_balanced=keep_depth&~near_person&~unsupported_large
    angular=scale/np.maximum(near,.01)
    keep_strict=keep_depth&(distance<=.2+.015*near)&~((angular>.06)&(distance>.15))&(opacity>=.03)
    return {'original':np.ones(len(near),dtype=bool),'depth':keep_depth,'balanced':keep_balanced,'strict':keep_strict},{
        'free_space_conflicts':int(free_bad.sum()),'person_candidates':int(person_bad.sum()),
        'near_camera_person_candidates':int(near_person.sum()),'unsupported_large_candidates':int(unsupported_large.sum())}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--checkpoint',type=pathlib.Path,required=True)
    ap.add_argument('--masks',type=pathlib.Path,required=True)
    ap.add_argument('--output',type=pathlib.Path,required=True)
    ap.add_argument('--mode',choices=['depth','balanced','strict'],default='balanced')
    args=ap.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    args.output.mkdir(parents=True);(args.output/'renders').mkdir()
    checkpoint=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    scene=pathlib.Path(checkpoint['scene']);views=json.loads((scene/'cameras.json').read_text())
    params={k:p.cuda() for k,p in checkpoint['params'].items()};pose=checkpoint['pose'].cuda()
    n=len(params['means']);measured=np.load(scene/'points.npz')['xyz']
    xyz=params['means'].cpu().numpy();distance=cKDTree(measured).query(xyz,workers=4)[0]
    photo_file=scene/'feature_points.npz'
    if photo_file.exists():distance=np.minimum(distance,cKDTree(np.load(photo_file)['xyz']).query(xyz,workers=4)[0])
    centers=np.array([v['camera_to_world'] for v in views])[:,:3,3]
    near=cKDTree(centers).query(xyz,workers=4)[0]
    opacity=params['opacities'].sigmoid().cpu().numpy();scale=params['scales'].exp().amax(-1).cpu().numpy()
    groups={cid:[i for i,v in enumerate(views) if v['camera_id']==cid and int(pathlib.Path(v['image_path']).stem.split('_')[0])%8!=4] for cid in range(4)}
    hold=[i for i,v in enumerate(views) if int(pathlib.Path(v['image_path']).stem.split('_')[0])%8==4]
    def matrix(i):
        delta=pose[i]
        if i in hold:
            candidates=groups[views[i]['camera_id']];t=views[i]['stamp_s']
            a=max((j for j in candidates if views[j]['stamp_s']<=t),key=lambda j:views[j]['stamp_s'])
            b=min((j for j in candidates if views[j]['stamp_s']>=t),key=lambda j:views[j]['stamp_s'])
            f=(t-views[a]['stamp_s'])/(views[b]['stamp_s']-views[a]['stamp_s']);delta=(1-f)*pose[a]+f*pose[b]
        return torch.linalg.inv(torch.tensor(views[i]['camera_to_world'],dtype=torch.float32,device='cuda')@pose_delta(delta))
    observed=torch.zeros(n,dtype=torch.int32,device='cuda');free=torch.zeros_like(observed)
    foreground=torch.zeros_like(observed);person=torch.zeros_like(observed)
    cloud=torch.tensor(measured,dtype=torch.float32,device='cuda')
    with torch.no_grad():
        for cid,indices in groups.items():
            for i in indices[::3]:
                v=views[i];T=matrix(i);K=torch.tensor(v['K'],dtype=torch.float32,device='cuda');h,w=v['height']//2,v['width']//2
                def project(points):
                    p=points@T[:3,:3].T+T[:3,3];z=p[:,2];uv=p[:,:2]/z[:,None].clamp_min(.001)
                    uv=uv*torch.stack([K[0,0],K[1,1]])/2+K[:2,2]/2
                    uv=uv.round().long();good=(z>.2)&(z<100)&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
                    return uv,z,good
                uv,z,good=project(cloud);depth=torch.full((h*w,),float('inf'),device='cuda')
                depth.scatter_reduce_(0,uv[good,1]*w+uv[good,0],z[good],reduce='amin',include_self=True)
                # Three-pixel minimum fills small sampling holes conservatively.
                depth=-F.max_pool2d(-depth.reshape(1,1,h,w),3,1,1)[0,0]
                mask=cv2.imread(str(args.masks/f'{i:04d}.png'),0)
                if mask is None:raise ValueError('Missing person mask')
                mask=torch.tensor(cv2.resize(mask,(w,h),interpolation=cv2.INTER_NEAREST)>0,device='cuda')
                uv,z,good=project(params['means']);ids=torch.where(good)[0];pixels=uv[ids];reference=depth[pixels[:,1],pixels[:,0]]
                reliable=torch.isfinite(reference);static=mask[pixels[:,1],pixels[:,0]]
                tested=ids[reliable&static];observed[tested]+=1
                violation=reliable&static&(z[ids]+.25+.02*reference<reference)
                free[ids[violation]]+=1
                # Only near visible surfaces can receive a semantic person vote.
                surface=reliable&(z[ids]<=reference+.2+.01*reference)&(z[ids]<8)
                foreground[ids[surface]]+=1;person[ids[surface&~static]]+=1
            print('AUDITED_CAMERA',cid,flush=True)
    observed=observed.cpu().numpy();free=free.cpu().numpy();foreground=foreground.cpu().numpy();person=person.cpu().numpy()
    modes,counts=cleanup_masks(observed,free,foreground,person,near,distance,scale,opacity)
    report={'input_gaussians':n,**counts,
            'retained':{k:int(v.sum()) for k,v in modes.items()},'mode':args.mode,'validation':{},
            'notes':['Depth is from the accumulated SLAM cloud, not independent ground truth.',
                     'Only training camera views and person masks used to classify Gaussians.',
                     'Removal is a diagnostic cleanup, not proof of sensor calibration accuracy.',
                     'Strict mode can remove valid sparse surfaces; compare geometry and images before use.']}
    np.savez_compressed(args.output/'audit.npz',observed=observed,free=free,person=person,foreground=foreground,
                        nearest_support=distance,nearest_camera=near,max_scale=scale,opacity=opacity,
                        keep_depth=modes['depth'],keep_balanced=modes['balanced'],keep_strict=modes['strict'])
    bg=torch.tensor([[.87,.90,.945]],device='cuda');degree=min(checkpoint['step']//1500,3)
    with torch.no_grad():
        for mode,keep in modes.items():
            sel=torch.tensor(keep,device='cuda');p={k:v[sel] for k,v in params.items()};metrics=[]
            for i in hold:
                v=views[i];K=torch.tensor(v['K'],dtype=torch.float32,device='cuda')[None]
                pred=rasterization(p['means'],p['quats'],p['scales'].exp(),p['opacities'].sigmoid(),torch.cat([p['sh0'],p['shN']],1),
                     matrix(i)[None],K,v['width'],v['height'],sh_degree=degree,near_plane=.2,far_plane=100,packed=False,backgrounds=bg)[0][0].clamp(0,1)
                target=cv2.cvtColor(cv2.imread(str(scene/v['image_path'])),cv2.COLOR_BGR2RGB)
                gt=torch.tensor(target,dtype=torch.float32,device='cuda')/255
                metrics.append({'psnr':-10*math.log10(max(float((pred-gt).square().mean()),1e-12)),'ssim':1-float(ssim_loss(pred,gt))})
                rgb=(pred.cpu().numpy()*255).astype('u1');cv2.imwrite(str(args.output/'renders'/f'{mode}_{i:04d}.jpg'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            report['validation'][mode]={k:float(np.mean([m[k] for m in metrics])) for k in ['psnr','ssim']}
    keep=modes[args.mode];selected=torch.tensor(keep);clean=copy.deepcopy(checkpoint)
    clean['params']={k:v[selected].clone() for k,v in checkpoint['params'].items()}
    for optimizer in clean['optimizers'].values():
        for state in optimizer['state'].values():
            for k,value in list(state.items()):
                if torch.is_tensor(value) and value.ndim and value.shape[0]==n:state[k]=value[selected].clone()
    for k in ['grad2d','count','radii']:
        if clean['strategy_state'].get(k) is not None:clean['strategy_state'][k]=clean['strategy_state'][k][selected].clone()
    clean['postprocess']=report;torch.save(clean,args.output/'best.pt');p=clean['params']
    export_gaussians(args.output/'gaussians.ply',p['means'],p['quats'],p['scales'],p['opacities'],torch.cat([p['sh0'],p['shN']],1))
    (args.output/'scene_metadata.json').write_text((scene/'scene.json').read_text())
    (args.output/'audit_report.json').write_text(json.dumps(report,indent=2));print('READY',json.dumps(report),flush=True)


if __name__=='__main__':main()
