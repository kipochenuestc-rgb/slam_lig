"""Render baseline and adaptive checkpoints at identical validation cameras."""
import argparse
import json
import math
import pathlib
import hashlib

import cv2
import numpy as np
import torch
from gsplat import rasterization
from refine_local_gs import pose_delta, ssim_loss
from scipy.spatial.transform import Rotation, Slerp


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scene',type=pathlib.Path,required=True)
    ap.add_argument('--baseline',type=pathlib.Path,required=True)
    ap.add_argument('--refined',type=pathlib.Path,required=True)
    ap.add_argument('--output',type=pathlib.Path,required=True)
    ap.add_argument('--animation',action='store_true')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    views=json.loads((args.scene/'cameras.json').read_text())
    base=torch.load(args.baseline,map_location='cuda',weights_only=False)
    new=torch.load(args.refined,map_location='cpu',weights_only=False)
    if new['scene']!=str(args.scene):raise ValueError('Refined checkpoint/scene mismatch')
    baseline_scene=pathlib.Path(base['scene'])
    baseline_views=json.loads((baseline_scene/'cameras.json').read_text())
    if baseline_views!=views:raise ValueError('Baseline and refined camera definitions differ')
    for v in views:
        a=hashlib.sha256((baseline_scene/v['image_path']).read_bytes()).digest()
        b=hashlib.sha256((args.scene/v['image_path']).read_bytes()).digest()
        if a!=b:raise ValueError('Baseline and refined images differ: '+v['image_path'])
    params={k:p.cuda() for k,p in new['params'].items()};pose=new['pose'].cuda()
    hold=[i for i,v in enumerate(views) if int(pathlib.Path(v['image_path']).stem.split('_')[0])%8==4]
    train=[i for i in range(len(views)) if i not in hold]
    scores={'baseline':[],'refined':[]};corrected_poses={}
    with torch.no_grad():
        for i in hold:
            v=views[i];target=cv2.cvtColor(cv2.imread(str(args.scene/v['image_path'])),cv2.COLOR_BGR2RGB)
            h,w=target.shape[:2];K=torch.tensor(v['K'],dtype=torch.float32,device='cuda')[None]
            T=torch.tensor(v['world_to_camera'],dtype=torch.float32,device='cuda')[None]
            a=[j for j in train if views[j]['camera_id']==v['camera_id'] and views[j]['stamp_s']<=v['stamp_s']]
            b=[j for j in train if views[j]['camera_id']==v['camera_id'] and views[j]['stamp_s']>=v['stamp_s']]
            a=max(a,key=lambda j:views[j]['stamp_s']);b=min(b,key=lambda j:views[j]['stamp_s'])
            t=(v['stamp_s']-views[a]['stamp_s'])/(views[b]['stamp_s']-views[a]['stamp_s'])
            delta=(1-t)*pose[a]+t*pose[b]
            c2w=torch.tensor(v['camera_to_world'],dtype=torch.float32,device='cuda')@pose_delta(delta)
            corrected_poses[i]=c2w.cpu().numpy()
            old=rasterization(base['means'],base['quats'],base['log_scales'].exp(),base['opacity_logits'].sigmoid(),
                             base['sh'],T,K,w,h,sh_degree=1,near_plane=.3,far_plane=60,
                             rasterize_mode='antialiased',packed=False,backgrounds=base['background'])[0][0].clamp(0,1)
            fresh=rasterization(params['means'],params['quats'],params['scales'].exp(),params['opacities'].sigmoid(),
                               torch.cat([params['sh0'],params['shN']],1),torch.linalg.inv(c2w)[None],K,w,h,
                               sh_degree=min(new['step']//1500,3),near_plane=.2,far_plane=100,packed=False,
                               backgrounds=torch.tensor([[.87,.90,.945]],device='cuda'))[0][0].clamp(0,1)
            gt=torch.tensor(target,dtype=torch.float32,device='cuda')/255
            for key,pred in [('baseline',old),('refined',fresh)]:
                scores[key].append({'image':v['image_path'],'psnr':-10*math.log10(max(float((pred-gt).square().mean()),1e-12)),
                                    'ssim':1-float(ssim_loss(pred,gt))})
            canvas=np.hstack([target,(old.cpu().numpy()*255).astype('u1'),(fresh.cpu().numpy()*255).astype('u1')])
            header=np.full((36,w*3,3),24,dtype='u1')
            for column,label in enumerate(['ORIGINAL','PREVIOUS 3DGS','REFINED 3DGS']):
                cv2.putText(header,label,(column*w+16,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(240,240,240),1,cv2.LINE_AA)
            canvas=np.vstack([header,canvas])
            cv2.imwrite(str(args.output/f'comparison_{i:04d}.jpg'),cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR))
    summary={key:{metric:float(np.mean([s[metric] for s in row])) for metric in ['psnr','ssim']} for key,row in scores.items()}
    report={'summary':summary,'views':scores,'refined_step':new['step'],'refined_gaussians':len(params['means']),
            'layout':'original | baseline | adaptive; all same temporal validation images','input_images_sha256_verified':True,
            'note':'Validation pose corrections use adjacent training frames only. Initialization colors include full scene.'}
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report['summary']),flush=True)
    if args.animation:
        camera=[i for i,v in enumerate(views) if v['camera_id']==1]
        matrices=[]
        for i in camera:
            if i in corrected_poses:matrices.append(corrected_poses[i])
            else:
                with torch.no_grad():matrices.append((torch.tensor(views[i]['camera_to_world'],dtype=torch.float32,device='cuda')@pose_delta(pose[i])).cpu().numpy())
        matrices=np.array(matrices);times=np.array([views[i]['stamp_s'] for i in camera]);times-=times[0]
        rotations=Slerp(times,Rotation.from_matrix(matrices[:,:3,:3]));frames=[]
        for t in np.linspace(times[0],times[-1],96):
            T=np.eye(4);T[:3,:3]=rotations(t).as_matrix()
            for j in range(3):T[j,3]=np.interp(t,times,matrices[:,j,3])
            T[:3,3]+=T[:3,0]*.1
            with torch.no_grad():
                pred=rasterization(params['means'],params['quats'],params['scales'].exp(),params['opacities'].sigmoid(),
                      torch.cat([params['sh0'],params['shN']],1),torch.tensor(np.linalg.inv(T),dtype=torch.float32,device='cuda')[None],K,w,h,
                      sh_degree=min(new['step']//1500,3),near_plane=.2,far_plane=100,packed=False,
                      backgrounds=torch.tensor([[.87,.90,.945]],device='cuda'))[0][0]
                frames.append((pred.clamp(0,1).cpu().numpy()*255).astype('u1'))
        from PIL import Image
        Image.fromarray(frames[0]).save(args.output/'flythrough.gif',save_all=True,append_images=[Image.fromarray(f) for f in frames[1:]],duration=100,loop=0)


if __name__=='__main__':main()
