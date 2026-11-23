"""Adaptive local 3DGS training with bounded growth and camera pose refinement."""
import argparse
import json
import math
import pathlib
import time

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from scipy.spatial import cKDTree
from gsplat import rasterization
from gsplat.strategy import DefaultStrategy
from train_local_gs import C0, export_gaussians


def ssim_loss(pred, target, valid_mask=None):
    """Gaussian-window SSIM, averaged over valid (unpadded) windows."""
    x = pred.permute(2, 0, 1)[None]
    y = target.permute(2, 0, 1)[None]
    t = torch.arange(11, device=x.device, dtype=x.dtype) - 5
    g = torch.exp(-t.square() / (2 * 1.5**2)); g = g / g.sum()
    kernel = (g[:, None] * g[None, :])[None, None].expand(3, 1, 11, 11)
    pool = lambda z: F.conv2d(z, kernel, groups=3)
    mx, my = pool(x), pool(y)
    vx, vy = pool(x*x) - mx*mx, pool(y*y) - my*my
    cov = pool(x*y) - mx*my
    score = ((2*mx*my + .01**2)*(2*cov + .03**2) /
             ((mx*mx + my*my + .01**2)*(vx + vy + .03**2)))
    if valid_mask is not None:
        valid = F.avg_pool2d(valid_mask[None,None],11,1) > .999
        return ((1-score)*valid).sum() / (valid.sum()*3).clamp_min(1)
    return 1 - score.mean()


class CappedStrategy(DefaultStrategy):
    """Grow the highest-gradient Gaussians while respecting an exact point cap."""
    def _grow_gs(self, params, optimizers, state, step):
        budget=max(0,self.point_cap-len(params['means']))
        grads=state['grad2d']/state['count'].clamp_min(1)
        candidates=torch.where(grads>self.grow_grad2d)[0]
        if len(candidates)>budget:
            keep=candidates[torch.topk(grads[candidates],budget).indices] if budget else candidates[:0]
            restricted=torch.zeros_like(state['grad2d']);restricted[keep]=state['grad2d'][keep]
            state['grad2d']=restricted
        return super()._grow_gs(params,optimizers,state,step)


def person_masks(images, output):
    """Exclude segmented people; keep parked bicycles and static street furniture."""
    from torchvision.models.segmentation import lraspp_mobilenet_v3_large, LRASPP_MobileNet_V3_Large_Weights
    weights=LRASPP_MobileNet_V3_Large_Weights.DEFAULT
    model=lraspp_mobilenet_v3_large(weights=weights).cuda().eval();preprocess=weights.transforms()
    masks=[];folder=output/'person_masks';folder.mkdir()
    with torch.no_grad():
        for i,image in enumerate(images):
            tensor=torch.tensor(image,device='cuda').permute(2,0,1).float()/255
            logits=model(preprocess(tensor)[None])['out']
            labels=F.interpolate(logits,size=image.shape[:2],mode='bilinear',align_corners=False)[0].argmax(0)
            person=(labels==15).cpu().numpy().astype('u1')
            person=cv2.dilate(person,np.ones((9,9),'u1'));valid=1-person
            cv2.imwrite(str(folder/f'{i:04d}.png'),valid*255)
            masks.append(torch.tensor(valid,dtype=torch.float32,device='cuda'))
    del model;torch.cuda.empty_cache();return masks


def append_parameters(params, optimizers, state, additions):
    """Extend all Gaussian tensors, preserving Adam moments for existing points."""
    count=len(additions['means'])
    for key in list(params):
        old=params[key];extra=additions[key].to(old)
        new=torch.nn.Parameter(torch.cat([old.detach(),extra],0));params[key]=new
        optimizer=optimizers[key];old_state=optimizer.state.pop(old,{})
        optimizer.param_groups[0]['params']=[new]
        optimizer.state[new]={k:(torch.cat([value,torch.zeros_like(extra)],0) if torch.is_tensor(value) and value.shape==old.shape else value)
                              for k,value in old_state.items()}
    for key in ['grad2d','count','radii']:
        if state.get(key) is not None:state[key]=torch.cat([state[key],state[key].new_zeros(count)])


def pose_delta(vector):
    """Camera-local translation and axis-angle rotation; identity at zero."""
    translation, omega = vector[..., :3], vector[..., 3:]
    x, y, z = omega.unbind(-1); zero = torch.zeros_like(x)
    skew = torch.stack([zero, -z, y, z, zero, -x, -y, x, zero], -1).reshape(*omega.shape[:-1], 3, 3)
    angle = torch.linalg.vector_norm(omega, dim=-1)[..., None, None]
    eye = torch.eye(3, device=vector.device).expand_as(skew)
    rotation = eye + torch.sinc(angle/math.pi)*skew + .5*torch.sinc(angle/(2*math.pi)).square()*(skew@skew)
    top = torch.cat([rotation, translation[..., None]], -1)
    bottom = torch.zeros(*vector.shape[:-1], 1, 4, device=vector.device); bottom[..., 0, 3] = 1
    return torch.cat([top, bottom], -2)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scene', type=pathlib.Path, required=True)
    ap.add_argument('--output', type=pathlib.Path, required=True)
    ap.add_argument('--steps', type=int, default=18000)
    ap.add_argument('--initial-points', type=int, default=120000)
    ap.add_argument('--point-cap', type=int, default=500000)
    ap.add_argument('--eval-every', type=int, default=1000)
    ap.add_argument('--resume', type=pathlib.Path)
    ap.add_argument('--no-pose-opt', action='store_true')
    ap.add_argument('--mask-people', action='store_true')
    ap.add_argument('--feature-points',type=pathlib.Path)
    ap.add_argument('--mask-dir',type=pathlib.Path)
    ap.add_argument('--export-only',action='store_true',help='Export --resume without more optimization')
    ap.add_argument('--patience',type=int,default=4,help='Stop after this many validation checks without improvement; 0 disables')
    ap.add_argument('--image-only',action='store_true',help='Require image-triangulated initialization and prohibit LiDAR geometry inputs')
    args = ap.parse_args()
    if args.steps < 1 or args.eval_every < 1 or args.initial_points < 8 or args.point_cap < args.initial_points:
        raise ValueError('Invalid training/growth settings')
    if args.export_only and not args.resume:raise ValueError('--export-only requires --resume')
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.mkdir(parents=True); (args.output/'renders').mkdir()
    torch.manual_seed(42); rng = np.random.default_rng(42)
    views = json.loads((args.scene/'cameras.json').read_text())
    metadata=json.loads((args.scene/'scene.json').read_text())
    if args.image_only:
        if metadata.get('initialization_source')!='image_tracks_bundle_adjusted':raise ValueError('Image-only mode requires a verified image-track scene')
        if args.resume or args.feature_points:raise ValueError('Image-only run starts from its own image tracks; external checkpoints/points prohibited')
        if not args.no_pose_opt:raise ValueError('Use --no-pose-opt to preserve the jointly optimized rigid camera rig')
    images = []
    for v in views:
        im = cv2.imread(str(args.scene/v['image_path']))
        if im is None: raise ValueError(v['image_path'])
        images.append(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
    h, w = images[0].shape[:2]
    if any(im.shape != (h,w,3) for im in images): raise ValueError('Image size mismatch')
    hold = [i for i,v in enumerate(views) if int(pathlib.Path(v['image_path']).stem.split('_')[0])%8 == 4]
    train = [i for i in range(len(views)) if i not in hold]
    if not hold or not train: raise ValueError('Need a temporal validation split')
    c2w = torch.tensor(np.array([v['camera_to_world'] for v in views]), dtype=torch.float32, device='cuda')
    Ks = torch.tensor(np.array([v['K'] for v in views]), dtype=torch.float32, device='cuda')
    scene_scale = float(torch.linalg.vector_norm(c2w[:,:3,3]-c2w[:,:3,3].mean(0),dim=-1).max().item())
    scene_scale = max(scene_scale, 1.)
    data = np.load(args.scene/'points.npz'); xyz = data['xyz']; rgb = data['rgb']/255.
    sel = rng.choice(len(xyz), min(args.initial_points,len(xyz)), replace=False); xyz, rgb = xyz[sel], rgb[sel]
    dist, _ = cKDTree(xyz).query(xyz, k=4, workers=4)
    scales = np.sqrt((dist[:,1:]**2).mean(1)).clip(.005,2. if args.image_only else .2)
    initial = {'means':xyz, 'scales':np.log(scales)[:,None].repeat(3,1),
               'quats':np.tile([1.,0,0,0],(len(xyz),1)),
               'opacities':np.full(len(xyz),math.log(.1/.9)),
               'sh0':((rgb-.5)/C0)[:,None,:], 'shN':np.zeros((len(xyz),15,3))}
    params = torch.nn.ParameterDict({k:torch.nn.Parameter(torch.tensor(v,dtype=torch.float32,device='cuda')) for k,v in initial.items()})
    del data, xyz, rgb, initial
    rates = {'means':1.6e-4*scene_scale,'scales':.005,'quats':.001,'opacities':.05,'sh0':.0025,'shN':.000125}
    optimizers = {k:torch.optim.Adam([p],lr=rates[k],eps=1e-15) for k,p in params.items()}
    pose = torch.nn.Parameter(torch.zeros(len(views),6,device='cuda'))
    pose_optimizer = torch.optim.Adam([pose], lr=1e-5)
    # Validation poses interpolate corrections from adjacent TRAINING images only.
    interpolation = {}
    for i in hold:
        candidates = [j for j in train if views[j]['camera_id']==views[i]['camera_id']]
        t = views[i]['stamp_s']; before=[j for j in candidates if views[j]['stamp_s']<=t]; after=[j for j in candidates if views[j]['stamp_s']>=t]
        a = max(before,key=lambda j:views[j]['stamp_s']) if before else min(candidates,key=lambda j:views[j]['stamp_s'])
        b = min(after,key=lambda j:views[j]['stamp_s']) if after else max(candidates,key=lambda j:views[j]['stamp_s'])
        f = 0 if a==b else (t-views[a]['stamp_s'])/(views[b]['stamp_s']-views[a]['stamp_s'])
        interpolation[i] = (a,b,f)
    strategy = CappedStrategy(absgrad=True,grow_grad2d=.0008,grow_scale3d=.003,
                               refine_stop_iter=min(12000,args.steps*2//3),reset_every=3000,
                               pause_refine_after_reset=len(train),verbose=True)
    strategy.point_cap=args.point_cap
    strategy.check_sanity(params,optimizers); state = strategy.initialize_state(scene_scale)
    start_step = 0
    added_features=0
    if args.resume:
        ckpt = torch.load(args.resume,map_location='cuda',weights_only=False)
        if ckpt['scene'] != str(args.scene): raise ValueError('Resume scene differs')
        for k in params:
            params[k] = torch.nn.Parameter(ckpt['params'][k].cuda())
            optimizers[k] = torch.optim.Adam([params[k]],lr=rates[k],eps=1e-15)
            optimizers[k].load_state_dict(ckpt['optimizers'][k])
        with torch.no_grad(): pose.copy_(ckpt['pose'].cuda())
        pose_optimizer.load_state_dict(ckpt['pose_optimizer']);state=ckpt['strategy_state'];start_step=ckpt['step']
        added_features=ckpt.get('added_features',0)
        if len(params['means'])>args.point_cap:raise ValueError('Resumed checkpoint exceeds --point-cap')
    if args.export_only:args.steps=start_step
    elif args.steps<=start_step:raise ValueError('--steps must exceed the resumed iteration')
    if args.feature_points and not added_features:
        feature=np.load(args.feature_points);xyz=feature['xyz'];rgb=feature['rgb']/255.
        # Keep only genuinely new surface support, avoiding duplicates of trained Gaussians.
        tree=cKDTree(params['means'].detach().cpu().numpy());distance=tree.query(xyz,workers=4)[0]
        xyz,rgb=xyz[distance>.08],rgb[distance>.08]
        budget=min(max(args.point_cap-len(params['means']),0),40000)
        selected=rng.choice(len(xyz),min(len(xyz),budget),replace=False);xyz,rgb=xyz[selected],rgb[selected]
        if len(xyz):
            scale=cKDTree(np.concatenate([params['means'].detach().cpu().numpy(),xyz])).query(xyz,k=4,workers=4)[0][:,1:].mean(1).clip(.01,.2)
            additions={'means':xyz,'scales':np.log(scale)[:,None].repeat(3,1),'quats':np.tile([1.,0,0,0],(len(xyz),1)),
                       'opacities':np.full(len(xyz),math.log(.1/.9)),'sh0':((rgb-.5)/C0)[:,None], 'shN':np.zeros((len(xyz),15,3))}
            append_parameters(params,optimizers,state,{k:torch.tensor(v,dtype=torch.float32) for k,v in additions.items()})
            added_features=len(xyz);print('ADDED_IMAGE_FEATURE_POINTS',added_features,flush=True)
    bg = torch.tensor([[.87,.90,.945]],device='cuda')
    targets = [torch.tensor(im,dtype=torch.float32,device='cuda')/255 for im in images]
    if args.mask_dir:
        masks=[]
        for i in range(len(views)):
            mask=cv2.imread(str(args.mask_dir/f'{i:04d}.png'),0)
            if mask is None or mask.shape!=(h,w):raise ValueError('Missing/wrong-sized person mask')
            masks.append(torch.tensor(mask>0,dtype=torch.float32,device='cuda'))
    else:masks=person_masks(images,args.output) if args.mask_people else [None]*len(views)
    active_degree = min(start_step//1500,3)
    def view_matrix(i, corrected=True):
        correction = pose[i]
        if i in interpolation:
            a,b,f=interpolation[i];correction=(1-f)*pose[a]+f*pose[b]
        if args.no_pose_opt or not corrected:correction=correction*0
        return torch.linalg.inv(c2w[i]@pose_delta(correction))
    def render(i, corrected=True):
        return rasterization(params['means'],params['quats'],params['scales'].exp(),
               params['opacities'].sigmoid(),torch.cat([params['sh0'],params['shN']],1),
               view_matrix(i,corrected)[None],Ks[i:i+1],w,h,sh_degree=active_degree,
               near_plane=.2,far_plane=150 if args.image_only else 100,packed=False,absgrad=True,backgrounds=bg)
    def evaluate(tag):
        psnrs=[];ssims=[]
        with torch.no_grad():
            for i in hold:
                pred=render(i)[0][0].clamp(0,1);target=targets[i]
                psnrs.append(-10*math.log10(max(float((pred-target).square().mean()),1e-12)))
                ssims.append(1-float(ssim_loss(pred,target)))
                if tag in ['initial','best']:
                    pred8=(pred.cpu().numpy()*255).astype('u1')
                    cv2.imwrite(str(args.output/'renders'/f'{tag}_{i:04d}.jpg'),cv2.cvtColor(np.hstack([images[i],pred8]),cv2.COLOR_RGB2BGR))
        result={'step':tag,'psnr':float(np.mean(psnrs)),'ssim':float(np.mean(ssims)),'gaussians':len(params['means'])}
        print('EVAL',json.dumps(result),flush=True);return result
    def save(path,step):
        temporary=path.with_suffix('.tmp')
        torch.save({'params':{k:p.detach().cpu() for k,p in params.items()},'pose':pose.detach().cpu(),
                    'optimizers':{k:o.state_dict() for k,o in optimizers.items()},'pose_optimizer':pose_optimizer.state_dict(),
                    'strategy_state':state,'step':step,'scene':str(args.scene),'added_features':added_features,
                    'geometry_source':'image_tracks_only' if args.image_only else 'lidar_initialized','background':bg.cpu()},temporary)
        temporary.replace(path)
    start=time.monotonic();initial=evaluate('initial');best=initial;history=[];order=[];stale=0;last_step=start_step
    if args.resume:
        best={**initial,'step':start_step};save(args.output/'best.pt',start_step)
    for step in range(start_step+1,args.steps+1):
        last_step=step
        active_degree=min(step//1500,3)
        if not order:order=list(rng.permutation(train))
        i=int(order.pop());pred,alpha,info=render(i);pred=pred[0]
        strategy.step_pre_backward(params,optimizers,state,step,info)
        if masks[i] is None:l1=(pred-targets[i]).abs().mean()
        else:l1=((pred-targets[i]).abs()*masks[i][...,None]).sum()/(masks[i].sum()*3).clamp_min(1)
        loss=.8*l1+.2*ssim_loss(pred,targets[i],masks[i])
        if not args.no_pose_opt:
            loss=loss+.01*pose[:,:3].square().mean()+.1*pose[:,3:].square().mean()
        if not torch.isfinite(loss):raise RuntimeError('Non-finite loss')
        loss.backward()
        for optimizer in optimizers.values():optimizer.step();optimizer.zero_grad(set_to_none=True)
        if step>1000 and not args.no_pose_opt:pose_optimizer.step()
        pose_optimizer.zero_grad(set_to_none=True)
        with torch.no_grad():
            params['scales'].clamp_(math.log(.001),math.log(2. if args.image_only else 1.))
            params['opacities'].clamp_(-12,12)
            pose[:,:3].clamp_(-.15,.15);pose[:,3:].clamp_(-.035,.035)
        strategy.step_post_backward(params,optimizers,state,step,info)
        optimizers['means'].param_groups[0]['lr']=rates['means']*(.01**(step/args.steps))
        if step%100==0:
            row={'step':step,'loss':float(loss.detach()),'l1':float(l1.detach()),'gaussians':len(params['means']),'elapsed_s':time.monotonic()-start}
            history.append(row);(args.output/'progress.json').write_text(json.dumps(row));print(json.dumps(row),flush=True)
        if step%args.eval_every==0 or step==args.steps:
            score=evaluate(step);history.append(score)
            if score['psnr']>best['psnr']:
                best=score;save(args.output/'best.pt',step);stale=0
            else:stale+=1
            (args.output/'history.json').write_text(json.dumps(history))
            if args.patience>0 and stale>=args.patience:
                print('EARLY_STOP',step,'best iteration',best['step'],flush=True);break
    if not (args.output/'best.pt').exists():raise RuntimeError('No validation improvement')
    ckpt=torch.load(args.output/'best.pt',map_location='cuda',weights_only=False)
    for k in params:params[k]=torch.nn.Parameter(ckpt['params'][k].cuda())
    with torch.no_grad():pose.copy_(ckpt['pose'].cuda())
    active_degree=min(ckpt['step']//1500,3);final=evaluate('best')
    export_gaussians(args.output/'gaussians.ply',params['means'],params['quats'],params['scales'],params['opacities'],torch.cat([params['sh0'],params['shN']],1))
    (args.output/'scene_metadata.json').write_text((args.scene/'scene.json').read_text())
    corrected=[]
    with torch.no_grad():
        for i,v in enumerate(views):corrected.append({**v,'refined_camera_to_world':torch.linalg.inv(view_matrix(i)).cpu().tolist()})
    (args.output/'refined_cameras.json').write_text(json.dumps(corrected))
    report={'initial':initial,'best':best,'final':final,'steps':last_step,'requested_steps':args.steps,
            'mode':'export_only' if args.export_only else 'training','scene':str(args.scene),
            'pose_optimization':not args.no_pose_opt,'person_mask':args.mask_people or bool(args.mask_dir),
            'added_image_feature_points':added_features,'peak_allocated_mb':torch.cuda.max_memory_allocated()/2**20,
            'geometry_source':'image_tracks_only' if args.image_only else 'lidar_initialized',
            'lidar_geometry_used':not args.image_only,'depth_loss_used':False,
            'notes':['Same exported images and temporal validation split as baseline.',
                     'Validation pose corrections interpolate adjacent training poses, never optimize validation images.',
                     'Image-only initialization uses training image tracks only; other modes may use colors from the complete scene. Metrics select a validation checkpoint.',
                     'Dynamic-image masks only affect training; reported validation metrics are unmasked. No explicit sky geometry.',
                     'Image-only mode fixes the jointly bundle-adjusted camera rig. Other modes may optimize individual camera corrections; neither proves calibration accuracy.']}
    (args.output/'training_report.json').write_text(json.dumps(report,indent=2))
    (args.output/'history.json').write_text(json.dumps(history));print('READY',json.dumps(report),flush=True)


if __name__=='__main__':main()
