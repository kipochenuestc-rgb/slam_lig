"""Small, LiDAR-initialized 3D Gaussian Splatting validation using gsplat."""
import argparse,json,math,pathlib,time
import cv2,numpy as np
import torch
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation,Slerp
from gsplat import rasterization

C0=0.28209479177387814


def export_gaussians(path,means,quats,log_scales,opacity,sh):
    # Standard Graphdeco PLY layout, with lower SH degrees padded to SH3.
    xyz=means.detach().cpu().numpy();q=torch.nn.functional.normalize(quats,dim=-1).detach().cpu().numpy();s=log_scales.detach().cpu().numpy();o=opacity.detach().cpu().numpy();coeff=sh.detach().cpu().numpy();rest=np.zeros((len(xyz),15,3),np.float32);rest[:,:coeff.shape[1]-1]=coeff[:,1:]
    fields=['x','y','z','nx','ny','nz']+[f'f_dc_{i}' for i in range(3)]+[f'f_rest_{i}' for i in range(45)]+['opacity']+[f'scale_{i}' for i in range(3)]+[f'rot_{i}' for i in range(4)]
    a=np.empty(len(xyz),dtype=[(k,'<f4') for k in fields]);values=np.column_stack([xyz,np.zeros_like(xyz),coeff[:,0],rest.transpose(0,2,1).reshape(len(xyz),-1),o,s,q])
    for j,k in enumerate(fields):a[k]=values[:,j]
    with path.open('wb') as f:f.write(('ply\nformat binary_little_endian 1.0\ncomment trained LiDAR-initialized 3DGS; SH3 field layout\nelement vertex '+str(len(xyz))+'\n'+''.join('property float '+k+'\n' for k in fields)+'end_header\n').encode());f.write(a.tobytes())


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--scene',type=pathlib.Path,required=True);ap.add_argument('--output',type=pathlib.Path,required=True);ap.add_argument('--steps',type=int,default=8000);ap.add_argument('--max-points',type=int,default=160000);ap.add_argument('--seed',type=int,default=42);ap.add_argument('--eval-every',type=int,default=500)
    args=ap.parse_args();out=args.output
    if args.steps<1 or args.max_points<8 or args.eval_every<1:raise ValueError('steps/eval-every must be positive and max-points >= 8')
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True);(out/'renders').mkdir();np.random.seed(args.seed);torch.manual_seed(args.seed)
    if not torch.cuda.is_available():raise RuntimeError('CUDA GPU is required')
    views=json.loads((args.scene/'cameras.json').read_text());data=np.load(args.scene/'points.npz');xyz=data['xyz'].astype(np.float32);rgb=data['rgb'].astype(np.float32)/255
    selection=np.random.choice(len(xyz),min(args.max_points,len(xyz)),replace=False);xyz,rgb=xyz[selection],rgb[selection]
    if not views:raise ValueError('Scene contains no cameras')
    images=[];w,h=views[0]['width'],views[0]['height']
    for v in views:
        image=cv2.imread(str(args.scene/v['image_path']))
        if image is None or image.shape!=(h,w,3):raise ValueError('Invalid training image: '+v['image_path'])
        images.append(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
    # Exclude points invisible in every rectified training view.
    seen=np.zeros(len(xyz),np.int32)
    for v in views:
        T=np.array(v['world_to_camera']);p=xyz@T[:3,:3].T+T[:3,3];K=np.array(v['K']);uv=p[:,:2]/np.maximum(p[:,2:3],1e-6);uv=uv*np.array([K[0,0],K[1,1]])+np.array([K[0,2],K[1,2]])
        seen+=(p[:,2]>.3)&(p[:,2]<60)&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    xyz,rgb=xyz[seen>=3],rgb[seen>=3]
    if len(xyz)<8:raise ValueError('Too few visible points for covariance initialization')
    tree=cKDTree(xyz);dist,neighbors=tree.query(xyz,k=8,workers=4);patch=xyz[neighbors];centered=patch-patch.mean(1,keepdims=True);cov=np.einsum('nki,nkj->nij',centered,centered)/8;vals,vec=np.linalg.eigh(cov);R=vec[:,:,[2,1,0]];R[np.linalg.det(R)<0,:,0]*=-1;quats=Rotation.from_matrix(R).as_quat()[:,[3,0,1,2]].astype('f4')
    tangent=np.maximum(dist[:,1:4].mean(1)*.85,.035).clip(.035,.3);scales=np.column_stack([tangent,tangent,np.maximum(tangent*.25,.012)]).astype('f4')
    device='cuda';means=torch.nn.Parameter(torch.tensor(xyz,device=device));q=torch.nn.Parameter(torch.tensor(quats,device=device));logs=torch.nn.Parameter(torch.tensor(np.log(scales),device=device));opacity=torch.nn.Parameter(torch.zeros(len(xyz),device=device));coeff=np.zeros((len(xyz),4,3),np.float32);coeff[:,0]=(rgb-.5)/C0;sh=torch.nn.Parameter(torch.tensor(coeff,device=device));initial_means=means.detach().clone()
    bg=np.median(np.concatenate([im[:20].reshape(-1,3) for im in images],axis=0),axis=0)/255;bg=torch.tensor(bg,dtype=torch.float32,device=device)[None]
    w2cs=torch.tensor(np.array([v['world_to_camera'] for v in views]),dtype=torch.float32,device=device);Ks=torch.tensor(np.array([v['K'] for v in views]),dtype=torch.float32,device=device)
    heldout=[i for i,v in enumerate(views) if int(pathlib.Path(v['image_path']).stem.split('_')[0])%8==4];train=[i for i in range(len(views)) if i not in heldout]
    if not heldout or not train:raise ValueError('Need several image time groups for a held-out split')
    optimizer=torch.optim.Adam([{'params':[means],'lr':.00006},{'params':[q],'lr':.0001},{'params':[logs],'lr':.0004},{'params':[opacity],'lr':.0025},{'params':[sh],'lr':.0008}],eps=1e-15)
    def render(T,K):
        image,alpha,meta=rasterization(means,q,torch.exp(logs),torch.sigmoid(opacity),sh,T[None],K[None],w,h,near_plane=.3,far_plane=60,sh_degree=1,packed=False,backgrounds=bg,rasterize_mode='antialiased')
        return image[0],alpha[0],meta
    def evaluate(tag):
        scores=[]
        with torch.no_grad():
            for index in heldout:
                pred,alpha,_=render(w2cs[index],Ks[index]);target=torch.tensor(images[index],dtype=torch.float32,device=device)/255;mse=torch.mean((pred-target)**2).item();scores.append(-10*math.log10(max(mse,1e-12)))
                if tag in ['initial','final']:
                    rgb8=(pred.clamp(0,1).cpu().numpy()*255).astype('u1');cv2.imwrite(str(out/'renders'/f'{tag}_{index:04d}.jpg'),cv2.cvtColor(rgb8,cv2.COLOR_RGB2BGR))
                    if tag=='final':cv2.imwrite(str(out/'renders'/f'comparison_{index:04d}.jpg'),cv2.cvtColor(np.hstack([images[index],rgb8]),cv2.COLOR_RGB2BGR))
        value=float(np.mean(scores));print('EVAL',tag,'PSNR',round(value,3),flush=True);return value
    parameters=[means,q,logs,opacity,sh]
    start=time.monotonic();initial_psnr=evaluate('initial');history=[];rng=np.random.default_rng(args.seed);order=[]
    best_psnr=initial_psnr;best_step=0;best_state=[p.detach().clone() for p in parameters]
    print('TRAINING',len(xyz),'gaussians',len(train),'train views',len(heldout),'held-out views',flush=True)
    for step in range(1,args.steps+1):
        if not order:order=list(rng.permutation(train))
        index=int(order.pop());target=torch.tensor(images[index],dtype=torch.float32,device=device)/255;optimizer.zero_grad(set_to_none=True);pred,alpha,meta=render(w2cs[index],Ks[index]);l1=torch.abs(pred-target).mean();loss=(pred-target).square().mean()+.02*torch.mean((means-initial_means)**2)+.001*torch.mean(torch.exp(logs)**2);
        if not torch.isfinite(loss):raise RuntimeError('Non-finite training loss')
        loss.backward();optimizer.step()
        with torch.no_grad():
            logs.clamp_(math.log(.005),math.log(.5));opacity.clamp_(-8,8);q.copy_(torch.nn.functional.normalize(q,dim=-1));shift=means-initial_means;norm=torch.linalg.vector_norm(shift,dim=1,keepdim=True);means.copy_(initial_means+shift*torch.clamp(.5/norm.clamp_min(1e-6),max=1))
        if step%50==0:
            row={'step':step,'loss':float(loss.detach()),'l1':float(l1.detach()),'elapsed_s':time.monotonic()-start};history.append(row);print(json.dumps(row),flush=True);(out/'progress.json').write_text(json.dumps(row))
        if step%args.eval_every==0 or step==args.steps:
            score=evaluate(str(step))
            if score>best_psnr:
                best_psnr=score;best_step=step;best_state=[p.detach().clone() for p in parameters]
    with torch.no_grad():
        for parameter,value in zip(parameters,best_state):parameter.copy_(value)
    if best_step==0:raise RuntimeError('Training failed to improve held-out views; no model exported')
    final_psnr=evaluate('final');export_gaussians(out/'gaussians.ply',means,q,logs,opacity,sh)
    torch.save({'means':means.detach().cpu(),'quats':q.detach().cpu(),'log_scales':logs.detach().cpu(),'opacity_logits':opacity.detach().cpu(),'sh':sh.detach().cpu(),'background':bg.cpu(),'scene':str(args.scene)},out/'model.pt')
    report={'method':'LiDAR-initialized anisotropic 3D Gaussian Splatting, SH degree 1, fixed calibrated cameras','gsplat_version':__import__('gsplat').__version__,'steps':args.steps,'selected_step':best_step,'loss':'MSE + position/scale regularization','gaussians':len(xyz),'train_images':len(train),'heldout_images':len(heldout),'heldout_psnr_initial_db':initial_psnr,'heldout_psnr_final_db':final_psnr,'elapsed_s':time.monotonic()-start,'peak_gpu_memory_mb':torch.cuda.max_memory_allocated()/2**20,'background_rgb':bg[0].tolist(),'scene':str(args.scene),'notes':['Local validation, not a full-route model.','No image-based bundle adjustment; poses use SLAM and existing candidate camera extrinsics.','Sky and moving objects are not explicitly modeled.','Validation images are withheld from optimization; initialization point colors use the complete exported scene.','PLY stores centered local-map coordinates; see scene.json for original map offset.']};(out/'training_report.json').write_text(json.dumps(report,indent=2));(out/'history.json').write_text(json.dumps(history))
    # Smooth render path through one physical camera; avoids direction jumps.
    camera_views=[v for v in views if v['camera_id']==1];tt=np.array([v['stamp_s'] for v in camera_views]);tt-=tt[0];m=np.array([v['camera_to_world'] for v in camera_views]);rot=Slerp(tt,Rotation.from_matrix(m[:,:3,:3]));frames=[]
    for t in np.linspace(0,tt[-1],72):
        pose=np.eye(4);pose[:3,:3]=rot(t).as_matrix()
        for j in range(3):pose[j,3]=np.interp(t,tt,m[:,j,3])
        pose[:3,3]+=pose[:3,0]*.15
        with torch.no_grad():rgb_image,_,_=render(torch.tensor(np.linalg.inv(pose),dtype=torch.float32,device=device),Ks[0]);frames.append((rgb_image.clamp(0,1).cpu().numpy()*255).astype('u1'))
    from PIL import Image
    Image.fromarray(frames[0]).save(out/'flythrough.gif',save_all=True,append_images=[Image.fromarray(v) for v in frames[1:]],duration=100,loop=0)
    print('GS_MODEL_READY',json.dumps(report),flush=True)


if __name__=='__main__':main()
