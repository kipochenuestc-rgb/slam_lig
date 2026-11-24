"""Export an unchanged trained model with gsplat's own standard exporters."""
import argparse,hashlib,json,pathlib
import numpy as np
import torch
from gsplat import export_splats


def read_ply(path):
    with path.open('rb') as f:
        names=[]
        while True:
            line=f.readline().decode().strip()
            if line.startswith('element vertex '):count=int(line.split()[-1])
            if line.startswith('property float '):names.append(line.split()[-1])
            if line=='end_header':break
        data=np.frombuffer(f.read(),dtype=[(k,'<f4') for k in names])
    if len(data)!=count:raise ValueError('Invalid vertex payload')
    return data


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--checkpoint',type=pathlib.Path,required=True);ap.add_argument('--output',type=pathlib.Path,required=True)
    args=ap.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    checkpoint=torch.load(args.checkpoint,map_location='cpu',weights_only=False);p=checkpoint['params']
    p={k:v.detach().float().contiguous() for k,v in p.items() if k in ['means','scales','quats','opacities','sh0','shN']}
    for key,value in p.items():
        if not torch.isfinite(value).all():raise ValueError('Non-finite '+key)
    p['quats']=torch.nn.functional.normalize(p['quats'],dim=-1)
    args.output.mkdir(parents=True);manifest={'source_checkpoint':str(args.checkpoint),'source_step':checkpoint['step'],'vertices':len(p['means']),'exporter':'gsplat.export_splats','files':{},'geometry_changed':False}
    for format,name in [('ply','model_standard.ply'),('splat','model.splat')]:
        path=args.output/name;export_splats(p['means'],p['scales'],p['quats'],p['opacities'],p['sh0'],p['shN'],format=format,save_to=str(path))
        manifest['files'][name]={'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    data=read_ply(args.output/'model_standard.ply')
    assert len(data)==len(p['means'])
    for fields,key in [(['x','y','z'],'means'),([f'scale_{i}' for i in range(3)],'scales'),([f'rot_{i}' for i in range(4)],'quats')]:
        np.testing.assert_array_equal(np.column_stack([data[k] for k in fields]),p[key].numpy())
    np.testing.assert_array_equal(data['opacity'],p['opacities'].numpy())
    np.testing.assert_array_equal(np.column_stack([data[f'f_rest_{i}'] for i in range(45)]),p['shN'].numpy().transpose(0,2,1).reshape(len(data),45))
    raw=np.fromfile(args.output/'model.splat',dtype=np.dtype([('xyz','<f4',(3,)),('scales','<f4',(3,)),('rgba','u1',(4,)),('quat','u1',(4,))]))
    assert len(raw)==len(data) and np.isfinite(raw['xyz']).all() and (raw['scales']>0).all()
    manifest['notes']=['Standard PLY retains all SH3 coefficients, log scales and opacity logits.','SPLAT is 32 bytes per Gaussian, linear scales, RGBA and quantized quaternion; higher SH coefficients are omitted.','No rotation, scale adjustment, denoising or retraining applied.','Format conversion does not repair reconstruction artifacts.']
    (args.output/'export_report.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest),flush=True)


if __name__=='__main__':main()
