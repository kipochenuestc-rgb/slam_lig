"""Separate static surfaces, people/vehicles and sky for geometry-constrained training."""
import argparse,json,pathlib
import cv2,numpy as np,torch
import torch.nn.functional as F
from transformers import SegformerImageProcessor,SegformerForSemanticSegmentation


def semantic_masks(labels,rgb,dynamic_ids,original_person=None):
    dynamic=np.isin(labels,dynamic_ids).astype('u1')
    if original_person is not None:
        count,components,stats,_=cv2.connectedComponentsWithStats((original_person==0).astype('u1'))
        for component in range(1,count):
            region=components==component
            if np.count_nonzero(region&(labels==12))>=max(5,stats[component,cv2.CC_STAT_AREA]*.05):dynamic[region]=1
    dynamic=cv2.dilate(dynamic,np.ones((9,9),'u1'));sky=((labels==2)&(rgb.min(2)>140)).astype('u1')
    return (1-dynamic)*(1-sky),sky,dynamic


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--scene',type=pathlib.Path,required=True);ap.add_argument('--output',type=pathlib.Path,required=True);ap.add_argument('--person-masks',type=pathlib.Path)
    args=ap.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    for name in ['static','sky','dynamic','labels','preview']:(args.output/name).mkdir()
    name='nvidia/segformer-b0-finetuned-ade-512-512';processor=SegformerImageProcessor.from_pretrained(name);model=SegformerForSemanticSegmentation.from_pretrained(name).cuda().eval()
    views=json.loads((args.scene/'cameras.json').read_text());summary=[]
    dynamic_ids=[i for i,label in model.config.id2label.items() if label.strip() in ['person','car','bus','truck','van']]
    with torch.no_grad():
        for i,v in enumerate(views):
            bgr=cv2.imread(str(args.scene/v['image_path']));rgb=bgr[:,:,::-1].copy();inputs=processor(images=rgb,return_tensors='pt').to('cuda')
            logits=model(**inputs).logits;labels=F.interpolate(logits,size=rgb.shape[:2],mode='bilinear',align_corners=False)[0].argmax(0).cpu().numpy().astype('u1')
            original=None
            if args.person_masks:
                original=cv2.imread(str(args.person_masks/f'{i:04d}.png'),0)
                if original is None:raise ValueError('Missing person mask')
            static,sky,dynamic=semantic_masks(labels,rgb,dynamic_ids,original)
            for folder,data in [('static',static*255),('sky',sky*255),('dynamic',dynamic*255),('labels',labels)]:cv2.imwrite(str(args.output/folder/f'{i:04d}.png'),data)
            if i<4 or i%16==1:
                overlay=bgr.copy();overlay[sky>0]=(180,110,30);overlay[dynamic>0]=(40,40,240);overlay=cv2.addWeighted(bgr,.6,overlay,.4,0);cv2.imwrite(str(args.output/'preview'/f'{i:04d}.jpg'),np.hstack([bgr,overlay]))
            summary.append({'image':v['image_path'],'static_fraction':float(static.mean()),'sky_fraction':float(sky.mean()),'dynamic_fraction':float(dynamic.mean())})
            if i%24==23:print('SEGMENTED',i+1,flush=True)
    (args.output/'semantics.json').write_text(json.dumps({'model':name,'views':summary,'dynamic_labels':[model.config.id2label[k] for k in dynamic_ids]},indent=2));print('READY',len(views),flush=True)


if __name__=='__main__':main()
