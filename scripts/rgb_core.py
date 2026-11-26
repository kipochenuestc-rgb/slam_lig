"""Camera calibration, timestamped poses and measured-point RGB projection."""
import json,pathlib
import cv2
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def transform(T,points):
    return np.asarray(points)@T[:3,:3].T+T[:3,3]


def load_rig(calibration_dir):
    root=pathlib.Path(calibration_dir)
    rig=json.loads((root/'results/rig_cam0_with_lidars_paired_1203.json').read_text())
    session=json.loads((root/'config/aprilgrid_session.json').read_text())
    mount=json.loads((root/'config/imu_mount.json').read_text())
    imu_cam=np.eye(4);imu_cam[:3,:3]=np.column_stack([mount['camera0_axes_in_imu'][k] for k in ['x','y','z']]);imu_cam[:3,3]=mount['camera0_origin_in_imu_m']
    origin=(imu_cam@np.array(rig['lidars']['0']['T_reference_from_lidar']))[:3,3]
    cameras={};lidars={}
    for key,item in rig['cameras'].items():
        intr=item['intrinsics_used'];T=imu_cam@np.array(item['T_reference_from_camera']);T[:3,3]-=origin
        cameras[int(key)]={'K':np.array(intr['K']), 'D':np.array(intr['D']), 'size':tuple(intr['image_size']), 'reference_from_camera':T}
    for key,item in rig['lidars'].items():
        T=imu_cam@np.array(item['T_reference_from_lidar']);T[:3,3]-=origin;cid=item['physical_camera_id']
        paired=np.linalg.inv(cameras[cid]['reference_from_camera'])@T
        if not np.allclose(paired,np.array(item['T_camera_from_lidar']),atol=2e-6):raise ValueError('Inconsistent paired-camera extrinsics')
        lidars[int(key)+1]={'camera_id':cid,'reference_from_lidar':T,'camera_from_lidar':paired}
    return cameras,lidars,session['quadrant_to_camera'],rig['status']


def decode_mosaic(message):
    raw=np.frombuffer(message.data,dtype=np.uint8)
    if message.encoding.lower()=='nv12':
        if message.height%3:raise ValueError('NV12 storage height must be 3/2 of image height')
        image=cv2.cvtColor(raw.reshape(message.height,message.step)[:,:message.width],cv2.COLOR_YUV2BGR_NV12)
    elif message.encoding.lower() in ['bgr8','rgb8']:
        image=raw.reshape(message.height,message.step)[:,:message.width*3].reshape(message.height,message.width,3)
        if message.encoding.lower()=='rgb8':image=image[:,:,::-1]
    elif raw.size>2 and raw[0]==255 and raw[1]==216:image=cv2.imdecode(raw,cv2.IMREAD_COLOR)
    else:raise ValueError('Unsupported camera encoding: '+message.encoding)
    if image is None:raise ValueError('Failed to decode image')
    return image


def split_cameras(image,quadrants,cameras):
    h,w=image.shape[:2]
    if h%2 or w%2:raise ValueError('Expected a 2x2 camera mosaic')
    bounds={'tl':(0,h//2,0,w//2),'tr':(0,h//2,w//2,w),'bl':(h//2,h,0,w//2),'br':(h//2,h,w//2,w)}
    result={}
    for name,cid in quadrants.items():
        y0,y1,x0,x1=bounds[name];tile=image[y0:y1,x0:x1]
        if (tile.shape[1],tile.shape[0])!=cameras[cid]['size']:raise ValueError('Image/calibration resolution mismatch')
        result[cid]=tile.copy()
    return result


class Poses:
    def __init__(self,pcd_file):
        from analyze_run import read_pcd
        p=read_pcd(pathlib.Path(pcd_file));self.times=p['time'].astype(float);self.start=float(self.times[0]);self.end=float(self.times[-1])
        if np.any(np.diff(self.times)<=0):raise ValueError('Pose times are not strictly increasing')
        self.xyz=np.column_stack([p[k] for k in ['x','y','z']]);self.slerp=Slerp(self.times-self.start,Rotation.from_euler('xyz',np.column_stack([p[k] for k in ['roll','pitch','yaw']])))

    def at(self,times):
        t=np.atleast_1d(times).astype(float)
        if t.min()<self.start-.02 or t.max()>self.end+.02:raise ValueError('Requested pose outside optimized trajectory')
        t=np.clip(t,self.start,self.end);m=np.broadcast_to(np.eye(4),(len(t),4,4)).copy();m[:,:3,:3]=self.slerp(t-self.start).as_matrix()
        for j in range(3):m[:,j,3]=np.interp(t,self.times,self.xyz[:,j])
        return m[0] if np.ndim(times)==0 else m

    def points_to_world(self,xyz,times):
        m=self.at(times);return np.einsum('nij,nj->ni',m[:,:3,:3],xyz)+m[:,:3,3]


def cloud_arrays(message):
    fields={f.name:f for f in message.fields};formats={2:'u1',4:'u2',6:'u4',7:'f4',8:'f8'};endian='>' if message.is_bigendian else '<'
    def get(name):
        f=fields[name];return np.ndarray((message.height,message.width),dtype=endian+formats[f.datatype],buffer=message.data,offset=f.offset,strides=(message.row_step,message.point_step)).ravel()
    return np.column_stack([get(k) for k in ['x','y','z']]),get('timestamp').astype(float)*1e-6


def fish_project(points,K,D,size,max_angle_deg=85):
    uv=cv2.fisheye.projectPoints(np.asarray(points,dtype=np.float64).reshape(-1,1,3),np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
    w,h=size;r=np.linalg.norm(points,axis=1)
    good=np.isfinite(uv).all(1)&np.isfinite(points).all(1)&(points[:,2]>0)&(points[:,2]>=r*np.cos(np.radians(max_angle_deg)))&(uv[:,0]>=1)&(uv[:,0]<w-2)&(uv[:,1]>=1)&(uv[:,1]<h-2)
    return uv,good


def visible_pixels(points,uv,good,size):
    w,h=size;indices=np.flatnonzero(good);pix=np.rint(uv[indices]).astype(int);depth=np.linalg.norm(points[indices],axis=1)
    z=np.full(h*w,np.inf);flat=pix[:,1]*w+pix[:,0];np.minimum.at(z,flat,depth)
    return indices[depth<=z[flat]+.15+.005*depth]


def voxel_rgb(xyz,rgb,leaf):
    if not len(xyz):return xyz,rgb
    grid=np.floor(xyz/leaf).astype(np.int64);_,inv,counts=np.unique(grid,axis=0,return_inverse=True,return_counts=True)
    p=np.column_stack([np.bincount(inv,weights=xyz[:,j])/counts for j in range(3)])
    c=np.column_stack([np.bincount(inv,weights=rgb[:,j])/counts for j in range(3)])
    return p.astype(np.float32),np.rint(c).clip(0,255).astype(np.uint8)


def write_rgb_ply(path,xyz,rgb):
    a=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])
    for j,k in enumerate(['x','y','z']):a[k]=xyz[:,j]
    for j,k in enumerate(['red','green','blue']):a[k]=rgb[:,j]
    with pathlib.Path(path).open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\nelement vertex '+str(len(xyz))+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode());f.write(a.tobytes())
