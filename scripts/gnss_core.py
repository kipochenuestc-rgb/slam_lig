"""WGS84/ENU conversion and scale-preserving online position alignment."""
import numpy as np

def ecef(longitude,latitude,altitude):
    lon,lat=np.radians([longitude,latitude]);e2=6.6943799901413165e-3
    n=6378137./np.sqrt(1-e2*np.sin(lat)**2)
    return np.array([(n+altitude)*np.cos(lat)*np.cos(lon),(n+altitude)*np.cos(lat)*np.sin(lon),(n*(1-e2)+altitude)*np.sin(lat)])

class LocalENU:
    def __init__(self,longitude,latitude,altitude):
        self.reference=[float(longitude),float(latitude),float(altitude)]
        self.origin=ecef(*self.reference);lon,lat=np.radians([longitude,latitude])
        self.rotation=np.array([[-np.sin(lon),np.cos(lon),0],[-np.sin(lat)*np.cos(lon),-np.sin(lat)*np.sin(lon),np.cos(lat)],[np.cos(lat)*np.cos(lon),np.cos(lat)*np.sin(lon),np.sin(lat)]])
    def convert(self,longitude,latitude,altitude):
        return self.rotation@(ecef(longitude,latitude,altitude)-self.origin)

def fix_covariance(values,known,horizontal_std,vertical_std):
    fallback=np.diag([horizontal_std**2,horizontal_std**2,vertical_std**2])
    c=np.asarray(values,dtype=float).reshape(3,3)
    if not known or not np.isfinite(c).all():return fallback
    c=(c+c.T)/2
    if np.any(np.diag(c)<=0) or np.linalg.eigvalsh(c).min() < -1e-8:return fallback
    return c

def fit_alignment(enu,poses,min_points=12,min_baseline=15.,max_rmse=1.5):
    p=np.asarray(enu,dtype=float);q=np.asarray(poses,dtype=float)
    if len(p)<min_points or p.shape!=q.shape or not np.isfinite([p,q]).all():return None
    keep=np.ones(len(p),dtype=bool)
    span=lambda v:float(np.max(np.linalg.norm(v[:,None,:2]-v[None,:,:2],axis=2)))
    for _ in range(3):
        a=p[keep];b=q[keep]
        if len(a)<min_points or min(span(a),span(b))<min_baseline:return None
        ap=a[:,:2].mean(0);bp=b[:,:2].mean(0)
        u,_,vt=np.linalg.svd((a[:,:2]-ap).T@(b[:,:2]-bp))
        if np.linalg.det(u@vt)<0:u[:,-1]*=-1
        rotation=np.eye(3);rotation[:2,:2]=(u@vt).T
        translation=np.r_[bp-rotation[:2,:2]@ap,np.median(b[:,2]-a[:,2])]
        errors=np.linalg.norm((p@rotation.T+translation-q)[:,:2],axis=1)
        median=float(np.median(errors[keep]));mad=float(np.median(np.abs(errors[keep]-median)))
        updated=errors<=max(.5,median+3*1.4826*mad)
        if np.array_equal(updated,keep):break
        keep=updated
    if keep.sum()<min_points or keep.sum()<.8*len(p):return None
    rmse=float(np.sqrt(np.mean(errors[keep]**2)))
    if rmse>max_rmse:return None
    return rotation,translation,{'pairs':int(keep.sum()),'horizontal_rmse_m':rmse,'baseline_m':span(p[keep]),'yaw_enu_to_map_rad':float(np.arctan2(rotation[1,0],rotation[0,0]))}
