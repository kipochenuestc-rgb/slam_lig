"""Geometry filtering that retains an actual point and its complete time record."""
import numpy as np
from scipy.spatial import cKDTree
from fusion_core import DTYPE

def filter_points(points, min_range=1.5, max_range=100., leaf_size=.2, mean_k=20, stddev_mul=2., query_workers=4):
    original=len(points)
    xyz=np.column_stack([points[k] for k in ['x','y','z']])
    good=np.isfinite(xyz).all(1)&np.isfinite(points['time'])
    radius=np.linalg.norm(xyz,axis=1)
    good&=(radius>=min_range)&(radius<=max_range)
    points=points[good];xyz=xyz[good];after_range=len(points)
    if leaf_size>0 and len(points):
        grid=np.floor(xyz/leaf_size).astype(np.int64);lo=grid.min(0);span=grid.max(0)-lo+1;grid-=lo
        keys=(grid[:,0]*span[1]+grid[:,1])*span[2]+grid[:,2]
        _,inverse,counts=np.unique(keys,return_inverse=True,return_counts=True)
        centers=np.column_stack([np.bincount(inverse,weights=xyz[:,j],minlength=len(counts))/counts for j in range(3)])
        distance=np.sum((xyz-centers[inverse])**2,axis=1)
        order=np.lexsort((distance,inverse));groups=inverse[order];selected=order[np.r_[True,groups[1:]!=groups[:-1]]]
        # Keep the observed sample closest to the voxel centroid; do NOT average
        # timestamps, sensor identifiers, or coordinates from different times.
        points=points[selected];xyz=xyz[selected]
    after_voxel=len(points)
    if mean_k>0 and len(points)>mean_k:
        distances,_=cKDTree(xyz).query(xyz,k=min(mean_k+1,len(points)),workers=query_workers)
        means=distances[:,1:].mean(1);threshold=float(means.mean()+stddev_mul*means.std())
        points=points[means<=threshold]
    points=points[np.argsort(points['time'],kind='stable')]
    if points.dtype!=DTYPE:raise ValueError('Filter changed the declared point layout')
    counts={str(i):int(np.count_nonzero(points['lidar_id']==i)) for i in range(1,5)}
    return points,{'input_points':original,'after_range':after_range,'after_voxel':after_voxel,'output_points':len(points),'per_lidar':counts}

def filter_message(message,**options):
    from sensor_msgs.msg import PointCloud2
    if message.point_step!=32 or len(message.data)!=message.row_step*message.height:
        raise ValueError('Expected valid 32-byte fusion point layout')
    points=np.frombuffer(message.data,dtype=DTYPE)
    points,info=filter_points(points,**options)
    output=PointCloud2();output.header=message.header;output.fields=message.fields;output.height=1;output.width=len(points);output.point_step=32;output.row_step=32*len(points);output.is_bigendian=False;output.is_dense=True;output.data=points.tobytes()
    return output,info
