"""Export final optimized poses and a top-down preview without changing web data."""
import csv,json,pathlib,re,sys
import numpy as np


def read_pcd(path):
    with path.open('rb') as f:
        header={}
        while True:
            line=f.readline()
            if not line:raise ValueError('Incomplete PCD header')
            line=line.decode().strip()
            if not line or line.startswith('#'):continue
            key,*values=line.split();header[key]=values
            if key=='DATA':break
        if header['DATA']!=['binary']:raise ValueError('Expected binary PCD')
        kinds={'F':'f','I':'i','U':'u'}
        dtype=np.dtype([(name,'<'+kinds[kind]+size) for name,kind,size in zip(header['FIELDS'],header['TYPE'],header['SIZE'])])
        return np.frombuffer(f.read(),dtype=dtype,count=int(header['POINTS'][0]))


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root=pathlib.Path(__file__).resolve().parents[1];run=pathlib.Path(sys.argv[1])
    poses=read_pcd(run/'map/transformations.pcd');t=poses['time'];xyz=np.column_stack([poses[k] for k in ['x','y','z']]).astype(float)
    if not np.isfinite(xyz).all() or np.any(np.diff(t)<=0):raise ValueError('Invalid optimized trajectory')
    with (run/'optimized_trajectory.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['stamp_s','x','y','z','roll','pitch','yaw']);writer.writerows(np.column_stack([t,xyz,poses['roll'],poses['pitch'],poses['yaw']]))
    gnss_path=run/'gnss_constraints.csv'
    # This is an output log of live fixes actually published by this run.
    if gnss_path.exists() and len(gnss_path.read_text().splitlines())>1:
        gnss=np.loadtxt(gnss_path,delimiter=',',skiprows=1,ndmin=2)
    else:gnss=np.empty((0,4))
    valid=(gnss[:,0]>=t[0])&(gnss[:,0]<=t[-1]);g=gnss[valid]
    interpolated=np.column_stack([np.interp(g[:,0],t,xyz[:,j]) for j in range(3)])
    residual=interpolated-g[:,1:4];xy=np.linalg.norm(residual[:,:2],axis=1)
    with (run/'gnss_constraint_residuals.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['stamp_s','dx','dy','dz','horizontal_m']);writer.writerows(np.column_stack([g[:,0],residual,xy]))
    corners=read_pcd(run/'map/CornerMap.pcd');surfaces=read_pcd(run/'map/SurfMap.pcd')
    clouds=[np.column_stack([p[k] for k in ['x','y','z']]) for p in [corners,surfaces]]
    p=np.concatenate(clouds);p=p[np.isfinite(p).all(1)]
    sample=p[np.linspace(0,len(p)-1,min(len(p),180000),dtype=int)]
    # XY-only PLY is convenient for inspecting the requested 2D projection.
    _,idx=np.unique(np.floor(sample[:,:2]/.25).astype(np.int64),axis=0,return_index=True);flat=sample[idx,:2]
    with (run/'map_2d.ply').open('wb') as f:
        f.write(f'ply\nformat binary_little_endian 1.0\nelement vertex {len(flat)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'.encode());f.write(np.column_stack([flat,np.zeros(len(flat))]).astype('<f4').tobytes())
    factors=re.findall(r'GNSS_FACTOR count=(\d+)',(run/'launch.log').read_text())
    summary={'optimized_keyframes':len(t),'duration_s':float(t[-1]-t[0]),'path_length_m':float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),'saved_corner_points':len(corners),'saved_surface_points':len(surfaces),'gnss_factors':int(factors[-1]) if factors else 0,'imu_resets':(run/'launch.log').read_text().count('reset IMU-preintegration'),'constraint_residuals_not_independent_accuracy':({'count':len(g),'horizontal_rmse_m':float(np.sqrt(np.mean(xy**2))),'horizontal_p95_m':float(np.quantile(xy,.95)),'height_rmse_m':float(np.sqrt(np.mean(residual[:,2]**2))),'last_height_residual_m':float(residual[-1,2])} if len(g) else {'count':0}),'map_xyz_min':p.min(0).tolist(),'map_xyz_max':p.max(0).tolist()}
    (run/'result_summary.json').write_text(json.dumps(summary,indent=2))
    fig,axes=plt.subplots(1,2,figsize=(13,6),layout='constrained')
    ax=axes[0];ax.scatter(sample[:,0],sample[:,1],s=.15,c=np.clip(sample[:,2],*np.quantile(sample[:,2],[.02,.98])),cmap='viridis',rasterized=True);ax.plot(xyz[:,0],xyz[:,1],color='red',lw=1,label='Optimized SLAM');ax.plot(g[:,1],g[:,2],color='orange',lw=.7,label='GNSS constraints');ax.set_aspect('equal');ax.legend();ax.set(xlabel='Map x (m)',ylabel='Map y (m)',title='Four lidars: fused, filtered, IMU deskewed')
    axes[1].plot(g[:,0]-t[0],xy,label='Horizontal');axes[1].plot(g[:,0]-t[0],residual[:,2],label='Height (signed)');axes[1].set(xlabel='Time from SLAM start (s)',ylabel='Constraint residual (m)',title='GNSS residuals (not independent accuracy)');axes[1].grid(alpha=.25);axes[1].legend()
    fig.savefig(run/'map_preview.png',dpi=170);print(json.dumps(summary))


if __name__=='__main__':main()
