"""Synthetic side-view width budget; pinhole projection of a rotated ellipse.
Marked reference is assumed coplanar; contours are presumed segmented. No real camera input.
"""
import numpy as np

def camera_study(seed=17004,n=2400):
    rng=np.random.default_rng(seed)
    w0=rng.uniform(.038,.047,n)
    aspect=rng.uniform(1.25,1.45,n)
    D=rng.uniform(.01,.12,n)
    depth0=rng.uniform(.32,.46,n)
    depth=depth0+rng.uniform(-.035,.035,n)
    theta=rng.uniform(-.22,.22,n)
    focal=610. # synthetic px
    long=w0*aspect
    projected=np.sqrt(((1-D)*w0*np.cos(theta))**2+(long*np.sin(theta))**2)
    def edges():
        return rng.normal(0,.45,(2,n)).sum(axis=0)
    base=focal*w0/depth0+edges()
    observed=focal*projected/depth+edges()
    # Calibration target of known width at both times gives local scale.
    marker_length=.050
    m0=focal*marker_length/depth0+edges()
    m=focal*marker_length/depth+edges()
    mm=observed/m*marker_length
    baseline=base/m0*marker_length
    est_angle=theta+rng.normal(0,.02,n)
    long_est=long*(1+rng.normal(0,.015,n))
    corrected=np.sqrt(np.maximum(mm**2-(long_est*np.sin(est_angle))**2,0))/np.cos(est_angle)
    occ=rng.random(n)<.12
    rows=[]
    for name,estimate,valid in [('pixel_ratio',1-observed/base,np.ones(n,bool)),
                                ('scale_pose',1-corrected/baseline,np.ones(n,bool)),
                                ('scale_pose_with_gaps',1-corrected/baseline,~occ)]:
        error=np.abs(estimate[valid]-D[valid])
        rows.append(dict(estimator=name,n=n,valid=int(valid.sum()),mae_fraction=error.mean(),
                         abs_error_p95_fraction=np.quantile(error,.95)))
    return rows
