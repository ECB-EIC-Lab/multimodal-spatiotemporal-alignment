"""Offline reference trajectory and kinematics. All lengths m, angles rad, time s."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

# Kinematic parameters of base_link -> link7 from agilexrobotics/agx_arm_urdf,
# nero/urdf/nero_description.urdf, commit 680fa925cd00db7e2628563d2edc5540963045de.
ORIGIN = np.array([[0,0,.138], [0,0,0], [0,-.31,0], [0,0,0], [0,-.27001,0],
                   [0,0,0], [0,-.0235,0]])
RPY = np.array([[0,0,0], [1.5707963,3.1415926,0], [-1.5707963,0,3.1415926926],
                [-1.5707963,0,3.1415926926], [1.5707963,-1.5707963,0],
                [1.5707963,-1.5707963,0], [1.5707963,0,0]])
REST = Rotation.from_euler('xyz', RPY).as_matrix()
LIMIT = np.array([[-2.70526,2.70526],[-1.74,1.74],[-2.75,2.75],[-1.01,2.14],
                  [-2.75,2.75],[-.7330383,.8726646],[-1.5707963,1.5707963]])
START = np.array([0,.45,.12,1.05,-.18,.10,-.10])
DT = .05  # outer loop; synthetic assumption, independent of hardware

def trajectory(mode):
    """Fifth-order rest-to-rest lift, carry, hold and place at the target."""
    travel_time = {'gentle': 1.0, 'brisk': .4}[mode]
    segments = [('support', .35, [0,0,0]), ('lift', travel_time, [0,0,.05]),
                ('carry', travel_time, [0,.10,0]), ('hold', .50, [0,0,0]),
                ('place', travel_time, [0,0,-.05]), ('supported', .25, [0,0,0])]
    start = np.zeros(3)
    rows = []
    for stage,duration,change in segments:
        delta = np.array(change)
        steps = round(duration / DT)
        for k in range(steps):
            s = k / steps
            blend = s**3 * (10 - 15*s + 6*s*s)
            accel = 60*s*(1-s)*(1-2*s) / duration**2
            load = {'support': 0., 'lift': min(1., k*DT/.15),
                    'place': min(1., (1-s)*duration/.15), 'supported': 0.}.get(stage,1.)
            rows.append((stage, start + blend*delta, accel*delta, load))
        start += delta
    return rows

def forward(q):
    xyz = np.zeros(3)
    rot = np.eye(3)
    for theta,offset,base in zip(q,ORIGIN,REST):
        xyz += rot @ offset
        rot = rot @ base @ Rotation.from_rotvec([0,0,theta]).as_matrix()
    return xyz,rot

def check_path(mode):
    """Bounded numerical IK for link7 with fixed orientation, warm-started on each sample."""
    xyz0, rot0 = forward(START)
    q = START.copy()
    samples = []
    for k,(stage,delta,accel,load) in enumerate(trajectory(mode)):
        target = xyz0 + delta
        def residual(joints):
            pos, rot = forward(joints)
            angle = Rotation.from_matrix(rot0.T @ rot).as_rotvec()
            return np.r_[pos-target, .18*angle, 1e-5*(joints-q)]
        sol = least_squares(residual, q, bounds=tuple(LIMIT.T), gtol=1e-12,
                            ftol=1e-12, xtol=1e-12, max_nfev=60)
        new = sol.x
        pos,rot = forward(new)
        angle = Rotation.from_matrix(rot0.T @ rot).magnitude()
        samples.append(dict(mode=mode,step=k,stage=stage,
               pos_error_mm=1000*np.linalg.norm(pos-target), angle_error_deg=np.degrees(angle),
               limit_margin_rad=np.min(np.minimum(new-LIMIT[:,0],LIMIT[:,1]-new)),
               max_joint_speed_rad_s=np.max(np.abs(new-q))/DT,
               flange_accel_m_s2=np.linalg.norm(accel)))
        q = new
    return samples
