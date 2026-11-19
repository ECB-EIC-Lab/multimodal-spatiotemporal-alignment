"""Reduced three-contact plant and controllers. SI units except normalized closure."""
from dataclasses import dataclass
import numpy as np
from motion import DT, trajectory

SHAPE = np.array([1.16, .94, .90])  # thumb, index, middle preset
WEIGHT = np.array([1.0, 1.2, 1.1])
POLICY = {
    'raw': dict(mu=.34, cap=1.12, preload=.43),
    'peeled': dict(mu=.30, cap=1.30, preload=.46),
}
D_LIMIT = .078
ERROR_F, DELTA_T, ETA = .075, .035, 1.06
SLEW, TRUST = .42, .48
ALPHA = .055
DT_INNER = DT/5
TOL = 1e-9

@dataclass
class Plant:
    mass: np.ndarray
    share: np.ndarray
    mu: np.ndarray
    spring: np.ndarray
    compliance: np.ndarray
    threshold: np.ndarray
    noise_f: np.ndarray
    noise_t: np.ndarray
    noise_d: np.ndarray

def population(seed, n, kind, steps):
    rng = np.random.default_rng(seed)
    share = rng.normal([.40,.31,.29], [.035,.025,.025], (n,3)).clip(.15,.55)
    share /= share.sum(axis=1,keepdims=True)
    raw = kind == 'raw'
    mu = rng.uniform(*((.30,.60) if raw else (.23,.50)), (n,1))
    mu = (mu + rng.normal(0,.018,(n,3))).clip(.16,.68)
    return Plant(rng.uniform(.046,.070,n), share, mu, rng.uniform(2.1,3.0,(n,3)),
                 rng.uniform(.065,.13,n),
                 rng.uniform(.82,1.48,n) if raw else rng.uniform(.070,.109,n),
                 rng.normal(0,.012,(steps+1,n,3)), rng.normal(0,.005,(steps+1,n,3)),
                 rng.normal(0,.0015,(steps+1,n)))

def step_pi(f, q, prev_error, ref, noise, spring):
    """Incremental PI, shared actuators; neutral feed-through bounded by travel and rate."""
    for _ in range(5):
        error = ref - (f+noise)
        prop = .12*(error-prev_error)
        integ = 6.5*DT_INNER*error
        dq = np.clip(prop+integ, -.08, .08)
        possible = np.clip(q+dq, 0, .9)-q
        # Conditional integration on saturation in the direction of the integral.
        outward = (prop+integ-possible)*integ > 1e-12
        dq = np.where(outward, np.clip(prop,-.08,.08), dq)
        q = np.clip(q+dq,0,.9)
        elastic = spring*(.94*q+.06*q.mean(axis=1,keepdims=True))
        f = f + .48*(elastic-f)
        prev_error = error
    return f,q,prev_error

def deformation(f, creep, compliance):
    # The hidden plant has elastic, delayed and nonlinear components. D is fractional width loss.
    effective = .44*f[:,0]+.31*f[:,1]+.25*f[:,2]
    creep = creep + (1-np.exp(-DT/.48))*(effective-creep)
    d = compliance*(.58*effective+.42*creep+.06*effective**2)
    return d,creep

def deformation_fit():
    """Train on synthetic excitation, estimate one-sided residual on separate specimens."""
    def sample(seed):
        n,steps = 64,44
        p=population(seed,n,'peeled',steps)
        rng=np.random.default_rng(seed+1)
        f=rng.uniform(.35,.70,(n,3))
        q=f/p.spring
        old=np.zeros_like(f)
        d,creep=deformation(f,f.mean(axis=1),p.compliance)
        x,y=[],[]
        for k in range(steps):
            seen=f+p.noise_f[k]
            ref=np.clip(seen+rng.uniform(-.31,.31,(n,3)),.20,1.35)
            f,q,old=step_pi(f,q,old,ref,p.noise_f[k],p.spring)
            new,creep=deformation(f,creep,p.compliance)
            x.append(np.column_stack((np.ones(n),ref-seen)))
            y.append(new-(d+p.noise_d[k]))
            d=new
        return np.vstack(x),np.concatenate(y)
    x,y=sample(11001)
    coef=np.linalg.lstsq(x,y,rcond=None)[0]
    x,y=sample(12001)
    margin=float(np.quantile(y-x@coef,.99))
    x,y=sample(13001)
    return dict(a=float(coef[0]),g=coef[1:].tolist(),eps_d=margin,
                independent_coverage=float(np.mean(y<=x@coef+margin)),
                split_ids=[11001,12001,13001],specimens_per_split=64)

def project(z, lo, hi, g=None, bound=None):
    """Weighted projection on box intersected with one half-space, vectorized by trial."""
    r=np.clip(z,lo,hi)
    box_ok=(lo<=hi).all(axis=1)
    half_ok=np.ones(len(r),dtype=bool)
    if g is None:
        return r,box_ok,half_ok
    g=np.asarray(g,float)
    half_ok=(np.minimum(lo*g,hi*g).sum(axis=1)<=bound+TOL)
    active=box_ok & half_ok & ((r*g).sum(axis=1)>bound+TOL)
    if active.any():
        # At this upper multiplier all nonzero coefficients reach their minimizing endpoint.
        zl,low,high=z[active],lo[active],hi[active]
        movable=g!=0
        endpoint=np.where(g>0,low,high)
        span=((zl[:,movable]-endpoint[:,movable])*WEIGHT[movable]/g[movable])
        left=np.zeros(len(zl))
        right=np.maximum(1.,span.max(axis=1)+1.)
        for _ in range(45):
            mid=(left+right)/2
            guess=np.clip(zl-mid[:,None]*g/WEIGHT,low,high)
            over=(guess*g).sum(axis=1)>bound[active]
            left=np.where(over,mid,left)
            right=np.where(over,right,mid)
        r[active]=np.clip(zl-right[:,None]*g/WEIGHT,low,high)
    return r,box_ok,half_ok

def simulate(kind, mode, method, seed, n, fit, fixed=.76, ratio=.74,
             stress='nominal', capture=False, width_sigma=.0015, example_id=0):
    if method not in ('A','B','shear','B_noD'):
        raise ValueError('Unknown controller: '+method)
    path=trajectory(mode)
    p=population(seed,n,kind,len(path))
    cfg=POLICY[kind]
    mu=p.mu*(.67 if stress=='low_friction' else 1.)
    f=np.broadcast_to(cfg['preload']*SHAPE,(n,3)).copy()
    q=f/p.spring
    old_error=np.zeros_like(f)
    target=f.copy()
    d,creep=deformation(f,f.mean(axis=1),p.compliance)
    slip=np.zeros(n)
    peak,peak_d,area=np.zeros(n),np.zeros(n),np.zeros(n)
    reason=np.full(n,'',dtype='<U20')
    count=np.zeros(n,int)
    coverage=np.zeros((n,6),int)
    ever_damage,ever_slip=np.zeros(n,bool),np.zeros(n,bool)
    trace=[]
    g=np.asarray(fit['g'])
    for k,(stage,_,accel,loading) in enumerate(path):
        active=reason==''
        if not active.any(): break
        # Return force *transmitted* at the fingertips. Excess demand becomes object slip.
        demand=p.share*(loading*p.mass*np.linalg.norm(accel+[0,0,9.81]))[:,None]
        tr=np.minimum(demand,mu*f)
        seen_t=np.maximum(0.,tr+p.noise_t[k])
        seen_f=np.maximum(0.,f+p.noise_f[k])
        seen_d=np.maximum(0.,d+p.noise_d[k]*(width_sigma/.0015))
        lo=np.maximum.reduce([ERROR_F+np.maximum(.29,ETA*(seen_t+DELTA_T)/cfg['mu']),
                              target-SLEW,seen_f-TRUST])
        hi=np.minimum.reduce([np.full_like(f,cfg['cap']-ERROR_F),target+SLEW,seen_f+TRUST])
        bound=D_LIMIT-fit['eps_d']-seen_d-fit['a']+seen_f@g
        if kind=='peeled' and stress=='stale_view' and k==round(.50/DT):
            reason[active]='stale_view'
        if method=='A':
            new=np.broadcast_to(fixed*SHAPE,(n,3))
        elif method=='shear':
            # Thresholded shear/normal heuristic with a deadband; adapted reference.
            utilization=seen_t/np.maximum(seen_f,.12)/cfg['mu']
            delta=np.where(utilization>ratio,.14,np.where(utilization<ratio-.22,-.05,0.))
            new=np.clip(target+delta,.32,cfg['cap']-ERROR_F)
        else:
            use_d=(kind=='peeled' and method=='B')
            new,box_ok,half_ok=project(target-ALPHA/WEIGHT,lo,hi,g if use_d else None,bound)
            reason[(reason=='') & ~box_ok]='box_abort'
            reason[(reason=='') & ~half_ok]='width_abort'
        moving=reason==''
        target=np.where(moving[:,None],new,target)
        next_f,q,old_error=step_pi(f,q,old_error,target,p.noise_f[k],p.spring)
        next_d,creep=deformation(next_f,creep,p.compliance)
        # Mid-interval demand is approximated by its value at the outer sample.
        deficit=np.maximum(0., demand-mu*(f+next_f)/2)
        slip+=np.where(moving,DT*105.*deficit.max(axis=1),0.)  # surrogate mm
        next_stage=path[min(k+1,len(path)-1)]
        next_demand=p.share*(next_stage[3]*p.mass*np.linalg.norm(next_stage[2]+[0,0,9.81]))[:,None]
        next_tr=np.minimum(next_demand,mu*next_f)
        coverage[:,0]+=moving
        coverage[:,1]+=moving & (np.abs(next_f-target).max(axis=1)<=ERROR_F)
        coverage[:,2]+=moving & ((next_tr-seen_t).max(axis=1)<=DELTA_T)
        prediction=seen_d+fit['a']+((target-seen_f)*g).sum(axis=1)
        coverage[:,3]+=moving & (next_d<=prediction+fit['eps_d'])
        covered_mu=(mu>=cfg['mu']).all(axis=1)
        coverage[:,4]+=moving & covered_mu
        coverage[:,5]+=moving & covered_mu & (np.abs(next_f-target).max(axis=1)<=ERROR_F) & ((next_tr-seen_t).max(axis=1)<=DELTA_T)
        peak=np.where(moving,np.maximum(peak,next_f.max(axis=1)),peak)
        peak_d=np.where(moving,np.maximum(peak_d,next_d),peak_d)
        area+=np.where(moving,DT*next_f.sum(axis=1),0.)
        count+=moving
        if kind=='raw':
            damaged=next_f.max(axis=1)>p.threshold
        else:
            damaged=next_d>p.threshold
        ever_damage |= moving & damaged
        ever_slip |= moving & (slip>2.0)
        reason[moving & damaged]='damage_proxy'
        reason[(reason=='') & (slip>2.0)]='slip_limit'
        reason[(reason=='') & ((next_f+p.noise_f[k+1]).max(axis=1)>1.45)]='force_guard'
        if capture and active[example_id]:
            j0=example_id
            final_status='complete' if k==len(path)-1 else 'active'
            line=dict(step=k,t_s=round(k*DT,3),stage=stage,
                      next_width_loss=next_d[j0] if moving[j0] and kind=='peeled' else '',
                      width_observed=seen_d[j0] if kind=='peeled' else '',
                      slip_mm=slip[j0],status=reason[j0] or final_status,
                      width_min_minus_bound=(np.minimum(lo[j0]*g,hi[j0]*g).sum()-bound[j0])
                          if kind=='peeled' else '')
            for j in range(3):
                for name,arr in [('measured_N',seen_f),('shear_N',seen_t),('target_N',target),
                                 ('next_actual_N',next_f),('proposed_low_N',lo),('proposed_high_N',hi)]:
                    line[f'{name}_{j+1}']=arr[j0,j] if moving[j0] or name not in ('target_N','next_actual_N') else ''
            trace.append(line)
        f,d=next_f,next_d
    reason[reason=='']='complete'
    return dict(outcome=reason,peak_N=peak,peak_width=peak_d,slip_mm=slip,
                force_time_Ns=area,steps=count,coverage=coverage,damage=ever_damage,slip_event=ever_slip),trace
