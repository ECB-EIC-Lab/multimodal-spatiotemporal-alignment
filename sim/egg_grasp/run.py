"""Reproduce the offline study. No device interface or network access is used."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from contact import POLICY, simulate, deformation_fit
from motion import check_path, DT
from vision import camera_study
from timing import timing_study

KINDS=('raw','peeled')
MODES=('gentle','brisk')
OUTCOMES=('complete','damage_proxy','slip_limit','box_abort','width_abort','stale_view','force_guard')

def write_csv(path, rows):
    with path.open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n')
        w.writeheader()
        for row in rows:
            w.writerow({k:format(v,'.9g') if isinstance(v,(float,np.floating)) else v
                        for k,v in row.items()})

def wilson(ok,n):
    p, z=ok/n,1.95996398454
    mid=(p+z*z/(2*n))/(1+z*z/n)
    gap=z*np.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n)
    return max(0.,mid-gap),min(1.,mid+gap)

def tune(kind,fit,kind_id):
    """Freeze A and the shear policy using equal calibration samples on both motions."""
    settings={}
    records=[]
    for m,grid in [('A',[.46,.55,.64,.73,.82,.91]),('shear',[.64,.72,.80,.88,.96,1.04])]:
        scored=[]
        for value in grid:
            successes,damage=0,0
            for j,mode in enumerate(MODES):
                x,_=simulate(kind,mode,m,20010+100*kind_id+j,48,fit,
                             fixed=value,ratio=value)
                successes+=int((x['outcome']=='complete').sum())
                damage+=int(x['damage'].sum())
            records.append(dict(kind=kind,method=m,value=value,n=96,complete=successes,damage=damage))
            scored.append((-successes, damage,value))
        settings[m]=min(scored)[2]
    return settings, records

def study(out, n=100):
    if n<2: raise ValueError('At least two simulated specimens per cell are required.')
    out.mkdir(exist_ok=True,parents=True)
    fit=deformation_fit()
    vision=camera_study()
    # Sensitivity model: match the corrected side-view P95 magnitude with Gaussian noise.
    stress_sigma=vision[1]['abs_error_p95_fraction']/1.95996398454
    settings,grid={},[]
    for i,kind in enumerate(KINDS):
        settings[kind],rows=tune(kind,fit,i)
        grid.extend(rows)
    (out/'calibration.json').write_text(json.dumps(dict(fit=fit,settings=settings,
            test_n_per_cell=n, side_view_stress_sigma=stress_sigma),indent=2)+'\n',encoding='utf-8')
    write_csv(out/'calibration_grid.csv',grid)
    summary, paired, trials,traces,conflict=[],[],[],[],[]
    for i,kind in enumerate(KINDS):
        for j,mode in enumerate(MODES):
            seed=51000+100*i+j
            for stress in ('nominal','low_friction','stale_view','camera_noise'):
                if stress in ('stale_view','camera_noise') and kind=='raw': continue
                methods=['A','B']
                if stress=='nominal':
                    methods+=['shear']
                    if kind=='peeled': methods+=['B_noD']
                results={}
                for m in methods:
                    x,lines=simulate(kind,mode,m,seed,n,fit,fixed=settings[kind]['A'],
                              ratio=settings[kind]['shear'],stress=stress,
                              width_sigma=stress_sigma if stress=='camera_noise' else .0015,
                              capture=(stress=='nominal' and m in ('A','B')))
                    group=dict(egg=kind,motion=mode,stress=stress,method=m)
                    success=x['outcome']=='complete'
                    results[m]=success
                    freq={o:int((x['outcome']==o).sum()) for o in OUTCOMES}
                    low,high=wilson(freq['complete'],n)
                    cov=x['coverage'].sum(axis=0)
                    summary.append(dict(**group,n=n,**freq,success_rate=success.mean(),lo95=low,hi95=high,
                          any_damage=x['damage'].sum(),any_slip=x['slip_event'].sum(),
                          median_peak_N=np.median(x['peak_N']),
                          median_observed_s=DT*np.median(x['steps']),
                          cycles=cov[0],track_cov=cov[1]/cov[0],shear_cov=cov[2]/cov[0],
                          width_cov=cov[3]/cov[0] if kind=='peeled' else '',
                          mu_cov=cov[4]/cov[0],joint_force_assumption_cov=cov[5]/cov[0]))
                    if (kind,mode,stress,m)==('peeled','gentle','nominal','B'):
                        ids=np.flatnonzero(x['outcome']=='width_abort')
                        if len(ids):
                            idx=int(ids[0])
                            _,example=simulate(kind,mode,m,seed,n,fit,capture=True,example_id=idx)
                            conflict.extend(dict(**group,seed=seed,id=idx,**line) for line in example)
                    for idx in range(n):
                        trials.append(dict(**group,seed=seed,id=idx,outcome=x['outcome'][idx],
                                damage=int(x['damage'][idx]),slip=int(x['slip_event'][idx]),
                                time_s=DT*x['steps'][idx],max_finger_N=x['peak_N'][idx],
                                max_width_loss=x['peak_width'][idx] if kind=='peeled' else '',
                                slip_mm=x['slip_mm'][idx],force_time_Ns=x['force_time_Ns'][idx]))
                    traces.extend(dict(**group,seed=seed,id=0,**line) for line in lines)
                for other in results:
                    if other=='B': continue
                    diff=results['B'].astype(float)-results[other]
                    rng=np.random.default_rng(seed+73000)
                    bootstrap=diff[rng.integers(0,n,(3000,n))].mean(axis=1)
                    paired.append(dict(egg=kind,motion=mode,stress=stress,contrast='B-'+other,n=n,
                        win=int((diff>0).sum()),loss=int((diff<0).sum()),tie=int((diff==0).sum()),
                        risk_difference=diff.mean(),lo95=np.quantile(bootstrap,.025),
                        hi95=np.quantile(bootstrap,.975)))
    for name,rows in [('summary',summary),('paired',paired),('trials',trials),('trace_id0',traces),
                      ('kinematics',sum((check_path(m) for m in MODES),[])),
                      ('vision',vision),('timing',timing_study())]:
        write_csv(out/(name+'.csv'),rows)
    if conflict:
        write_csv(out/'first_width_conflict.csv',conflict)
    elif (out/'first_width_conflict.csv').exists():
        (out/'first_width_conflict.csv').unlink()
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=Path('results'))
    p.add_argument('--n',type=int,default=100,help='synthetic specimens per object and motion')
    args=p.parse_args()
    for r in study(args.out,args.n):
        if r['stress']=='nominal':
            print(r['egg'],r['motion'],r['method'],str(r['complete'])+'/'+str(r['n']))
