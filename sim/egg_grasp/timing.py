"""Software-only timing sensitivity. The simulated counter records a physical edge.
The edge coincides with sensing only in the matched case. No AX7020 bitstream.
"""
import numpy as np

def timing_study(seed=17005):
    rng=np.random.default_rng(seed)
    hz=50_000_000   # a scenario value, never used as measured cross-device precision
    event=np.arange(180)*.1
    clock=lambda t: .203+(1.+35e-6)*t
    device=clock(event)+rng.normal(0,.00012,len(event))
    ticks=np.rint(hz*(event+rng.normal(0,.00003,len(event))))
    train=np.arange(len(event))<60
    slope,offset=np.polyfit(device[train],ticks[train]/hz,1)
    held=~train
    target=event[held]
    link_latency=rng.uniform(.002,.028,len(target))
    aligned=slope*device[held]+offset
    results=[]
    for name,estimate,ground in [('host_arrival',target+link_latency,target),
                                 ('shared_edge',aligned,target),
                                 ('edge_lags_sample_6ms',aligned,target-.006)]:
        error=1000*(estimate-ground)
        results.append(dict(method=name,n=len(target),mean_bias_ms=error.mean(),
                            abs_p95_ms=np.quantile(np.abs(error),.95)))
    return results
