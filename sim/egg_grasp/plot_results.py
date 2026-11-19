"""Plot nominal outcomes of all attempted simulations, and one disclosed failure trace."""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def read(path):
    with path.open(encoding='utf-8') as stream:
        return list(csv.DictReader(stream))

def draw(directory):
    rows=[r for r in read(directory/'summary.csv')
          if r['method'] in ('A','B') and r['stress']=='nominal']
    groups=[f"{r['egg'].title()} / {r['motion']} / {r['method']}" for r in rows]
    y=np.arange(len(rows))
    fig,ax=plt.subplots(figsize=(9.2,4.5),layout='constrained')
    colors=['#146656','#c06430','#be9756','#536f91','#8c6a91','#777777','#bbbbbb']
    names=['complete','damage_proxy','slip_limit','box_abort','width_abort','force_guard','stale_view']
    labels=['Completed','Damage proxy','Slip limit','Force box infeasible','Width infeasible',
            'Force stop','Stale observation']
    left=np.zeros(len(rows))
    for name,color,label in zip(names,colors,labels):
        width=np.array([100*int(r[name])/int(r['n']) for r in rows])
        if width.any():
            ax.barh(y,width,left=left,label=label,color=color,height=.72)
        left+=width
    ax.set(yticks=y,yticklabels=groups,xlim=(0,100),xlabel='All initiated simulations (%)',
           title='Three-contact model | nominal conditions')
    ax.invert_yaxis()
    ax.spines[['right','top']].set_visible(False)
    ax.legend(loc='upper center',bbox_to_anchor=(.43,-.18),ncol=3,frameon=False)
    fig.savefig(directory/'nominal_outcomes.png',dpi=170)
    plt.close(fig)
    path=directory/'first_width_conflict.csv'
    if not path.is_file():
        return
    trace=read(path)
    t=[float(r['t_s']) for r in trace]
    gap=[100*float(r['width_min_minus_bound']) for r in trace]
    fig,ax=plt.subplots(figsize=(7.2,3.5),layout='constrained')
    ax.axhline(0,color='#555555',ls='--',lw=1)
    ax.plot(t,gap,color='#375979',marker='o',ms=3)
    ax.scatter(t[-1],gap[-1],color='#ae452c',zorder=3)
    ax.set(xlabel='Time from preload (s)',ylabel='Minimum predicted excess (pp)',
           title=f"First width conflict | peeled / gentle / B | specimen {trace[0]['id']}")
    ax.spines[['right','top']].set_visible(False)
    fig.savefig(directory/'width_conflict.png',dpi=170)
    plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=Path('results'))
    draw(p.parse_args().out)
