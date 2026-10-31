import sys,json; import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as H, numpy as np
from scipy.optimize import differential_evolution
RHO=H.RHO_T; out={'RHO':RHO}
def solve(tgt,n,wn,seed=2,lift_max=100):
    cost=lambda p: float(np.sum((H.thumb3(p[0],p[1],RHO,p[2])['pad']-tgt)**2)+wn*np.sum((H.thumb3(p[0],p[1],RHO,p[2])['n']-n)**2))
    return differential_evolution(cost,[(-10,100),(-10,lift_max),(0,75)],seed=seed,tol=1e-10,maxiter=300,popsize=30,polish=True)
for nm,tg in [('chuck3',[3,0,575]),('chuck5',[5,0,575]),('chuck8',[8,0,575])]:
    r=solve(np.array(tg,float),np.array([-1.,0,0]),400); c=H.thumb3(r.x[0],r.x[1],RHO,r.x[2]); print(nm,np.round(r.x,1),'err',np.round(c['pad']-np.array(tg),2),'n',np.round(c['n'],2)); out[nm]=list(map(float,r.x))
cr=H.chain('ring',92,0); tg=0.5*cr['P1']+0.5*cr['P2']+np.array([17.0,0,0])
r=solve(tg,np.array([-1.,0,0]),3,lift_max=72); c=H.thumb3(r.x[0],r.x[1],RHO,r.x[2]); print('tuck',np.round(r.x,1),'err',np.round(c['pad']-tg,2)); out['tuck']=list(map(float,r.x))
json.dump(out,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','stl','hand','thumb3.json'),'w'))
