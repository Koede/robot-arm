import os,sys,json,numpy as np
sys.path.insert(0,'/home/claude/robot-arm/cad/j1r2')
import mj11_sweep as W, mj_concepts3 as C3, j1r2_opt as O
parts=C3.mj11_geo(None)
r,out=W.run(parts,{"S1p":5.0,"S2":5.0,"S3":[0.0]})
ok,vals,worst=O.allowed_map(out)
reach=out["_internal"]["reach"]; reach=reach.reshape(reach.shape[0],reach.shape[1],-1).any(axis=2)
print(r['reach_cov'],r['box'],ok.shape,reach.shape,{k:len(v) for k,v in vals.items()})
np.savez('/tmp/claude-0/ts/free.npz',ok=ok,reach=reach,S1=vals['S1p'],S2=vals['S2'])
print('reach cells',reach.sum(),'of',reach.size)
for i,t in enumerate(vals['S1p']):
    row=''.join('#' if reach[i,j] else '.' for j in range(len(vals['S2'])))
    print('%6.1f'%t,row)
print('S2:',vals['S2'][0],'..',vals['S2'][-1])
