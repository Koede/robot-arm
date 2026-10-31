import sys, itertools, json; import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import hand_kin as H, hand_pose as HP
src=open('chuck_scan.py').read().split("rows=[]")[0].replace("+0.5) if sg!=0","+SM) if sg!=0"); SM=-1.0
exec(src.split("import hand_kin as H, numpy as np")[1].replace("\n",";\n",0)) if False else None
from scipy.optimize import least_squares
def solve(rb=5.0):
    names=['index','middle','ring']
    def res(p):
        r=[]
        for nm,th in zip(names,p[:3]):
            c=H.chain(nm,th,0 if nm=='middle' else p[3]); h=np.array(c['pad'][:2]); nh=np.array(c['n'][:2])
            r.append(np.linalg.norm(h)-rb); r.append(1-np.dot(nh,-h/(np.linalg.norm(h)+1e-9))/(np.linalg.norm(nh)+1e-9))
            sg=np.sign(H.U[nm]); r.append(2.0*max(0.0,-sg*c['pad'][1]+SM) if sg!=0 else 0.0)
        return r
    best=None
    for t0 in (4,10):
        for ps in (20,40):
            r=least_squares(res,[t0,t0,t0,ps],bounds=([0,0,0,-5],[H.TH_MAX]*3+[110]))
            if best is None or r.cost<best.cost: best=r
    return best
def inter_volume(meshes):
    tot=0.0; worst=('',0.0)
    names=list(meshes)
    cm=trimesh.collision.CollisionManager()
    for n in names: cm.add_object(n,meshes[n])
    _,pairs=cm.in_collision_internal(return_names=True)
    for a,b in pairs:
        fa,fb=a.split('_')[0],b.split('_')[0]
        if fa==fb: continue
        try: v=meshes[a].intersection(meshes[b],engine='manifold').volume
        except Exception: v=0.0
        tot+=v
        if v>worst[1]: worst=(a+'|'+b,v)
    return tot,worst
rows=[]
if __name__=='__main__':
    for tau,pitch,zi,zr,zm in itertools.product((45,),(21.5,23.0),(500,504),(538,541),(558,)):
        H.TAU=tau; H.PITCH=pitch; H.U.update(index=pitch,ring=-pitch); H.ZM.update(index=zi,ring=zr,middle=float(zm))
        r=solve(5.0)
        if r.cost>0.02: rows.append((tau,pitch,zi,zr,zm,'nosol',round(r.cost,3))); continue
        thi,thm,thr,psi=r.x
        scene,chk=[],[]
        for nm,(th,ps) in dict(index=(thi,psi),middle=(thm,0),ring=(thr,psi)).items(): HP.finger_pose(nm,th,ps,scene,chk)
        meshes={n:m for n,m in HP.to_robot(scene)}
        v,w=inter_volume(meshes); rows.append((tau,pitch,zi,zr,zm,round(float(psi),1),round(v,1),w[0],round(w[1],1)))
        print(rows[-1],flush=True)