import sys, itertools, json; import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as H, numpy as np
def seg_dist(p1,q1,p2,q2):
    # min distance between segments (sampled, fine for 3-5 mm accuracy)
    t=np.linspace(0,1,25); A=p1+np.outer(t,q1-p1); B=p2+np.outer(t,q2-p2); return np.min(np.linalg.norm(A[:,None]-B[None],axis=2))
def clearance(ths,psi,r=6.4):
    cs={}
    for nm,th in zip(['index','middle','ring'],ths): cs[nm]=H.chain(nm,th,0 if nm=='middle' else psi)
    segs=lambda c:[(c['M'],c['P1']),(c['P1'],c['P2']),(c['P2'],c['P3'])]
    worst=1e9
    for a,b in [('index','middle'),('middle','ring'),('index','ring')]:
        for (p1,q1) in segs(cs[a]):
            for (p2,q2) in segs(cs[b]): worst=min(worst,seg_dist(p1,q1,p2,q2)-2*r)
    return worst
def quick_solve(rb):
    best=None
    from scipy.optimize import least_squares
    names=['index','middle','ring']
    def res(p):
        r=[]
        for nm,th in zip(names,p[:3]):
            c=H.chain(nm,th,0 if nm=='middle' else p[3]); h=np.array(c['pad'][:2]); nh=np.array(c['n'][:2])
            r.append(np.linalg.norm(h)-rb); r.append(1-np.dot(nh,-h/(np.linalg.norm(h)+1e-9))/(np.linalg.norm(nh)+1e-9))
            sg=np.sign(H.U[nm]); r.append(2.0*max(0.0,-sg*c['pad'][1]+0.5) if sg!=0 else 0.0)
        return r
    for t0 in (4,10):
        for ps in (25,45):
            r=least_squares(res,[t0,t0,t0,ps],bounds=([0,0,0,-5],[H.TH_MAX]*3+[110]))
            if best is None or r.cost<best.cost: best=r
    return best
rows=[]
for tau,pitch,zi,zr in itertools.product((30,45,60),(15.5,19.5),(520,532),(540,550)):
    H.TAU=tau; H.PITCH=pitch; H.U.update(index=pitch,ring=-pitch); H.ZM.update(index=zi,ring=zr,middle=550.0)
    r=quick_solve(5.0)
    if r.cost>1e-3: rows.append((tau,pitch,zi,zr,'nosol',r.cost)); continue
    cl=clearance(r.x[:3],r.x[3]); rows.append((tau,pitch,zi,zr,round(float(r.x[3]),1),round(float(cl),1)))
for r in sorted(rows,key=lambda x:-(x[5] if isinstance(x[5],float) else -99))[:14]: print(r)
