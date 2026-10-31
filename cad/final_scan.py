import sys; import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh, itertools
import hand_kin as H, hand_pose as HP
exec(open('chuck_cad_scan.py').read().split("rows=[]")[0].split("from scipy.optimize import least_squares",1)[1].join(["from scipy.optimize import least_squares",""]) if False else "")
from chuck_cad_scan import solve, inter_volume
for zr,zi in itertools.product((536,537,538,539,540),(498,504)):
    H.ZM.update(ring=float(zr),index=float(zi)); r=solve(5.0); thi,thm,thr,psi=r.x
    scene,chk=[],[]
    for nm,(th,ps) in dict(index=(thi,psi),middle=(thm,0),ring=(thr,psi)).items(): HP.finger_pose(nm,th,ps,scene,chk)
    v,w=inter_volume({n:m for n,m in HP.to_robot(scene)}); print(zr,zi,round(float(psi),1),round(v,2),w[0],round(w[1],2),flush=True)
