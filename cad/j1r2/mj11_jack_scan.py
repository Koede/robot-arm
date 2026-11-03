"""Where on the MJ11 dish rim can the jack plate sit?  skin gap (>= 5 mm rule) and clearance to the moving S1'/S2 stage groups over the hard ranges (target >= 3 mm).
usage: J1R2_ROOT=<root> python3 mj11_jack_scan.py  -> stl/j1r2_mounts/MJ11/jack_scan.json"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import mj_eval as E, mj_frame as M, mj_concepts3 as K, mj_concepts as C
from mj_eval import samp, TK, TP, TN, DER, LIM, MOV1, MOV2, G, HARN
rows = []
poses = []
for th in np.arange(LIM["S1p"][0], LIM["S1p"][1] + 0.1, 5.0):
    Mx = G.pose_matrix("M1", DER, th, 0.0); poses.append(MOV1 @ Mx[:3, :3].T + Mx[:3, 3])
    for ph in np.arange(LIM["S2"][0], LIM["S2"][1] + 0.1, 10.0):
        if th <= -85.0 and ph > 50.0: continue
        Mm = G.pose_matrix("M2", DER, th, ph); poses.append(MOV2 @ Mm[:3, :3].T + Mm[:3, 3])
ALL = np.vstack(poses); TT = cKDTree(ALL)
for t_, JW in ((5.0, 38.0), (5.0, 26.0)):
  for R0, R1 in ((62.0, 74.0),):
    for psi in sorted(set(range(0, 361, 10)) | set(range(180, 281, 5))):
        a = math.radians(psi); u = np.array([math.cos(a), math.sin(a)]); n = np.array([-u[1], u[0]])
        K.JACK_PSI = float(psi); K.JACK_R0, K.JACK_R1 = R0, R1
        pl = K.boxbeam(np.array([K.X0 - t_ / 2 - 1.0, K.Y1 + R0 * u[0], K.Z1 + R0 * u[1]]), np.array([K.X0 - t_ / 2 - 1.0, K.Y1 + R1 * u[0], K.Z1 + R1 * u[1]]), JW, t_, None, up=(1, 0, 0), ext=0.0)
        s_ = samp(pl, 1.5); d, i = TK.query(s_); sd = np.sign(np.einsum("ij,ij->i", s_ - TP[i], TN[i])) * d
        st = float(TT.query(s_, distance_upper_bound=40)[0].min())
        hc = {h: round(float(T.query(s_, distance_upper_bound=30)[0].min()), 1) for h, T in HARN.items()}
        rows.append(dict(t=t_, W=JW, R0=R0, R1=R1, psi=psi, rail=hc['wing_rail'], bridge=hc['spine_bridge'], skin_gap=round(float(sd.min()), 1), stage_clear=round(min(st, 40.0), 1)))
        print(rows[-1], flush=True)
json.dump(rows, open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "jack_scan.json"), "w"), indent=0)
