"""MJ11 fixed fairing checks (2026-11-03): min distance to every moving group over the S1' x S2 range (passive box +-5 deg hard stops), to skin, rail/wing, spine bridge, spacers, other fixed parts.
usage: J1R2_ROOT=<work root> J1_PAD=-60,-190,12 python3 mj11_fairing_check.py   -> stl/j1r2_mounts/MJ11/fairing_check.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import clib as L, j1r2_geom as G, mj_frame as M, mj_concepts3 as C3
S1_RNG = (-100.0, 10.0); S2_RNG = (-25.0, 40.0)          # S1' box -95..5 (task: -80..5) and S2 box -20..35 (park-limited -15..17.4) plus the 5 deg hard-stop expansion
M1N = ("S1_yoke", "S1_stop_collar", "S2_AK70", "S2_clamp", "S2_sleeve", "S2_stop_posts", "tpu_S2_bumpers")
M2N = ("S2_adapter", "S3_bracket", "S3_XM540", "S2_stop_lug", "arm_tube", "arm_elbow_ball")
def samp(m, sp=1.0):
    p, _ = trimesh.sample.sample_surface(m, int(m.area / sp ** 2) + 300, seed=1); return np.vstack([p, m.vertices]) if len(p) else m.vertices
def run(fair=None, parts=None, step1=2.5, step2=5.0):
    BASE, INFO = G.build(y1=M.Y1, z1=M.Z1, standoff=M.DER["standoff"], full=True); DER = INFO["derived"]
    parts = parts or C3.mj11_geo(None); fair = fair if fair is not None else parts["fairing"]["mesh"]; T = cKDTree(samp(fair, 1.5)); Tf = cKDTree(samp(fair, 0.5)); out = {}
    mv1 = {k: samp(BASE[k]["mesh"], 2.0) for k in M1N}; mv2 = {k: samp(BASE[k]["mesh"], 2.5) for k in M2N}; fine = {k: samp(BASE[k]["mesh"], 0.6) for k in M1N + M2N}; worst = {k: (99.0, 0.0, 0.0) for k in M1N + M2N}
    for th in np.arange(S1_RNG[0], S1_RNG[1] + 0.1, step1):
        Mx = G.pose_matrix("M1", DER, th, 0.0)
        for k, p in mv1.items():
            d = float(T.query(p @ Mx[:3, :3].T + Mx[:3, 3], distance_upper_bound=15.0, workers=-1)[0].min()); 
            if d < worst.get(k, (1e9,))[0]: worst[k] = (d, th, 0.0)
        for ph in np.arange(S2_RNG[0], S2_RNG[1] + 0.1, step2):
            Mm = G.pose_matrix("M2", DER, th, ph)
            for k, p in mv2.items():
                d = float(T.query(p @ Mm[:3, :3].T + Mm[:3, 3], distance_upper_bound=15.0, workers=-1)[0].min())
                if d < worst.get(k, (1e9,))[0]: worst[k] = (d, th, ph)
    for k, v in list(worst.items()):                                 # refine the worst pose of every moving part with dense samples (spacing 0.5-0.6 mm)
        if v[0] < 12.0:
            Mm = G.pose_matrix("M1" if k in M1N else "M2", DER, v[1], v[2]); d = float(Tf.query(fine[k] @ Mm[:3, :3].T + Mm[:3, 3], workers=-1)[0].min()); worst[k] = (d, v[1], v[2])
    out["moving_min_mm"] = {k: dict(d=round(v[0], 2), S1p=v[1], S2=v[2]) for k, v in worst.items()}
    out["moving_min_all_mm"] = round(min(v[0] for v in worst.values()), 2)
    fs = samp(fair, 1.0); sk = L.skin_gap(fair, 6000)[0]; out["skin_min_mm"] = round(float(sk), 1)
    wing = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")); wing = L.I(wing, L.box(-300, 100, -150, -100.0, -500, 100)); br = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "spine_bridge.stl"))
    out["rail_min_mm"] = round(float(cKDTree(samp(wing, 1.0)).query(fs)[0].min()), 2); out["spine_bridge_min_mm"] = round(float(cKDTree(samp(br, 1.0)).query(fs)[0].min()), 2)
    ot = {}
    for k in ("pad", "seat_dish", "liner", "rail_spacers", "clasp_bracket"):
        ot[k] = round(float(cKDTree(samp(parts[k]["mesh"], 1.0)).query(fs)[0].min()), 2)
    for k in ("S1_AK70", "S1_stop_posts", "tpu_S1_bumpers"):
        ot[k] = round(float(cKDTree(samp(BASE[k]["mesh"], 1.0)).query(fs)[0].min()), 2)
    out["fixed_parts_min_mm"] = ot
    return out
if __name__ == "__main__":
    r = run(); print(json.dumps(r, indent=1))
    d = os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11"); os.makedirs(d, exist_ok=True); json.dump(r, open(os.path.join(d, "fairing_check.json"), "w"), indent=1)
