"""MJ11 clasp bracket search (2026-11-03, SHOULDER / CLASP): bar width, anchor angle psi, anchor radius, overhang, thickness, optional via point, against
   * arm sweep: S1' group (M1), arm (M2), wrist/hand (M3) vs the bracket over the passive box +-5 deg (gate 3 mm; < 1 mm failure).  M1 depends on S1' only, M2 on S1' x S2, M3 on S1' x S2 (S3 = 0),
   * the real wing rail strip and spine-bridge meshes (3 mm),
   * skin (MJ11 rule: rail / blade 5 mm, top 10 mm; the 30 mm seats around F1 / F4 are exempt, as in mj_eval.skin_zones),
   * the other MJ11 parts (dish, liner, rail spacers: > 0, report), pad (designed contact).
The feet positions F5 / F4 / F1 and the ~270 deg clasp coverage are untouched (feet bars 16 x 6 are the same mesh).
usage: J1R2_ROOT=<root> python3 mj11_bracket_search.py grid|one [key=value ...]   -> stl/j1r2_mounts/MJ11/bracket_search.jsonl
Box for the arm sweep: S1' -100..10, S2 -20..22.4 (the 2026-11-02 passive box -95..5 x -15..17.4, expanded by the 5 deg hard stops)."""
import os, sys, json, time, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh, fcl
from scipy.spatial import cKDTree
import clib as L, mj_frame as M, mj_concepts as C, mj_concepts3 as C3, mj11_sweep as W, j1r2_sweep as S
BOX = dict(S1p=(-100.0, 10.0), S2=(-20.0, 22.4))
RULE = dict(rail=5.0, blade=5.0, top=10.0)
def samp(m, sp=1.2):
    p, _ = trimesh.sample.sample_surface_even(m, int(m.area / sp ** 2) + 300); return np.vstack([p, m.vertices]) if len(p) else m.vertices
class Ctx:
    def __init__(s):
        t0 = time.time(); s.parts = C3.mj11_geo(None); s.sc, s.info = W.scene(s.parts); s.F = S.Fcl(s.sc)
        w = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")); s.rail = cKDTree(samp(L.I(w, L.box(-300, 100, -150, -100.0, -500, 100)), 1.2))
        s.bridge = cKDTree(samp(trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "spine_bridge.stl")), 1.2))
        s.TK, s.TP, s.TN = L._tree(); s.seats = np.array([np.asarray(M.SITES[k]["p"], float) for k in ("F1_upper_trap", "F4_acromion")])
        s.others = {k: S._bvh(s.parts[k]["mesh"]) for k in ("pad", "seat_dish", "liner", "rail_spacers")}
        s.grid = {j: np.arange(lo, hi + 1e-6, 2.5) for j, (lo, hi) in BOX.items()}; print("ctx %.1f s" % (time.time() - t0), flush=True)
    def arm(s, br, coarse=False):
        bo = fcl.CollisionObject(S._bvh(br), fcl.Transform()); req = fcl.DistanceRequest(); out = {}
        s1 = s.grid["S1p"][::2] if coarse else s.grid["S1p"]; s2 = s.grid["S2"][::2] if coarse else s.grid["S2"]
        for g in ("M1", "M2", "M3"):
            best = 1e9
            for th in s1:
                for ph in ([s.sc.park["S2"]] if g == "M1" else s2):
                    q = dict(s.sc.park, S1p=float(th), S2=float(ph)); o = s.F.set_group(g, s.sc.pose_fn(g, q)); res = fcl.DistanceResult(); best = min(best, float(fcl.distance(o, bo, req, res)))
            out[g] = round(best, 2)
        return out
    def eval(s, br, coarse=False):
        r = dict(arm=s.arm(br, coarse)); pts = samp(br, 1.2)
        r["rail"] = round(float(s.rail.query(pts)[0].min()), 2); r["bridge"] = round(float(s.bridge.query(pts)[0].min()), 2)
        sp = samp(br, 1.5); d, i = s.TK.query(sp); sd = np.sign(np.einsum("ij,ij->i", sp - s.TP[i], s.TN[i])) * d
        near = np.min(np.linalg.norm(sp[:, None, :] - s.seats[None], axis=2), axis=1) < 30.0
        zone = np.where(near, "seat", np.where(sp[:, 1] <= -100.0, "rail", np.where(sp[:, 0] < -150.0, "top", "blade")))
        r["skin"] = {z: round(float(sd[zone == z].min()), 1) for z in ("rail", "blade", "top", "seat") if (zone == z).any()}
        bo = fcl.CollisionObject(S._bvh(br), fcl.Transform()); r["parts"] = {}
        for k, m in s.others.items():
            res = fcl.DistanceResult(); r["parts"][k] = round(float(fcl.distance(fcl.CollisionObject(m, fcl.Transform()), bo, fcl.DistanceRequest(), res)), 2)
        gate = min(r["arm"].values()); gate = min(gate, r["rail"], r["bridge"]); skin_m = min(v - RULE[z] for z, v in r["skin"].items() if z in RULE)
        r["min_part_gate"] = round(gate, 2); r["min_skin_margin"] = round(skin_m, 1); r["pass"] = bool(gate >= 3.0 and skin_m >= 0.0)
        return r
def cand(**kw): return C3.mj11_bracket(**kw)
if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "grid"; ctx = Ctx(); d = os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11"); os.makedirs(d, exist_ok=True)
    logf = open(os.path.join(d, "bracket_search.jsonl"), "a")
    def run(kw, coarse=False):
        t = time.time(); r = ctx.eval(cand(**kw), coarse); r["kw"] = {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in kw.items()}; r["t"] = round(time.time() - t, 1)
        print(json.dumps(r, default=float), flush=True); logf.write(json.dumps(r, default=float) + "\n"); logf.flush(); return r
    if mode == "one":
        kw = {}
        for a in sys.argv[2:]:
            k, v = a.split("="); kw[k] = np.array([float(x) for x in v.split(",")]) if "," in v else float(v)
        run(kw)
    elif mode == "grid":
        for w, psi, ra, ext in itertools.product((14, 18, 24), (100, 110, 120, 130, 140), (58.0, 62.0), (0.0, 6.0)): run(dict(w=w, psi=psi, ra=ra, ext=ext), coarse=True)
    elif mode == "fine":
        for w, psi, ra, ext, t in itertools.product((20, 24), (116, 118, 120, 122, 124), (60.0, 62.0, 64.0, 66.0), (0.0, 3.0), (6.0, 5.0)): run(dict(w=w, psi=psi, ra=ra, ext=ext, t=t))
