"""Renders of the J1 mount concepts (cad/render_three.py, three.js).  usage: J1R2_ROOT=... python3 mj_render.py [CID,..]  -> renders_j1r2/mounts/<CID>_*.png + overview."""
import os, sys, json, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import clib as L, j1r2_params as P, j1r2_geom as G, mj_frame as M, mj_concepts3   # noqa: F401
REPO = M.REPO; ST = os.path.join(REPO, "stl"); MD = os.path.join(ST, "j1r2_mounts"); OUTD = os.path.join(REPO, "renders_j1r2", "mounts"); os.makedirs(OUTD, exist_ok=True)
TAG = os.environ.get("J1_PAD", "default").replace(",", "_"); CACHE = os.path.join(MD, "_render_cache" + ("" if TAG == "default" else "_" + TAG)); os.makedirs(CACHE, exist_ok=True)
SKIP = ("pad_riser", "standoffs", "fairing_lap", "rail") + (() if TAG == "default" else ("pad", "fairing_fixed", "column_anchor"))      # MJ11: own pad and own fairing (mj11_fairing, from the concept parts); the J1 rev 2 riser-lap fairing_fixed stays out
BASE, INFO = G.build(y1=M.Y1, z1=M.Z1, standoff=M.DER["standoff"], full=True); DER = INFO["derived"]; th, ph = P.PARK["S1p"], P.PARK["S2"]
def rgb(c): return [int(255 * v) for v in c]
def shared():
    if os.path.exists(os.path.join(CACHE, "shared.json")): return json.load(open(os.path.join(CACHE, "shared.json")))
    parts = []
    b = L.body(); c = b.triangles_center; keep = (c[:, 2] > -420) & (c[:, 2] < 150) & (c[:, 1] > -330) & (c[:, 1] < 170) & (c[:, 0] > -340)
    p_ = os.path.join(CACHE, "body.stl"); b.submesh([np.where(keep)[0]], append=True).export(p_); parts.append(dict(stl=p_, color=[214, 182, 160], rough=0.8))
    w = trimesh.load(os.path.join(ST, "shoulder", "scapular", "wing_R.stl")); w = L.I(w, L.box(-300, 100, -150, -100.0, -500, 100)); p_ = os.path.join(CACHE, "wing_rail.stl"); w.export(p_); parts.append(dict(stl=p_, color=[70, 74, 82], rough=0.6))
    for f, col in (("scapular/spine_bridge.stl", [70, 74, 82]), ("clasp_frame_R.stl", [62, 66, 74])): parts.append(dict(stl=os.path.join(ST, "shoulder", f), color=col, rough=0.6))
    for k, v in BASE.items():
        if k in SKIP or v.get("kind") == "cable": continue
        m = v["mesh"].copy(); m.apply_transform(G.pose_matrix(v["group"], DER, th, ph)); p_ = os.path.join(CACHE, f"stage_{k}.stl"); m.export(p_)
        col = [52, 54, 60] if v.get("kind") == "bought" else ([150, 158, 172] if k == "fairing_fixed" else [180, 184, 190])
        parts.append(dict(stl=p_, color=col, metal=0.25 if v.get("kind") == "bought" else 0.0, rough=0.5))
    json.dump(parts, open(os.path.join(CACHE, "shared.json"), "w")); return parts
def scene(cid):
    met = json.load(open(os.path.join(MD, cid, "metrics.json"))); cols = met.get("colours", {}); parts = list(shared())
    for k, r in met["parts"].items():
        p_ = os.path.join(MD, cid, f"{cid}_{k}_R.stl"); c = cols.get(k, [0.2, 0.34, 0.52]); mat = r["mat"]
        parts.append(dict(stl=p_, color=rgb(c), metal=0.6 if mat in ("AL", "STEEL") else (0.15 if mat == "CF" else 0.05), rough=0.35 if mat in ("AL", "CF") else 0.5))
    # the radial balancer tube (J1 rev 2 unresolved placeholder) is NOT drawn: it is not a design, only the unsolved problem (see metrics balancer.verdict)
    title = f"{cid}: {met['title']}" + ("  (balancer: no home yet, not shown)" if met["balancer"].get("home", "").startswith("radial") else "")
    sc = dict(out_prefix=os.path.join(OUTD, cid), title=title, width=820, height=820, parts=parts, up=[0, 0, 1], time_budget=int(os.environ.get("TB", 50)), fill=0.86,
              views=[dict(name="rear34", dir=[1.0, 0.70, 0.55], label="rear-right 3/4"), dict(name="rear", dir=[1.0, -0.12, 0.18], label="from behind"),
                     dict(name="side", dir=[0.15, 1.0, 0.10], label="from the right"), dict(name="top", dir=[0.30, 0.10, 1.0], label="from above")])
    js = os.path.join(CACHE, f"scene_{cid}.json"); json.dump(sc, open(js, "w"))
    subprocess.run([sys.executable, os.path.join(REPO, "cad", "render_three.py"), js], check=True)
if __name__ == "__main__":
    ids = sys.argv[1].split(",") if len(sys.argv) > 1 else ["REF", "MJ1", "MJ2", "MJ3", "MJ4", "MJ5", "MJ6"]
    for c in ids: scene(c)
