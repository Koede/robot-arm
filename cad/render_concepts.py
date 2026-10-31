"""3D views of the compliant-wrist concepts A..F (concepts/*/ *.stl) with the arm context (W3 servo, W2 yoke, carrier plate, hand floor)."""
import sys, os, glob, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_hand as RH
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."); ST = os.path.join(ROOT, "stl")
def ctx():
    out = []
    for n, p, c in [("w2_yoke", "distal/w2_yoke_R.stl", (150, 165, 190)), ("sts_w3", "distal/proxies/sts_w3_R.stl", (70, 70, 78)), ("palm", "wrist_rotary/palm_tray_wrist.stl", (205, 208, 218))]:
        m = trimesh.load(os.path.join(ST, p)); out.append((n, m, c))
    return out
PAL = [(240, 140, 40), (30, 170, 160), (210, 190, 40), (190, 80, 120), (110, 150, 230), (140, 200, 90), (230, 110, 90)]
def load_concept(d):
    ms = []; i = 0
    for f in sorted(glob.glob(os.path.join(d, "*.stl"))):
        try: m = trimesh.load(f)
        except Exception: continue
        if isinstance(m, trimesh.Scene): m = trimesh.util.concatenate(list(m.geometry.values()))
        ms.append((os.path.basename(f)[:-4], m, PAL[i % len(PAL)])); i += 1
    return ms
def prep(items):
    out = []
    for n, m, c in items:
        m = m.copy()
        if len(m.faces) > 2500:
            try: m = m.simplify_quadric_decimation(face_count=2500)
            except Exception: pass
        out.append((n, m, np.tile(np.array([*c, 255.0]), (len(m.faces), 1))))
    return out
NAMES = {"A": "A  slotted TPU core + 2 tendons", "B": "B  coil-spring core + tendons on curved rim plates", "C": "C  monolithic printed cross-strip flexure",
         "D": "D  helical-slit compliant roll core", "E": "E  Kresling bellows coupler", "F": "F  three tube legs (3-SPS parallel stage)"}
for d in sorted(glob.glob(os.path.join(ROOT, "concepts", "?_*"))):
    k = os.path.basename(d)[0]; items = load_concept(d)
    if not items: print("no stl", d); continue
    allm = prep(ctx() + items); pts = np.vstack([m.vertices for _, m, _ in allm[3:]]); c = pts.mean(0); r = max(60.0, 0.8 * np.ptp(pts, axis=0).max() + 20)
    fig = plt.figure(figsize=(15, 6.5), dpi=110)
    for j, (el, az, t) in enumerate([(24, -52, "3/4 view"), (4, -90, "side (looking along x)")]):
        ax = fig.add_subplot(1, 2, j + 1, projection="3d"); RH.draw(ax, allm, el, az, t, dict(c=c, r=r))
    fig.suptitle(NAMES.get(k, k) + "  (grey = W2 yoke, W3 servo, hand floor for context)", fontsize=12); fig.tight_layout()
    fig.savefig(os.path.join(ROOT, f"concept_{k}_3d.png")); plt.close(fig); print("ok", k, len(items))
