"""Render the arm with the seven cable loops (park pose) and the soft-boot envelope (hull of the loop inflated by OD/2 + 3 mm). Output ../boots_view.png, ../boots_mass.json"""
import os, sys, json, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_wiring_lib as L
PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50); F0 = ak.frames(PARK)
d = json.load(open("../stl/electronics/arm_chain_loops.json")); G = ak.groups(include_fingers=True, g_pct=50.0)
M = []
for g, dd in G.items():
    for n, m in dd.items(): mm = m.copy(); mm.apply_transform(F0[g]); M.append(mm)
loops = {}; hulls = {}
for j, c in d["joints"].items():
    gA, gB = d["frm"][j], d["to"][j]; od = d["od"][j]
    Pa = (F0[gA] @ np.r_[c["A"], 1])[:3]; Pb = (F0[gB] @ np.r_[c["B"], 1])[:3]; ta = F0[gA][:3, :3] @ np.array(c["tA"]); tb = F0[gB][:3, :3] @ np.array(c["tB"])
    x, info = L.rod_relax(Pa, ta, Pb, tb, c["L"], F0, od / 2, 0.8); loops[j] = np.asarray(x)
    rr = od / 2 + 3.0; u = trimesh.creation.icosphere(1, rr).vertices
    pts = np.vstack([p + u for p in loops[j][::2]]); hulls[j] = trimesh.convex.convex_hull(pts)
    print(j, "loop pts", len(x), "min clr %.1f" % info["clr"], "hull vol cm3 %.1f" % (hulls[j].volume / 1000), flush=True)
fig, axs = plt.subplots(1, 2, figsize=(15, 8)); col = "#e08214"
B = np.vstack([m.vertices for m in M])
for ax_, (u, v, nm) in zip(axs, [(0, 2, "side (x-z)"), (1, 2, "back (y-z)")]):
    for mm in M:
        f = mm.faces[::3]; ax_.add_collection(PolyCollection(mm.vertices[f][:, :, [u, v]], facecolor="#c9ced6", edgecolor="none", alpha=0.6))
    for j, h in hulls.items():
        f = h.faces; ax_.add_collection(PolyCollection(h.vertices[f][:, :, [u, v]], facecolor=col, edgecolor="none", alpha=0.35))
        ax_.plot(loops[j][:, u], loops[j][:, v], color="#b2182b", lw=2); c = loops[j].mean(0); ax_.text(c[u] + 8, c[v] + 8, j, fontsize=10, weight="bold")
    ax_.set_xlim(B[:, u].min() - 40, B[:, u].max() + 40); ax_.set_ylim(B[:, 2].min() - 30, B[:, 2].max() + 30); ax_.set_aspect("equal"); ax_.set_title(nm)
fig.suptitle("Free cable loops (red) under soft boots (orange envelope), park pose: S1 45 / S2 25 / E 45")
plt.tight_layout(); plt.savefig("../boots_view.png", dpi=110)
json.dump({j: dict(hull_cm3=round(h.volume / 1000, 1), area_cm2=round(h.area / 100, 1)) for j, h in hulls.items()}, open("../boots_mass.json", "w"), indent=1)
print({j: round(h.area / 100, 1) for j, h in hulls.items()})
