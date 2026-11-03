"""Renders (numpy painter's-algorithm, matplotlib) of the MJ11 whole-arm scene at park and at first-contact poses from the sweep report + 2-D sweep maps.
usage: J1R2_ROOT=<work root> python3 mj11_whole_arm_render.py -> renders_j1r2/mj11_whole_arm/*.png"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import j1r2_params as P, mj_frame as M, mj_concepts3 as C3, mj11_whole_arm as W
REPO = M.REPO; OUTD = os.path.join(REPO, "renders_j1r2", "mj11_whole_arm"); os.makedirs(OUTD, exist_ok=True)
STL = os.path.join(REPO, "stl", "j1r2_sweep"); REP = json.load(open(os.path.join(STL, "mj11_whole_arm_report.json")))
COL = dict(wearer_skin=(0.93, 0.80, 0.70), wearer_head=(0.80, 0.55, 0.45), wing=(0.95, 0.60, 0.20), rail=(0.55, 0.55, 0.60), S1_AK70=(0.35, 0.35, 0.40), S1_stop_posts=(0.5, 0.5, 0.5), tpu_S1_bumpers=(0.2, 0.2, 0.2),
           mj11_pad=(0.20, 0.55, 0.85), mj11_seat_dish=(0.25, 0.70, 0.60), mj11_liner=(0.85, 0.85, 0.30), mj11_rail_spacers=(0.6, 0.6, 0.7), mj11_clasp_bracket=(0.15, 0.35, 0.95),
           M1=(0.85, 0.25, 0.25), M2=(0.80, 0.45, 0.15), M3=(0.20, 0.65, 0.30), ME=(0.55, 0.30, 0.75), MW1=(0.25, 0.60, 0.80), MW2=(0.90, 0.30, 0.60), MW3=(0.95, 0.80, 0.15))
VIEWS = {"side": (np.array([0, -1, 0.]), np.array([0, 0, 1.]), np.array([0, 1.0, 0])), "back": (np.array([1, 0, 0.]), np.array([0, 0, 1.]), np.array([-1.0, 0, 0])), "top": (np.array([0, 0, -1.]), np.array([1, 0, 0.]), np.array([0, 0, 1.0]))}
def basis(v):
    fwd, up, cam = VIEWS[v]; fwd = fwd / np.linalg.norm(fwd); right = np.cross(fwd, up); right /= np.linalg.norm(right); up2 = np.cross(right, fwd); return right, up2, fwd
def project(ax, items, view, lim, light=np.array([0.3, -0.5, 0.8])):
    r, u, f = basis(view); polys, cols, depth = [], [], []
    for m, c in items:
        tri = m.triangles; n = m.face_normals; sh = 0.45 + 0.55 * np.abs(n @ (-f * 0.7 + u * 0.5 + r * 0.3) / 1.0).clip(0, 1)
        xy = np.stack([tri @ r, tri @ u], -1); z = (tri @ f).mean(1); vis = (n @ f) < 0.15
        polys += list(xy[vis]); depth += list(z[vis]); cols += [tuple(np.clip(np.array(c) * s, 0, 1)) + (1.0,) for s in sh[vis]]
    o = np.argsort(depth)[::-1] if True else None; o = np.argsort(depth)  # far first: larger f-component = farther along view direction
    pc = PolyCollection([polys[i] for i in o], facecolors=[cols[i] for i in o], edgecolors="none", linewidths=0); ax.add_collection(pc); ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_aspect("equal"); ax.axis("off")
def meshes_at(sc, q, wearer=True):
    items = []
    for k, m in sc.fixed.items(): items.append((m, COL.get(k, (0.6, 0.6, 0.6))))
    for g, d in sc.groups.items(): mm = d["mesh"].copy(); mm.apply_transform(sc.pose_fn(g, q)); items.append((mm, COL[g]))
    if wearer:
        for k, m in sc.wearer.items():
            mm = m.copy()
            if len(mm.faces) > 9000: mm = mm.simplify_quadric_decimation(face_count=9000)
            items.append((mm, COL["wearer_" + k]))
    return items
def sheet(sc, poses, fname, views=("side", "back", "top"), wearer=True):
    fig, axs = plt.subplots(len(poses), len(views), figsize=(5.2 * len(views), 5.6 * len(poses)), dpi=110, squeeze=False); fig.patch.set_facecolor("white")
    lims = {"side": ((-600, 200), (-260, 700)), "back": ((-330, 330), (-260, 700)), "top": ((-330, 330), (-330, 260))}
    for i, (title, q) in enumerate(poses):
        items = meshes_at(sc, q, wearer)
        for j, v in enumerate(views):
            r, u, f = basis(v); L_ = lims[v]
            # fixed view windows in screen coordinates
            if v == "side": lim = ((-300, 450), (-300, 700))
            elif v == "back": lim = ((-300, 300), (-300, 700))
            else: lim = ((-300, 300), (-420, 260))
            project(axs[i][j], items, v, lim); axs[i][j].set_title(f"{v}", fontsize=9)
        axs[i][0].text(0.0, 1.06, title, transform=axs[i][0].transAxes, fontsize=10, weight="bold", va="bottom")
    fig.tight_layout(); fig.savefig(os.path.join(OUTD, fname)); plt.close(fig)
def maps():
    S1 = np.array(REP["shoulder_ok_map_S3all"]["S1p"]); S2 = np.array(REP["shoulder_ok_map_S3all"]["S2"]); ok = np.array(REP["shoulder_ok_map_S3all"]["ok"])
    fig, axs = plt.subplots(1, 3, figsize=(18, 5.6), dpi=110)
    ax = axs[0]; ax.pcolormesh(S2, S1, ok, shading="nearest", cmap="Greens", vmin=-0.3, vmax=1.3, edgecolors="w", linewidth=0.5)
    b = REP["passive_box_10deg_grid_allS3"]["ranges"] if REP.get("passive_box_10deg_grid_allS3") else None
    if b: ax.add_patch(plt.Rectangle((b["S2"][0], b["S1p"][0]), b["S2"][1] - b["S2"][0], b["S1p"][1] - b["S1p"][0], fill=False, ec="b", lw=2.5, label="passive box (10° grid)"))
    ax.plot(P.PARK["S2"], P.PARK["S1p"], "k*", ms=14, label="park"); ax.set_xlabel("S2 (deg)"); ax.set_ylabel("S1' (deg)"); ax.set_title("Shoulder groups M1-M3: gate-free for every S3 (green)"); ax.legend(loc="lower left", fontsize=8)
    for key, c, mk in (("M2|wearer_skin", "r", "o"), ("M3|wearer_skin", "m", "s"), ("M3|mj11_clasp_bracket", "b", "^"), ("M3|wearer_head", "k", "D"), ("M1|M3", "orange", "v")):
        r = next((x for x in REP["shoulder_level_contacts"] if x["pair"] == key), None)
        if r and "example" in r: ax.plot(r["example"]["S2"], r["example"]["S1p"], mk, color=c, ms=8, label=key)
    ax.legend(loc="upper right", fontsize=7); ax.set_ylim(-100, 40)
    for ax, nm, tab in ((axs[1], "FULL tier: max E (deg), -1 = none", REP["tier_FULL"]["Emax"]), (axs[2], "RESTRICT tier: max E (deg), -1 = none", (REP.get("tier_RESTRICT") or {}).get("Emax"))):
        if tab is None: continue
        t = np.array(tab); im = ax.pcolormesh(S2, S1, t, shading="nearest", cmap="viridis", vmin=-1, vmax=145, edgecolors="w", linewidth=0.5)
        for a in range(t.shape[0]):
            for b_ in range(t.shape[1]): ax.text(S2[b_], S1[a], f"{t[a, b_]:.0f}", ha="center", va="center", fontsize=6, color="w" if t[a, b_] < 90 else "k")
        ax.set_xlabel("S2 (deg)"); ax.set_ylabel("S1' (deg)"); ax.set_title(nm); ax.plot(P.PARK["S2"], P.PARK["S1p"], "r*", ms=12)
    fig.tight_layout(); fig.savefig(os.path.join(OUTD, "sweep_maps.png")); plt.close(fig)
    pg = REP["per_group"]; fig, ax = plt.subplots(figsize=(9, 4.4), dpi=110); gs = list(pg); bot = np.zeros(len(gs))
    for cat, c in (("fixed", "#4477aa"), ("wearer_skin", "#ee8866"), ("wearer_head", "#aa3377"), ("group", "#999999")):
        v = np.array([pg[g]["by_partner"][cat] for g in gs], float); ax.bar(gs, v, bottom=bot, label=cat, color=c); bot += v
    ax.set_ylabel("first-contact frontier cells (7-D grid, partner overlaps counted per category)"); ax.set_title("Frontier cells by moving group and partner category"); ax.legend(); fig.tight_layout(); fig.savefig(os.path.join(OUTD, "frontier_by_group.png")); plt.close(fig)
def main():
    maps(); sc, info = W.scene(C3.mj11_geo(None)); ex = lambda k: next(x["example"] for x in REP["shoulder_level_contacts"] if x["pair"] == k)
    park = dict(P.PARK); poses = [("PARK  S1' -18.2 / S2 17.4 / E 45 (HAND rev B/C envelope proxy)", park)]
    for k, ttl in (("M3|wearer_head", "first contact upper arm vs head/neck gate (60 mm): "), ("M3|mj11_clasp_bracket", "first contact upper arm vs clasp bracket: "), ("M2|wearer_skin", "first contact S2 group vs skin: ")):
        q = {j: ex(k)[j] for j in ex(k)}; poses.append((ttl + ", ".join(f"{j} {q[j]:.0f}" for j in ("S1p", "S2", "S3")) , q))
    sheet(sc, poses[:1], "park.png"); sheet(sc, poses[1:], "shoulder_first_contacts.png")
    # one distal example: the worst hand-vs-head first contact
    r = next((x for x in REP["pairs_with_frontier"] if x["pair"] == "MW3|wearer_head"), None)
    if r: q = r["example"]; sheet(sc, [("hand vs head gate: " + ", ".join(f"{j} {q[j]:.0f}" for j in q), q)], "hand_head_first_contact.png")
if __name__ == "__main__": main()
