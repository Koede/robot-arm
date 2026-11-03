"""MJ11 fixed fairing checks (2026-11-03): clearances to every moving group over the hard ranges, to fixed parts / rail / spine bridge / skin, lead channels + gland open, printability, mass.
usage: J1R2_ROOT=<work root> python3 mj11_fairing_check.py  -> stl/j1r2_mounts/MJ11/fairing_check.json (MJ11 fairing from mj_concepts3.mj11_geo)"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import clib as L, j1r2_geom as G, mj_frame as M, mj_concepts3 as C3, mj_eval as E
parts = C3.mj11_geo(None); parts_single = C3.fairing_mesh((C3.feet_xyz()['F1_upper_trap'], C3.pad_pt(C3.CLASP_PSI, C3.PAD_R - 8, x=C3.XB - 3.0))); fm = trimesh.util.concatenate([parts["fairing_ring"]["mesh"], parts["fairing_hood"]["mesh"]]); fs = E.samp(fm, 1.0); T = cKDTree(fs); out = {}
out["watertight"] = bool(parts["fairing_ring"]["mesh"].is_watertight and parts["fairing_hood"]["mesh"].is_watertight); out["volume_cm3"] = {k: round(float(parts[k]["mesh"].volume) / 1000.0, 2) for k in ("fairing_ring", "fairing_hood")}
out["mass_g"] = {k: round(E.mass_of(parts[k]), 1) for k in ("fairing_ring", "fairing_hood")}; out["mass_g"]["total"] = round(sum(out["mass_g"].values()), 1)
out["dens_PACF_g_cm3"] = E.DENS.get("PACF"); out["fill"] = 1.0; out["bbox_mm"] = {k: np.round(parts[k]["mesh"].extents, 1).tolist() for k in ("fairing_ring", "fairing_hood")}; out["fits_256"] = all(bool(np.all(parts[k]["mesh"].extents <= 256)) for k in ("fairing_ring", "fairing_hood"))
# ---- moving groups over the hard ranges (M1: S1' only; M2 / arm: S1' x S2)
S1 = np.arange(-100.0, 10.1, 2.5); S2 = np.arange(-25.0, 40.1, 5.0); b1 = (1e9, None); b2 = (1e9, None); b1_task = (1e9, None); b2_task = (1e9, None)
for th in S1:
    Mx = G.pose_matrix("M1", E.DER, th, 0.0); d = float(T.query(E.MOV1 @ Mx[:3, :3].T + Mx[:3, 3])[0].min()); b1 = min(b1, (d, float(th)))
    if -85.0 <= th <= 10.0: b1_task = min(b1_task, (d, float(th)))
    for ph in S2:
        Mm = G.pose_matrix("M2", E.DER, th, ph); d = float(T.query(E.MOV2 @ Mm[:3, :3].T + Mm[:3, 3])[0].min()); b2 = min(b2, (d, (float(th), float(ph))))
        if -85.0 <= th <= 10.0: b2_task = min(b2_task, (d, (float(th), float(ph))))
out["M1_min_mm"] = dict(all=round(b1[0], 2), at_S1p=b1[1], task_range_m85_10=round(b1_task[0], 2)); out["M2_arm_min_mm"] = dict(all=round(b2[0], 2), at=b2[1], task_range=round(b2_task[0], 2))
# ---- fixed parts, harness, skin
fx = {k: round(float(cKDTree(E.samp(p["mesh"], 1.0)).query(fs)[0].min()), 2) for k, p in parts.items() if not k.startswith("fairing")}
out["to_fixed_parts_mm"] = fx; out["to_harness_mm"] = {h: round(float(Tr.query(fs)[0].min()), 2) for h, Tr in E.HARN.items()}
d, i = E.TK.query(fs); sd = np.sign(np.einsum("ij,ij->i", fs - E.TP[i], E.TN[i])) * d; out["skin_min_mm"] = round(float(sd.min()), 1)
# the rail itself (stl wing rail strip is the mesh in HARN['wing_rail']); S1 stage fixed parts of the J1 build (motor, stop posts)
for k in ("S1_AK70", "S1_stop_posts", "tpu_S1_bumpers"):
    out["to_" + k + "_mm"] = round(float(cKDTree(E.samp(E.BASE[k]["mesh"], 1.0)).query(fs)[0].min()), 2)
# ---- lead channels and gland
chan = []
for psi in C3.LEAD_PSI:
    a = math.radians(psi); p0 = np.array([C3.X0 - 40.0, C3.Y1 + 18.0 * math.cos(a), C3.Z1 + 18.0 * math.sin(a)]); p1 = np.array([C3.X0 - 40.0, C3.Y1 + 80.0 * math.cos(a), C3.Z1 + 80.0 * math.sin(a)])
    cb = C3.boxbeam(p0, p1, 8.0, 60.0, None, up=(1, 0, 0), ext=0.0); chan.append(round(float(cKDTree(E.samp(cb, 1.0)).query(fs)[0].min()), 1))
out["lead_channel_to_fairing_mm"] = dict(zip(C3.LEAD_PSI, chan))
gx, gz = C3.GLAND["x"], C3.Z1 + C3.GLAND["dz"]; gh = L.cyl("y", C3.GLAND["d"] / 2, C3.PAD_TRIM_Y - 12.0, C3.PAD_TRIM_Y + 6.0, c=(gx, gz), n=40)
out["gland_hole_intersects_fairing_mm3"] = round(float(L.I(fm, gh).volume), 3) if len(L.I(fm, gh).faces) else 0.0
gcl = {k: round(float(cKDTree(E.samp(p["mesh"], 1.0)).query(E.samp(gh, 1.0))[0].min()), 1) for k, p in list(parts.items()) + [("S1_AK70", E.BASE["S1_AK70"])] if not k.startswith("fairing")}
out["gland_hole_to_parts_mm"] = gcl; out["gland_hole_to_bridge_mm"] = {h: round(float(Tr.query(E.samp(gh, 1.0))[0].min()), 1) for h, Tr in E.HARN.items()}
# ---- printability: overhang = downward face within 45 deg of vertical (n . up < -cos45), excluding faces on the bed plane
def pr(m, up, label):
    up = np.asarray(up, float); n = m.face_normals; a = m.area_faces; h = m.triangles_center @ up; bed = (h < h.min() + 0.3) & (n @ up < -0.95); oh = (n @ up < -0.7071) & ~bed
    d = dict(orientation=label, height_mm=round(float(h.max() - h.min()), 1), bed_contact_mm2=round(float(a[bed].sum()), 0), overhang_mm2_gt45=round(float(a[oh].sum()), 0))
    if oh.any():
        sub = m.submesh([np.where(oh)[0]], append=False); d["regions"] = sorted([(round(float(s.area), 0), np.round(s.bounds, 0).tolist()) for s in sub], key=lambda r: -r[0])[:5]
    return d
rg, hd = parts["fairing_ring"]["mesh"], parts["fairing_hood"]["mesh"]
out["print"] = dict(ring=pr(rg, [-1, 0, 0], "stands on its rear edge (x = %.1f on the bed), lip edge up" % C3.FAIR_XS), hood=pr(hd, [-1, 0, 0], "cap down (x = %.1f on the bed), tongue up" % C3.FAIR_X_END),
                    ring_front_down=pr(rg, [1, 0, 0], "ring on its lip edge (alternative)"), single_piece_capdown=pr(parts_single, [-1, 0, 0], "ONE piece, cap down (ring hangs over the S2 window)"))
json.dump(out, open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "fairing_check.json"), "w"), indent=1, default=float)
print(json.dumps(out, indent=1, default=float))
