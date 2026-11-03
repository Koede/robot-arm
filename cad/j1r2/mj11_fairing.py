"""MJ11 fixed fairing (2026-11-03): builds fairing_body + fairing_cover (mj_concepts3.fairing_geo), then checks clearances (moving groups over the slot ranges and the wide rev 2 ranges, wearer skin, rail, spine bridge,
pad / dish / liner / spacers / clasp bracket / S1' stage) and printability (build volume, down-facing overhang area >45 deg off the bed, bridge spans).  usage: J1R2_ROOT=<work root> python3 mj11_fairing.py -> stl/j1r2_mounts/MJ11/fairing.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import clib as L, j1r2_geom as G, mj_frame as M, mj_concepts3 as C
def samp(m, sp=1.2): p, _ = trimesh.sample.sample_surface_even(m, int(m.area / sp ** 2) + 300); return np.vstack([p, m.vertices])
def overhang(m, up):
    """area (mm2) of faces whose outward normal points more than 45 deg below the horizontal (n.up < -0.707), excluding faces lying on the bed plane; also the lowest-z layer area (bed contact)."""
    up = np.asarray(up, float); h = m.vertices @ up; fc = m.triangles_center @ up; n = m.face_normals @ up; bed = fc < h.min() + 0.15
    bad = (n < -0.7071) & ~bed; return dict(overhang_mm2=round(float(m.area_faces[bad].sum()), 1), bed_contact_mm2=round(float(m.area_faces[bed & (n < -0.99)].sum()), 1), height_mm=round(float(h.max() - h.min()), 1),
                                            overhang_faces_z_range=None if not bad.any() else [round(float(fc[bad].min() - h.min()), 1), round(float(fc[bad].max() - h.min()), 1)])
if __name__ == "__main__":
    BASE, INFO = G.build(y1=M.Y1, z1=M.Z1, standoff=M.DER["standoff"], full=True)
    parts = C.mj11_geo(BASE); body, cover = parts["fairing_body"]["mesh"], parts["fairing_cover"]["mesh"]; res = {"info": C.FAIR_INFO}
    res["slot_ranges"] = C.fairing_clearance(body, cover, BASE, lim_th=C.FAIR_TH); res["wide_rev2_ranges"] = C.fairing_clearance(body, cover, BASE, lim_th=(-95.0, 35.0), lim_ph=(-30.0, 80.0))
    res["task_box_plus_stops"] = C.fairing_clearance(body, cover, BASE, lim_th=(-85.0, 10.0), lim_ph=(-25.0, 40.0), step_th=2.5, step_ph=5.0)
    H = {"wing_rail": L.I(trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")), L.box(-300, 100, -150, -100.0, -500, 100)), "spine_bridge": trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "spine_bridge.stl"))}
    others = {k: parts[k]["mesh"] for k in ("pad", "seat_dish", "liner", "rail_spacers", "clasp_bracket")}; others.update({k: BASE[k]["mesh"] for k in ("S1_AK70", "S1_stop_posts", "tpu_S1_bumpers")})
    res["parts"] = {}
    for nm, m in (("fairing_body", body), ("fairing_cover", cover)):
        s = samp(m, 1.5); r = dict(watertight=bool(m.is_watertight), volume_cm3=round(abs(m.volume) / 1000, 2), mass_g=round(abs(m.volume) / 1000 * L.DENS["PACF"], 1), bbox=np.round(m.extents, 1).tolist(), skin_gap_mm=round(float(L.skin_gap(m, 4000)[0]), 1),
                                    clear_mm={k: round(float(cKDTree(samp(o, 1.2)).query(s)[0].min()), 2) for k, o in {**H, **others}.items()}, components=len(m.split(only_watertight=False)))
        res["parts"][nm] = r
    res["print"] = dict(body_x0_face_down=overhang(body, [1, 0, 0]), cover_rear_face_down=overhang(cover, [-1, 0, 0]))
    res["mass_total_g"] = round(sum(p["mass_g"] for p in res["parts"].values()), 1)
    d = os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11"); os.makedirs(d, exist_ok=True); json.dump(res, open(os.path.join(d, "fairing.json"), "w"), indent=1, default=float); print(json.dumps(res, indent=1, default=float))
