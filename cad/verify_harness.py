"""Harness gate (I2): skin gaps per zone, fixed-part interferences, and the pressure budget.
Zones (robot frame, y from the spine): wing RAIL (foam contact, gap >= 5), wing BRIDGE (over the moving blade, gap >= 10),
TONGUE (under the plate), spine bridge (>= 10 over the spinous processes), clasp (>= 2 everywhere except the designed contact
faces: electrode bosses/pads on F1-F4, the axilla cradle and its 40 mm padded tail).
Writes stl/shoulder/verify_harness.json.  (unify 2026-10-13f: logic unchanged; the pressure gate reads the new harness_loads.json.)"""
import sys, os, json, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sweep_shoulder as ss
B = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "shoulder") + "/"
body = trimesh.load(B + "proxies/wearer_makehuman.stl")
fr = json.load(open(B + "scapular/scapular_frame.json")); SP = fr["spine_y"]; Z = fr["zones"]
cc = json.load(open(B + "clasp_cradle.json")); sites = json.load(open(B + "clasp_sites.json")); paths = json.load(open(B + "clasp_paths.json"))
from trimesh.proximity import ProximityQuery
def gaps(mesh, pts):
    lo, hi = pts.min(0) - 30, pts.max(0) + 30
    keep = np.all((body.triangles_center > lo) & (body.triangles_center < hi), axis=1); loc = body.submesh([np.where(keep)[0]], append=True)
    out, sg = [], []
    for j in range(0, len(pts), 5000):
        q = pts[j:j + 5000]; cp, d, fi = trimesh.proximity.closest_point(loc, q)
        out.append(d); sg.append(np.einsum("ij,ij->i", q - cp, loc.face_normals[fi]))     # sign from the closest face's outward normal
    d = np.concatenate(out); sgn = np.concatenate(sg)
    neg = np.where(sgn < 0)[0]                                      # confirm 'inside' with the full watertight body (few points)
    if len(neg): sgn[neg] = np.where(body.contains(pts[neg]), -1.0, 1.0)
    return np.where(sgn < 0, -d, d)
res = {}
wing = trimesh.load(B + "scapular/wing_R.stl")
v = wing.vertices; dy = v[:, 1] - SP
inz = lambda k: (dy >= Z[k]["y"][0]) & (dy <= Z[k]["y"][1]) & (v[:, 2] >= Z[k]["z"][0]) & (v[:, 2] <= Z[k]["z"][1])
gw = gaps(wing, v)
rail = inz("RAIL"); tongue = inz("TONGUE")
bridge = inz("BRIDGE") & ~tongue & ~rail & (dy >= Z["RAIL"]["y"][1] + 3.0)     # exclude the 3 mm transition wall at the rail edge
res["wing_rail_min_gap_mm"] = round(float(gw[rail].min()), 1)
res["wing_bridge_min_gap_mm"] = round(float(gw[bridge].min()), 1)
res["wing_tongue_min_gap_mm"] = round(float(gw[tongue].min()), 1)
res["wing_other_min_gap_mm"] = round(float(gw[~rail & ~bridge & ~tongue].min()), 1)
sp = trimesh.load(B + "scapular/spine_bridge.stl"); res["spine_bridge_min_gap_mm"] = round(float(gaps(sp, sp.vertices).min()), 1)
# rev 2026-10-16 (gI): the main-box M4 x 8 button heads (2.2 mm) sit under the bridge inner face, in the spine gap: gate them too (sensor_pods.py fastener table)
_pj = os.path.join(B, "..", "pods", "pods.json"); _fz = json.load(open(_pj)).get("fasteners", {}) if os.path.exists(_pj) else {}
_hd = [v["head_to_skin_mesh_mm"] for k, v in _fz.items() if k.startswith("main_box")]
res["spine_screw_heads_min_gap_mm"] = round(min(_hd), 1) if _hd else None
clasp = trimesh.load(B + "clasp_frame_R.stl"); cv = clasp.vertices[::2]; gc = gaps(clasp, cv)
A = np.array(cc["A"]); _t = np.radians(cc.get("TILT", 0.0)); _d = cv - A                       # cradle frame (tilted about x)
_ly = _d[:, 1] * np.cos(_t) - _d[:, 2] * np.sin(_t); _lz = _d[:, 1] * np.sin(_t) + _d[:, 2] * np.cos(_t)
contact = (np.abs(_d[:, 0]) < cc["CL"] / 2 + 3) & (np.abs(_lz) < cc["CH"] / 2 + 3) & (np.abs(_ly) < 12)
tail_end = np.array(paths["arc R2>cradle"][-1]); contact |= np.linalg.norm(cv - tail_end, axis=1) < 40.0 + 9.0
for k in ("F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion"):          # pad faces (the foam/electrode stack is the contact)
    p, n = np.array(sites[k]["p"]), np.array(sites[k]["n"]); w, h = sites[k]["plate"]
    contact |= (np.linalg.norm((cv - p) - np.outer((cv - p) @ n, n), axis=1) < np.hypot(w, h) / 2 + 2) & (np.abs((cv - p) @ n) < 14)
res["clasp_min_gap_outside_contact_faces_mm"] = round(float(gc[~contact].min()), 1)
res["clasp_contact_faces_min_gap_mm"] = round(float(gc[contact].min()), 1)   # bosses press 0.5 mm in by design; cradle sits on the fused crease
# under-plate foam gap for the load facets (plate inner face to skin, sampled over the filleted outline)
import bodyprobe as bp
underf = {}
_npz = np.load(B + "pad_foam.npz") if os.path.exists(B + "pad_foam.npz") else None
for k in ("F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion"):
    s_ = sites[k]
    if _npz is not None and s_.get("contour") and (k + "_t") in _npz.files:      # rev 2026-10-14: contoured plates - foam gap = built inner face (incl. rim roll) to the skin over the bearing area
        _t, _in = _npz[k + "_t"], _npz[k + "_inside"]
        underf[k] = dict(min_mm=round(float(_t[_in].min()), 1), centre_mm=7.0, mean_mm=round(float(_t[_in].mean()), 1), max_mm=round(float(_t[_in].max()), 1), contoured=True,
                         skirt_min_mm=(round(float(_npz[k + "_skirt_t"][_npz[k + "_skirt_mask"]].min()), 1) if (k + "_skirt_t") in _npz.files else None))
        continue
    s_ = sites[k]; p_, n_, u_ = np.array(s_["p"]), np.array(s_["n"]), np.array(s_["u"]); v_ = np.cross(n_, u_); W_, H_ = s_["plate"]; r_ = min(s_.get("fillet", 12.0), min(W_, H_) / 2 - 1)
    gg = []
    for a_ in np.linspace(-W_ / 2, W_ / 2, 13):
        for b_ in np.linspace(-H_ / 2, H_ / 2, 9):
            ax_, bx_ = abs(a_) - (W_ / 2 - r_), abs(b_) - (H_ / 2 - r_)
            if ax_ > 0 and bx_ > 0 and ax_ * ax_ + bx_ * bx_ > r_ * r_: continue
            q_ = p_ + n_ * 7.0 + u_ * a_ + v_ * b_; rr = bp.cast(body, q_ + n_ * 60, -n_)
            if rr is not None: gg.append(float(np.linalg.norm(rr[0] - q_) * np.sign((rr[0] - q_) @ -n_)))
    underf[k] = dict(min_mm=round(min(gg), 1), centre_mm=7.0)
res["load_pad_foam_gap"] = underf
# rev 2026-10-14 printability of the contoured plates: slope of the inner face against the pad plane (print with the pad plane on the bed, outer face up), plate extents
if _npz is not None:
    _pr = {}
    for k in ("F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion"):
        if (k + "_zin") not in _npz.files: continue
        _z, _in = _npz[k + "_zin"], _npz[k + "_inside0"]; _ga, _gb = np.gradient(_z, 1.0, 1.0); _sl = np.degrees(np.arctan(np.hypot(_ga, _gb)))
        _pr[k] = dict(max_slope_deg=round(float(_sl[_in].max()), 0), p95_slope_deg=round(float(np.percentile(_sl[_in], 95)), 0), z_span_mm=round(float(_z[_in].max() - _z[_in].min()), 1),
                      w_h_mm=sites[k]["plate"], fits_256=bool(max(sites[k]["plate"]) + 2 * (_z[_in].max() - _z[_in].min()) < 256))
    res["contoured_plate_print"] = _pr
# rev 2026-10-14: AC-joint foam relief (F4) - nearest clasp surface to skin points inside the relief disc; strap/route clearances from chassis_axilla.py
_f4 = sites["F4_acromion"]
if _f4.get("contour", {}).get("ac_uv"):
    _p, _n, _u = np.array(_f4["p"]), np.array(_f4["n"]), np.array(_f4["u"]); _v = np.cross(_n, _u); _ac = _f4["contour"]["ac_uv"]; _r = _f4["contour"]["foam_relief"][0][2]
    _pts = []
    for _rr in (0.0, 4.0, 8.0, 11.0):
        for _th in np.linspace(0, 2 * np.pi, 8, endpoint=False):
            _a_, _b_ = _ac[0] + _rr * np.cos(_th), _ac[1] + _rr * np.sin(_th)
            if np.hypot(_a_, _b_) < 10.5: continue          # electrode boss footprint (r 8.5 + 2): designed contact, not part of the relief
            _q = _p + _u * _a_ + _v * _b_; _h = bp.cast(body, _q + _n * 60, -_n)
            if _h is not None: _pts.append(_h[0])
    _cp, _dd, _ = trimesh.proximity.closest_point(clasp, np.array(_pts))
    res["ac_relief"] = dict(foam_relief_radius_mm=_r, centre_uv=[round(_ac[0], 1), round(_ac[1], 1)], min_clasp_to_skin_in_window_mm=round(float(_dd.min()), 1), at_ac_proxy_mm=round(float(_dd[0]), 1))
_sa = json.load(open(B + "strap_axilla.json")) if os.path.exists(B + "strap_axilla.json") else {}
res["strap"] = _sa.get("clearances")
worst = cv[~contact][np.argmin(gc[~contact])]; res["clasp_worst_point"] = worst.round(0).tolist()
# fixed-part interferences (designed seats reported as seats)
P = dict(wing=wing, spine=sp, clasp=clasp, plate=trimesh.load(B + "chassis_plate_R.stl"))
SEAT = {("wing", "plate"): "tongue on plate front (bolted)", ("clasp", "plate"): "clasp feet on plate front (bolted)", ("wing", "spine"): "lap joint (bolted, 0.8 mm design offset)"}
names = list(P); inter = {}
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        a, b = names[i], names[j]; ma, mb = ss.mgr({a: P[a]}), ss.mgr({b: P[b]})
        d = ma.min_distance_other(mb); c = ma.in_collision_other(mb)
        depth = 0.0
        if c:
            q = P[a].vertices[np.all((P[a].vertices > P[b].bounds[0]) & (P[a].vertices < P[b].bounds[1]), axis=1)]
            if len(q) > 3000: q = q[np.random.default_rng(0).choice(len(q), 3000, replace=False)]
            if len(q): depth = float(max(0.0, ProximityQuery(P[b]).signed_distance(q).max()))
        inter[f"{a}-{b}"] = dict(min_mm=round(float(d), 2), contact=bool(c), penetration_mm=round(depth, 2), seat=SEAT.get((a, b)))
res["interfaces"] = inter
hl = json.load(open(B + "harness_loads.json")) if os.path.exists(B + "harness_loads.json") else {}
res["pressure_worst_kPa"] = hl.get("worst_kPa"); res["pressure_target_kPa"] = 14.7
res["gates"] = dict(rail_ge_5=res["wing_rail_min_gap_mm"] >= 5, bridge_ge_10=res["wing_bridge_min_gap_mm"] >= 10, spine_ge_10=res["spine_bridge_min_gap_mm"] >= 10 and (res["spine_screw_heads_min_gap_mm"] is None or res["spine_screw_heads_min_gap_mm"] >= 10),
                    clasp_ge_2=res["clasp_min_gap_outside_contact_faces_mm"] >= 2, foam_under_pads_ge_2=all(v["min_mm"] >= 2 for v in underf.values()),
                    no_unplanned_interference=all((not v["contact"]) or (v["seat"] and v["penetration_mm"] <= 0.3) for v in inter.values()),
                    pressure=bool(hl.get("passes", False)))
json.dump(res, open(B + "verify_harness.json", "w"), indent=1)
print(json.dumps(res, indent=1))
