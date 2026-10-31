"""Compact chassis plate (replaces the big backplate) + axilla cradle + front-strap path. Right side, robot frame, mm.
Chassis plate: 6061-T6 8 mm, x in [-88,-80] (front face seats the clasp bridge feet, rear face seats the S1 base block).
Axilla cradle: printed saddle under the armpit with soft pad + 2 webbing slots; the FRONT strap runs from it over the front of
the shoulder to the strap slot on the front end of the clasp (F3 facet)."""
import sys, os, json, numpy as np, trimesh, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodyprobe as bp
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "shoulder")
feet = json.load(open(os.path.join(OUT, "clasp_feet.json"))); sites = json.load(open(os.path.join(OUT, "clasp_sites.json")))
lm = json.load(open(os.path.join(OUT, "proxies", "wearer_landmarks.json"))); gh = np.array(lm["gh_joint_rel_P"])

# ---------------------------------------------------------------- chassis plate
T, X0 = 8.0, -88.0
from scipy.spatial import ConvexHull
def circ(cy, cz, r, n=40): return [(cy + r * np.cos(a), cz + r * np.sin(a)) for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]
seed = []
for cy_ in (-68.0, 68.0):                                  # S1 base block footprint (y +-68, z -100..-62), 8 mm margin
    for cz_ in (-100.0, -62.0): seed += circ(cy_, cz_, 8.0)
for k_, f_ in feet.items():                                # clasp feet bolt pairs, 16 mm radius -> organic bulge round each foot
    for dz_ in f_["bolts_dz"]: seed += circ(f_["y"], f_["z"] + dz_, 16.0)
# rev 2026-10-13: feet of the faceted pauldron's plate struts (rear face, 2 x M4 each); the outline grows to include them
PAULDRON_FEET = [(-48.0, -140.0), (48.0, -140.0), (0.0, -150.0)]
for cy_, cz_ in PAULDRON_FEET:
    for dz_ in (-7.0, 7.0): seed += circ(cy_, cz_ + dz_, 11.0)
seed = np.array(seed); hull = seed[ConvexHull(seed).vertices]
Y0, Y1, Z0, Z1 = hull[:, 0].min(), hull[:, 0].max(), hull[:, 1].min(), hull[:, 1].max()
plate = cq.Workplane("YZ").polyline([tuple(map(float, p)) for p in hull]).close().extrude(T).translate((X0, 0, 0))
holes = []   # (diameter, y, z, purpose)
for y in (-62.0, 62.0):                       # S1 base block: 4 x M5 (tap 4.2 -> clearance noted in spec), same pattern as s1_base
    for z in (-68.0, -94.0):
        holes.append((4.2, y, z, "M5 tapped (S1 base)"))
for k, f in feet.items():                     # clasp bridge feet: 2 x M4 per foot at +-10 mm
    for dz in f["bolts_dz"]:
        holes.append((3.3, f["y"], f["z"] + dz, "M4 tapped (clasp foot %s)" % k))
for cy_, cz_ in PAULDRON_FEET:                # pauldron keel feet: 2 x M4 per foot at +-7 mm (rev 2026-10-30: was tapped 3.3, which cannot
    for dz in (-7.0, 7.0):                     # clamp a screw driven from the plate front; now clearance 4.5 + 90 deg countersink D9.0 for ISO 10642
        holes.append((4.5, cy_, cz_ + dz, "M4 clearance + csk D9 (pauldron keel foot: ISO 10642 M4 x 12 from the plate front into a 5.6 x 4 insert)"))
json.dump(dict(pauldron_feet=PAULDRON_FEET, x_rear=X0 + T), open(os.path.join(OUT, "plate.json"), "w"))
for d, y, z, why in holes:
    plate = plate.cut(cq.Workplane("YZ").circle(d / 2).extrude(T + 2).translate((X0 - 1, y, z)))
    if "csk" in why:                           # 90 deg countersink on the FRONT face (x = X0), head flush (ISO 10642 M4 head D 8.96)
        plate = plate.cut(cq.Solid.makeCone(4.5 + 0.05 + 1.0, 0.0, 5.55, cq.Vector(X0 - 1.0, y, z), cq.Vector(1, 0, 0)))
# rev 2026-10-13: the spherical relief for the 190 mm ball is gone (the socket cowl's bottom is at z -49.6, above the plate top -54)
cq.exporters.export(plate, os.path.join(OUT, "chassis_plate_R.stl"), tolerance=0.03, angularTolerance=0.05)
m = trimesh.load(os.path.join(OUT, "chassis_plate_R.stl")); m.apply_transform(np.diag([1, -1, 1, 1.0]))
if m.volume < 0: m.invert()
m.export(os.path.join(OUT, "chassis_plate_L.stl"))
# hole-to-hole and hole-to-edge checks (min 2.5 d web)
H = np.array([[y, z] for _, y, z, _ in holes]); D = np.array([d for d, *_ in holes])
mind = min(np.linalg.norm(H[i] - H[j]) - (D[i] + D[j]) / 2 for i in range(len(H)) for j in range(i + 1, len(H)))
edge = min(min(h[0] - Y0, Y1 - h[0], h[1] - Z0, Z1 - h[1]) for h in H)
print("plate %.0f x %.0f x %.0f mm, %d holes, min hole web %.1f mm, min hole-to-edge %.1f mm, Al mass %.0f g" % (Y1 - Y0, Z1 - Z0, T, len(holes), mind, edge, plate.val().Volume() * 2.70e-3))

cc = json.load(open(os.path.join(OUT, "clasp_cradle.json"))); A = np.array(cc["A"]); CL = cc["CL"]

# ---------------------------------------------------------------- front strap path: cradle front slot -> over the front of the shoulder -> F3 slot
body = bp.load()
s3 = sites["F3_pec"]; n3, u3 = np.array(s3["n"]), np.array(s3["u"]); v3 = np.cross(n3, u3)
if s3.get("slot_edge") == "+u":     # rev 2026-10-14: 35 x 3.2 slot (32 mm strap) at the +u end, long axis along v
    S3 = np.array(s3["p"]) + n3 * (7.0 + 1.75) + u3 * (s3["plate"][0] / 2 - 4.5)
else:
    S3 = np.array(s3["p"]) + n3 * (7.0 + 1.75) + v3 * (s3["plate"][1] / 2 - 4.5)       # F3 strap slot centre (mid plate thickness; G 7 rev 2026-10-12c)
S0 = np.array(cc["front_slot"])                                   # cradle front slot centre (now part of the clasp frame)
lo = np.minimum(S0, S3) - 80; hi = np.maximum(S0, S3) + 80
keep = np.all((body.triangles_center > lo) & (body.triangles_center < hi), axis=1)
local = body.submesh([np.where(keep)[0]], append=True)
def route(a, b, need=2.5):
    pts = [np.asarray(a, float), np.asarray(b, float)]
    for _ in range(14):
        out = [pts[0]]; ch = False
        for p, q in zip(pts[:-1], pts[1:]):
            Q = p[None] + np.linspace(0, 1, 40)[1:-1, None] * (q - p)[None]
            cp, d, fi = trimesh.proximity.closest_point(local, Q)
            sg = np.einsum("ij,ij->i", Q - cp, local.face_normals[fi]); dist = np.where(sg >= 0, d, -d); i = np.argmin(dist)
            if dist[i] < need - 0.2:
                out.append(Q[i] + local.face_normals[fi[i]] * (need - dist[i] + 1.0)); ch = True
            out.append(q)
        pts = out
        if not ch: break
    return pts
path = route(S0, S3)
L = sum(np.linalg.norm(b - a) for a, b in zip(path[:-1], path[1:])); chord = float(np.linalg.norm(S3 - S0))

# ---------------------------------------------------------------- rev 2026-10-14: strap width, route clearances, on-skin line pressure
STRAP_W = float(cc.get("strap_w", 20.0)); SLOT_L = float(cc.get("slot_l", 22.0)); T_SPEC = float(os.environ.get("STRAP_T", 10.0))     # donning-tension mark on the strap
atl = json.load(open(os.path.join(OUT, "emg_atlas.json")))["landmarks"]
NOTCH, ACR = np.array(atl["NOTCH"]), np.array(atl["ACR_top"]); cl = ACR - NOTCH; PLEX = NOTCH + 0.65 * cl       # estimated landmarks; plexus proxy = clavicle point at 65 % (+-20 mm)
APEX = np.array([A[0], A[1] - 0.0, cc["apex_z"]])                                                               # axillary vault apex above the cradle (x, y of the cradle plane, z from the mesh sections)
# dense on-skin strap line: resample the routed polyline (2 mm), pull each sample onto the skin (closest point), smooth, then compute edges and curvature
P = np.array(path); seg = np.linalg.norm(np.diff(P, axis=0), axis=1); sarr = np.r_[0, np.cumsum(seg)]
sg = np.arange(0, sarr[-1] + 1e-6, 2.0); Q = np.c_[[np.interp(sg, sarr, P[:, i]) for i in range(3)]].T
cp, dd, fi = trimesh.proximity.closest_point(local, Q); nrm = local.face_normals[fi]
from scipy.ndimage import gaussian_filter1d
Qs = np.c_[[gaussian_filter1d(cp[:, i], 3.0, mode="nearest") for i in range(3)]].T         # skin-following centre line (sigma 6 mm)
cps, dds, fis = trimesh.proximity.closest_point(local, Qs); Qs = cps; nrm = local.face_normals[fis]
tan = np.gradient(Qs, axis=0); tan /= np.linalg.norm(tan, axis=1)[:, None]
side = np.cross(tan, nrm); side /= np.linalg.norm(side, axis=1)[:, None]                 # in-skin direction across the strap width
edgeA, edgeB = Qs + side * STRAP_W / 2, Qs - side * STRAP_W / 2
dt = np.gradient(tan, axis=0) / 2.0                                                      # d t / d s  (1/mm, 2 mm spacing)
kap_n = np.maximum(0.0, -np.einsum("ij,ij->i", dt, nrm))                                  # curvature toward the body (presses the strap into the skin)
kap_tot = np.linalg.norm(dt, axis=1)
def pr(T, w): return T * kap_n * 1e3 / (w * 1e-3) * 1e-3           # kPa  (T N, kappa 1/mm -> 1/m, w mm -> m)
def pr_win(T, w):                                                   # webbing bridges features narrower than its width: average the curvature over a window of one strap width
    k = max(3, int(round(w / 2.0)) | 1); return np.convolve(np.pad(pr(T, w), (k // 2, k // 2), mode="edge"), np.ones(k) / k, mode="valid")
def mind(X, y): return float(np.linalg.norm(X - y, axis=1).min())
Xall = np.vstack([edgeA, edgeB, Qs])
def mind_seg(X, a0, b0):
    t_ = np.clip(((X - a0) @ (b0 - a0)) / ((b0 - a0) @ (b0 - a0)), 0, 1); return float(np.linalg.norm(X - (a0 + t_[:, None] * (b0 - a0)), axis=1).min())
clr = dict(strap_w_mm=STRAP_W, slot_l_mm=SLOT_L, tension_spec_N=T_SPEC,
           edge_to_axilla_apex_mm=round(mind(Xall, APEX), 1), edge_to_plexus_proxy_mm=round(mind(Xall, PLEX), 1), edge_to_AC_joint_mm=round(mind(Xall, ACR), 1),
           edge_to_clavicle_line_mm=round(mind_seg(Xall, NOTCH, ACR), 1), edge_to_C7_mm=round(mind(Xall, np.array(atl["C7"])), 1),
           min_z_start=round(float(Qs[:, 2].min()), 1), max_z_end=round(float(Qs[:, 2].max()), 1),
           path_len_mm=round(float(np.linalg.norm(np.diff(Qs, axis=0), axis=1).sum()), 0),
           kappa_n_max_per_m=round(float(kap_n.max() * 1e3), 1), kappa_n_mean_per_m=round(float(kap_n.mean() * 1e3), 1), kappa_tot_mean_per_m=round(float(kap_tot.mean() * 1e3), 1),
           line_pressure_normal_kPa=dict(mean_at_spec_T=round(float(pr(T_SPEC, STRAP_W).mean()), 2), max_at_spec_T=round(float(pr(T_SPEC, STRAP_W).max()), 2), max_width_averaged_at_spec_T=round(float(pr_win(T_SPEC, STRAP_W).max()), 2),
                                         max_width_averaged_at_12N_20mm=round(float(pr_win(12.0, 20.0).max()), 2),
                                         mean_at_12N=round(float(pr(12.0, STRAP_W).mean()), 2), mean_at_12N_20mm=round(float(pr(12.0, 20.0).mean()), 2)))
# strap travel direction at the slots vs the slot long axes (a flat strap passes a slot without a twist only if its width lies along the slot)
dz0 = tan[0]; dz1 = tan[-1]
cr_slot_axis = np.array([0.0, -np.sin(np.radians(cc["TILT"])), np.cos(np.radians(cc["TILT"]))])     # cradle slot long axis (cradle-local z, tilted)
f3_long = v3 if s3.get("slot_edge") == "+u" else u3
width_dir0 = np.cross(dz0, nrm[0]); width_dir1 = np.cross(dz1, nrm[-1])
clr["twist_deg_at_cradle_slot"] = round(float(np.degrees(np.arccos(abs(width_dir0 / np.linalg.norm(width_dir0) @ cr_slot_axis)))), 0)
clr["twist_deg_at_F3_slot"] = round(float(np.degrees(np.arccos(abs(width_dir1 / np.linalg.norm(width_dir1) @ f3_long)))), 0)
_pw = pr_win(T_SPEC, STRAP_W); _i = int(_pw.argmax()); clr["hotspot_at_arclength_mm"] = round(float(2.0 * _i), 0); clr["hotspot_point"] = Qs[_i].round(0).tolist()
clr["line_pressure_normal_kPa"]["width_averaged_profile_every_8mm"] = [round(float(x), 1) for x in _pw[::4]]
print("strap %.0f mm: clearances" % STRAP_W, json.dumps(clr))
json.dump(dict(axilla=A.round(1).tolist(), strap_front=[p.round(1).tolist() for p in path], strap_front_chord_mm=round(chord, 0), strap_front_routed_upper_bound_mm=round(L, 0),
               strap_w_mm=STRAP_W, tension_spec_N=T_SPEC, strap_skin_line=Qs[::3].round(1).tolist(), clearances=clr),
          open(os.path.join(OUT, "strap_axilla.json"), "w"), indent=1)
print("axilla cradle at", A.round(0).tolist(), "| front strap slot-to-slot: straight %.0f mm, routed-over-skin upper bound %.0f mm" % (chord, L))
