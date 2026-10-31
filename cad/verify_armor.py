"""Gates for the armoured pauldron (rev 2026-10-30): crown, keel, spaulder (group F).
  1 static clearance to every neighbour (stage contents, plate, clasp, wing, spine bridge, wearer skin, each other)
  2 S1 sweep: everything that turns with S1 (M1 + balancer M1), HARD range 5..150 deg at 1 deg
  3 arm sweep: M2 / M3 (+ balancer M2) over the SOFT box (S1 10..140 / 5, S2 -30..88 / 2, S3 -70/0/90)
  4 wearer range of motion (ergonomics): first-contact angle of Forest's own arm (abduction, scaption, flexion) and neck
    (lateral flexion to the right, rotation to the right, flexion, extension) against the new armour, the old pauldron
    (rev 2026-10-23) and the clasp, from the MakeHuman mesh with rigid rotations (estimate, see SPEC_shoulder_housing.md)
  5 wall thickness (ray test), watertight, volume / mass (PA-CF 1.15 g/cm3), 256 mm cube, best print orientation (overhang)
  6 assembly paths: crown slides on along -x over the bare S1 base; keel rises +z; spaulder drops -z
Writes stl/shoulder/pauldron/verify_armor.json. Distances use dense surface samples + KD-trees (error ~ +-0.3 mm)."""
import os, sys, json, glob, time, numpy as np, trimesh
from scipy.spatial import cKDTree
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak
ST = os.path.join(HERE, "..", "stl"); SH = os.path.join(ST, "shoulder"); PD = os.path.join(SH, "pauldron")
t0 = time.time()
def log(*a): print("[%5.0fs]" % (time.time() - t0), *a, flush=True)
L = lambda p: trimesh.load(os.path.join(ST, p + ".stl"))
def pts(m, sp=0.7):
    p, _ = trimesh.sample.sample_surface_even(m, int(m.area / sp ** 2) + 500); return np.vstack([p, m.vertices])
ARM = {k: L("shoulder/pauldron/pauldron_%s_R" % k) for k in ("crown", "keel", "spaulder")}
OLD = trimesh.util.concatenate([trimesh.load(p) for p in glob.glob(os.path.join(ST, "archive", "pauldron_rev2026-10-23", "pauldron_*_R.stl"))
                                if not p.endswith("pauldron_R.stl")])
AP = {k: pts(m, 0.8) for k, m in ARM.items()}; AT = {k: cKDTree(p) for k, p in AP.items()}
ALLT = cKDTree(np.vstack(list(AP.values()))); OLDT = cKDTree(pts(OLD, 0.6))
G = ak.groups(include_fingers=False); ak.install_balancer(G)
stops = {k: L("shoulder/stops/" + k) for k in ("s1_post_a_R", "s1_post_b_R", "tpu_s1_a_R", "tpu_s1_b_R")}
body = L("shoulder/proxies/wearer_makehuman"); plate = L("shoulder/chassis_plate_R")
clasp = L("shoulder/clasp_frame_R"); wing = L("shoulder/scapular/wing_R"); spine = L("shoulder/scapular/spine_bridge")
info = json.load(open(os.path.join(PD, "pauldron.json"))); PADS = {k: (np.array(v["centre"]), np.array(v["normal"])) for k, v in info["pads"].items()}
R = dict(gates={})
def gate(name, value, need, ok=None, note=""):
    ok = (value >= need) if ok is None else ok
    R["gates"][name] = dict(value=round(float(value), 2), need=need, pass_=bool(ok), note=note); log(("PASS " if ok else "FAIL ") + name, round(float(value), 2), "need", need, note)

ROM_ONLY = os.environ.get('ROM_ONLY', '0') == '1'
if not ROM_ONLY:
    # ---------------------------------------------------------------- 1 static clearances
    def dmin(k, P, mask=None):
        if mask is not None: P = P[mask]
        d, i = AT[k].query(P, distance_upper_bound=60.0, workers=-1); j = int(np.argmin(d)); return float(min(d[j], 60.0)), P[j]
    cont = np.vstack([pts(m, 0.8) for m in list(G["F"].values()) + list(stops.values())])
    for k in ARM:
        gate(f"static/{k}-stage contents", dmin(k, cont)[0], 3.0)
    pp = pts(plate, 0.8)
    gate("static/crown-plate", dmin("crown", pp)[0], 3.0)
    gate("static/spaulder-plate", dmin("spaulder", pp)[0], 3.0)
    kq = AP["keel"][AP["keel"][:, 0] > -74.5]                                     # keel away from its flange / boss seat (x -80..-74)
    gate("static/keel-plate outside the bolted flange", float(cKDTree(pp).query(kq, workers=-1)[0].min()), 3.0, note="flange face on the plate rear face is the bolted seat")
    cp = pts(clasp, 0.8); near = np.min([np.linalg.norm(cp - c, axis=1) for c, _ in PADS.values()], axis=0) < 20.0
    for k in ARM:
        gate(f"static/{k}-clasp" + (" outside the 2 seats" if k == "spaulder" else ""), dmin(k, cp, ~near if k == "spaulder" else None)[0], 3.0)
    for nm, m in (("wing", wing), ("spine bridge", spine)):
        P = pts(m, 0.8)
        for k in ARM: gate(f"static/{k}-{nm}", dmin(k, P)[0], 3.0)
    bp = pts(body, 1.0); bp = bp[(bp[:, 2] > -300) & (bp[:, 0] > -330)]
    for k in ARM:
        if k != "spaulder": d, p_ = dmin(k, bp); gate(f"static/{k}-skin", d, 2.0); continue
        d, i = AT[k].query(bp, distance_upper_bound=60.0, workers=-1); ap = AP[k][np.minimum(i, len(AP[k]) - 1)]
        nearpad = np.min([np.linalg.norm(ap - c, axis=1) for c, _ in PADS.values()], axis=0) < 35.0
        gate("static/spaulder-skin away from the seats", float(min(d[~nearpad].min(), 60)), 10.0, note="rule 10 mm over the shoulder top")
        gate("static/spaulder-skin within 35 mm of a seat", float(min(d[nearpad].min(), 60)), 2.5, note="seats sit on stable zones (F1 / F4)")
    # part-to-part: designed joints
    d, _ = AT["crown"].query(AP["keel"]); lipzone = AP["keel"][:, 2] > -101.0
    gate("joint/keel lip inside crown hem (gap)", float(d[lipzone].min()), 0.3, note="designed nesting gap 0.6-1.4 mm; screws M3x8 at the hem")
    gate("static/keel-crown away from the lip", float(d[~lipzone].min()), 1.0)
    d, _ = AT["crown"].query(AP["spaulder"]); posts = (np.abs(AP["spaulder"][:, 1]) > 78) & (AP["spaulder"][:, 0] > -100) & (AP["spaulder"][:, 2] < -55)
    gate("static/spaulder-crown away from the ear posts", float(d[~posts].min()), 1.0)

    # ---------------------------------------------------------------- 2 S1 sweep (hard range)
    m1 = [m for m in G["M1"].values()] + [L("shoulder/stops/s1_lug_R")] + [m for nm, (m, g) in ak.balancer_parts().items() if g == "M1"]
    P1 = np.vstack([pts(m, 1.0) for m in m1]); P1 = P1[P1[:, 2] < -20]
    best = {k: (1e9, None) for k in ARM}
    for th in np.arange(5.0, 150.01, 1.0):
        Rz = trimesh.transformations.rotation_matrix(np.radians(-th), [0, 0, 1])[:3, :3]; Q = P1 @ Rz.T
        for k in ARM:
            d = float(min(AT[k].query(Q, distance_upper_bound=40.0, workers=-1)[0].min(), 40.0))
            if d < best[k][0]: best[k] = (d, th)
    for k, (d, th) in best.items(): gate(f"S1 sweep/{k} (hard 5..150)", d, 3.0, note=f"worst at S1 {th:.0f}")

    # ---------------------------------------------------------------- 3 arm sweep (soft box)
    parts2 = {(g, k): pts(m, 1.5) for g in ("M2", "M3") for k, m in G[g].items()}
    bal2 = {nm: pts(m, 1.5) for nm, (m, g) in ak.balancer_parts().items() if g == "M2"}
    worst = {k: (1e9, None) for k in ARM}; old_w = (1e9, None)
    for s1 in np.arange(10.0, 140.01, 5.0):
        for s2 in np.arange(-30.0, 88.01, 3.0):
            for s3 in (-70.0, 0.0, 90.0):
                q = dict(S1=s1, S2=s2, S3=s3, E=0, W1=0, W2=0, W3=0, G=50); F = ak.frames(q); BF = ak.balancer_frames(q)
                ch = [p @ F[g][:3, :3].T + F[g][:3, 3] for (g, k), p in parts2.items()] + [p @ BF[nm][:3, :3].T + BF[nm][:3, 3] for nm, p in bal2.items() if nm in BF]
                Q = np.vstack(ch); Q = Q[Q[:, 2] < 0]
                for k in ARM:
                    bb = ARM[k].bounds; Qk = Q[np.all((Q > bb[0] - 41.0) & (Q < bb[1] + 41.0), axis=1)]
                    d = float(min(AT[k].query(Qk, distance_upper_bound=40.0, workers=-1)[0].min(), 40.0)) if len(Qk) else 40.0
                    if d < worst[k][0]: worst[k] = (d, (s1, s2, s3))
                bb = OLD.bounds; Qk = Q[np.all((Q > bb[0] - 41.0) & (Q < bb[1] + 41.0), axis=1)]
                d = float(min(OLDT.query(Qk, distance_upper_bound=40.0, workers=-1)[0].min(), 40.0)) if len(Qk) else 40.0
                if d < old_w[0]: old_w = (d, (s1, s2, s3))
    for k, (d, q) in worst.items(): gate(f"arm sweep/{k} (soft box)", d, 3.0, note="worst at S1/S2/S3 %s" % (q,))
    R["arm_sweep_old_pauldron_mm"] = [round(old_w[0], 2), old_w[1]]


if ROM_ONLY: R['gates'] = {}
# ---------------------------------------------------------------- 4 wearer ROM (ergonomics)
lm = json.load(open(os.path.join(SH, "proxies", "wearer_landmarks.json")))
GH = np.array(lm["gh_joint_rel_P"], float); NECK = np.array(lm["neck_joint_rel_P"], float)
V = body.vertices
ax = np.array([0.0, np.sin(np.radians(10)), -np.cos(np.radians(10))])           # hanging arm axis (10 deg abduction)
t = (V - GH) @ ax; radial = V - GH - t[:, None] * ax; rr = np.linalg.norm(radial, axis=1)
armv = V[(t > 25) & (t < 320) & (rr < 62) & (radial[:, 1] > -40)]             # upper arm + elbow, torso side excluded
cl_t = cKDTree(pts(clasp, 0.8))
def rot(axis, ang, piv, P, wgt=None):
    a = np.radians(ang) if wgt is None else np.radians(ang) * wgt
    axis = np.asarray(axis, float) / np.linalg.norm(axis); P = P - piv
    if wgt is None:
        Rm = trimesh.transformations.rotation_matrix(a, axis)[:3, :3]; return P @ Rm.T + piv
    c, s = np.cos(a)[:, None], np.sin(a)[:, None]; k = axis[None, :]
    return P * c + np.cross(k, P) * s + k * (P @ axis)[:, None] * (1 - c) + piv
def first_contact(Pfun, angles, trees, need=2.0):
    out = {}
    for nm, tr in trees.items():
        out[nm] = None
        for a in angles:
            if float(tr.query(Pfun(a), distance_upper_bound=need + 1, workers=-1)[0].min()) < need: out[nm] = float(a); break
    return out
_cp = pts(clasp, 0.8); _seat = np.min([np.linalg.norm(_cp - c, axis=1) for c, _ in PADS.values()], axis=0) < 16.0
TREES = dict(armour=ALLT, old_pauldron=OLDT, clasp=cl_t, clasp_seat_bosses=cKDTree(_cp[_seat]))   # the D24 bosses the armour seats sit on
rom = {}
for nm, target in (("abduction", [0, 1.0, 0]), ("scaption (30 deg)", [-np.sin(np.radians(30)), np.cos(np.radians(30)), 0]), ("flexion", [-1.0, 0, 0])):
    axis = np.cross([0, 0, -1.0], target)
    rom["arm " + nm] = first_contact(lambda a: rot(axis, a, GH, armv), np.arange(0, 110.1, 2.5), TREES)   # GH share of full elevation (McClure 2001, 1.7:1)
hv = (V[:, 2] > NECK[2]) & (np.abs(V[:, 1] - NECK[1]) < 115) & (V[:, 0] < -60)
head = V[hv]; wgt = np.clip((head[:, 2] - NECK[2]) / 50.0, 0, 1)
for nm, axis, rng in (("neck lateral flexion to the right", [-1.0, 0, 0], 60), ("neck rotation to the right", [0, 0, -1.0], 90),
                      ("neck flexion", [0, -1.0, 0], 70), ("neck extension", [0, 1.0, 0], 70)):
    rom[nm] = first_contact(lambda a: rot(axis, a, NECK, head, wgt), np.arange(0, rng + 1, 2.5), TREES)
R["wearer_rom_first_contact_deg"] = rom
NORM = {"arm abduction": "110 GH (171 total)", "arm scaption (30 deg)": "110 GH", "arm flexion": "110 GH (165 total)", "neck lateral flexion to the right": 42, "neck rotation to the right": 71, "neck flexion": 58, "neck extension": 59}
for nm, v in rom.items():
    log("ROM %-36s armour %s | old pauldron %s | clasp seat bosses %s | clasp %s" % (nm, v["armour"], v["old_pauldron"], v["clasp_seat_bosses"], v["clasp"]))
    a = v["armour"]; b = [x for x in (v["clasp"],) if x is not None]
    # gate: the armour must not stop the motion before the old pauldron or the clasp already do (no new restriction)
    # gate: no new restriction vs the rev 2026-10-23 pauldron (the clasp touches the skin by design: reported, not a reference)
    ref = v["old_pauldron"]; sb = v["clasp_seat_bosses"]
    # pass if no new restriction vs the old pauldron, or if the armour is first touched only where the clasp boss it sits on is
    # already touched (within one 2.5 deg step): the seat cannot be reached later than the boss under it
    ok = (a is None) or (ref is not None and a >= ref) or (sb is not None and a >= sb - 2.5)
    gate("ROM/" + nm, a if a is not None else 999, ref if ref is not None else 999, ok=ok,
         note=("no contact in the tested range" if a is None else "first contact") + "; old pauldron %s; clasp seat boss %s; clasp %s; normal %s deg" % (ref, sb, v["clasp"], NORM.get(nm, "-")))

if ROM_ONLY: print(json.dumps(R['wearer_rom_first_contact_deg'], indent=1)); sys.exit(0)
# ---------------------------------------------------------------- 5 printability, walls, mass
pr = {}
for k, m in ARM.items():
    p_, fi = trimesh.sample.sample_surface_even(m, 6000); n_ = m.face_normals[fi]
    loc, ri, _ = m.ray.intersects_location(p_ - n_ * 0.05, -n_, multiple_hits=False)
    th = np.full(len(p_), np.nan); th[ri] = np.linalg.norm(loc - p_[ri], axis=1)
    th = th[np.isfinite(th)]
    best_o = None
    for nm, up in (("+z", [0, 0, 1]), ("-z", [0, 0, -1]), ("+x", [1, 0, 0]), ("-x", [-1, 0, 0]), ("+y", [0, 1, 0]), ("-y", [0, -1, 0])):
        up = np.array(up, float); h = m.vertices @ up; zf = m.triangles_center @ up
        over = (m.face_normals @ up < -np.cos(np.radians(45))) & (zf > h.min() + 0.6)
        a_ = float(m.area_faces[over].sum())
        if best_o is None or a_ < best_o[1]: best_o = (nm, a_)
    pr[k] = dict(volume_cm3=round(m.volume / 1e3, 2), mass_g=round(m.volume / 1e3 * 1.15, 1), watertight=bool(m.is_watertight),
                 extents=m.extents.round(1).tolist(), fits_256=bool(m.extents.max() <= 256), wall_p1_mm=round(float(np.percentile(th, 1)), 2),
                 wall_p5_mm=round(float(np.percentile(th, 5)), 2), wall_median_mm=round(float(np.median(th)), 2),
                 best_print_up=best_o[0], support_area_cm2=round(best_o[1] / 100, 1))
    gate(f"print/{k} watertight + 256 cube", 1.0 if (m.is_watertight and m.extents.max() <= 256) else 0.0, 1.0)
    gate(f"print/{k} wall p5", pr[k]["wall_p5_mm"], 1.4, note="nominal 1.6 (keel flange 3-6)")
R["print"] = pr
tot = sum(v["mass_g"] for v in pr.values()); R["mass_total_g"] = round(tot, 1)
gate("mass/armour total <= 182 g (harness model's pauldron)", 182.0 - tot, 0.0, note=f"{tot:.1f} g")

# ---------------------------------------------------------------- 6 assembly paths
base_t = cKDTree(pts(G["F"]["s1_base"], 0.8)); Fall = cKDTree(cont); crown_t = AT["crown"]
def path_min(Pmov, direction, dists, tree):
    return min(float(min(tree.query(Pmov + np.asarray(direction) * d, distance_upper_bound=20.0, workers=-1)[0].min(), 20.0)) for d in dists)
gate("assembly/crown slides on along -x over the bare S1 base", path_min(AP["crown"], [1, 0, 0], np.arange(0.5, 160, 1.0), base_t), 0.5)
kp_ = AP["keel"]
gate("assembly/keel rises +z (stage, motor, crown)", min(path_min(kp_, [0, 0, -1], np.arange(0.5, 90, 1.0), Fall),
                                                        path_min(kp_[kp_[:, 0] > -76.5], [0, 0, -1], np.arange(0.5, 90, 1.0), crown_t)), 0.3,
     note="flange slides parallel to the plate face")
sp_ = AP["spaulder"]; obst = cKDTree(np.vstack([cp[~near], pts(plate, 1.0), pts(wing, 1.0)]))
gate("assembly/spaulder drops -z onto the ears and seats", min(path_min(sp_, [0, 0, 1], np.arange(0.5, 60, 1.0), obst),
                                                                path_min(sp_[~posts], [0, 0, 1], np.arange(0.5, 60, 1.0), crown_t)), 0.3)
R["all_pass"] = all(v["pass_"] for v in R["gates"].values())
json.dump(R, open(os.path.join(PD, "verify_armor.json"), "w"), indent=1)  # full run only
log("ALL PASS" if R["all_pass"] else "SOME GATES FAIL", "| mass %.1f g" % tot)
