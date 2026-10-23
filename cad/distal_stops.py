"""Passive stops for S3, E, W1, W2, W3 (I6): rigid sector posts with TPU 95A capsule facings, each placed by bisection so the
FIRST contact (TPU) happens exactly at the hard-stop angle (rom_limits.HARD; soft + 5 deg). Lugs/posts are annular sectors about
the joint axis, joined to their parent part by a convex riser down to the part's surface (printed as part of it / bonded).
Outputs stl/distal/stops_<group>/*.stl (picked up by arm_kin.groups) and stl/distal/stops_distal.json.
Joint / axis / parent groups (frames from arm_kin):
  S3: axis z through P;            lug on M3 (upper arm),     posts on M2 (S3 bracket)
  E : axis y through (0,0,230);    lug = drive-cheek mount neck (ME, already there), posts on M3 (upper-arm tube / housing)
  W1: axis z through (0,D,*);      lug on MW1 (distal tube),  posts on ME (forearm root)
  W2: axis y through (0,D,Zw);     lug = yoke cradle (MW2),   posts on MW1 (wrist caps)
  W3: axis x through (0,D,Zw3);    lug = hand arm (MW3),      posts on MW2 (yoke cradle)"""
import os, sys, json, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak, rom_limits as rl, sweep_shoulder as ss
from trimesh.transformations import rotation_matrix as rot
OUT = os.path.join(HERE, "..", "stl", "distal")
for g in ("stops_M2", "stops_M3", "stops_ME", "stops_MW1", "stops_MW2"):
    d = os.path.join(OUT, g); os.makedirs(d, exist_ok=True)
    for f in os.listdir(d): os.remove(os.path.join(d, f))
D, ZW, ZW3 = ak.D, ak.ZW, ak.ZW3
T_TPU = 6.0          # TPU facing thickness along the strike direction (mm)

def sector(c, ax, ref, r0, r1, h0, h1, a0, a1, n=10):
    """annular sector about axis (point c, unit ax): radii r0..r1, axial h0..h1, angles a0..a1 (deg) from ref (unit, _|_ ax)."""
    ax = np.asarray(ax, float) / np.linalg.norm(ax); ref = np.asarray(ref, float); ref = ref - ax * (ref @ ax); ref /= np.linalg.norm(ref)
    b = np.cross(ax, ref); P = []
    for a in np.linspace(np.radians(a0), np.radians(a1), n):
        u = np.cos(a) * ref + np.sin(a) * b
        for r in (r0, r1):
            for h in (h0, h1): P.append(np.asarray(c, float) + u * r + ax * h)
    return trimesh.Trimesh(vertices=np.array(P)).convex_hull
def riser(block, parent, depth=2.0):
    """convex riser from a block's vertices to the closest points on the parent mesh (+depth into it)."""
    v = block.vertices; cp, d, fi = trimesh.proximity.closest_point(parent, v)
    nrm = parent.face_normals[fi]; inside = cp - nrm * depth
    return trimesh.Trimesh(vertices=np.vstack([v, inside])).convex_hull
def union(ms): return trimesh.boolean.union(ms, engine="manifold") if len(ms) > 1 else ms[0]

def joint_T(joint, ang):
    """transform of the moving group relative to the fixed group at joint angle ang (deg)."""
    if joint == "S3": return rot(np.radians(ang), [0, 0, 1])
    if joint == "E": return ak.about([0, 0, ak.ZE], [0, 1, 0], -ang)
    if joint == "W1": return ak.about([0, D, 0], [0, 0, 1], ang)
    if joint == "W2": return ak.about([0, D, ZW], [0, 1, 0], -ang)
    if joint == "W3": return ak.about([0, D, ZW3], [1, 0, 0], ang)
AX = {"S3": ([0, 0, 0], [0, 0, 1], [1, 0, 0]), "E": ([0, 0, ak.ZE], [0, -1, 0], [0, 0, 1]), "W1": ([0, D, 0], [0, 0, 1], [1, 0, 0]),
      "W2": ([0, D, ZW], [0, -1, 0], [0, 0, 1]), "W3": ([0, D, ZW3], [1, 0, 0], [0, 0, 1])}
# NOTE on signs: sector angles are measured right-handed about AX[ax]; for E and W2 the axis is -y so that +angle = +joint.

G = ak.groups(include_fingers=False)
def first_contact(joint, moving, fixed, angs):
    mm, mf = ss.mgr(moving), ss.mgr(fixed)
    for a in angs:
        T = joint_T(joint, a)
        for k in moving: mm.set_transform(k, T)
        if mm.in_collision_other(mf): return float(a)
    return None

def place(joint, lug_mesh, lug_name, moving_extra, fixed_parent, fixed_group_meshes, hard, side, r0, r1, h0, h1, post_w, guess):
    """bisect the post angle so the TPU facing is first struck at joint angle = hard. side +1: stop at the + end."""
    c, ax, ref = AX[joint]
    rmid = 0.5 * (r0 + r1); wdeg = np.degrees(post_w / rmid); tdeg = np.degrees(T_TPU / rmid)
    moving = dict(moving_extra); moving[lug_name] = lug_mesh
    def build(theta):     # theta = angular position of the TPU strike face
        if side > 0: tpu = sector(c, ax, ref, r0, r1, h0, h1, theta, theta + tdeg); post = sector(c, ax, ref, r0, r1, h0, h1, theta + tdeg, theta + tdeg + wdeg)
        else:        tpu = sector(c, ax, ref, r0, r1, h0, h1, theta - tdeg, theta); post = sector(c, ax, ref, r0, r1, h0, h1, theta - tdeg - wdeg, theta - tdeg)
        return tpu, post
    lo, hi = guess - 40.0, guess + 40.0
    def contact_angle(theta):
        tpu, post = build(theta)
        rng = np.arange(hard - side * 25.0, hard + side * 25.0, side * 0.25)
        return first_contact(joint, moving, {"tpu": tpu}, rng)
    # contact angle increases monotonically with theta for side>0 (post further along -> later contact)
    for _ in range(30):
        mid = 0.5 * (lo + hi); ca = contact_angle(mid)
        if ca is None: ca = hard + side * 30.0
        if (ca - hard) * side > 0: hi, lo = (mid, lo) if side > 0 else (hi, mid)
        else: lo, hi = (mid, hi) if side > 0 else (lo, mid)
        if hi - lo < 0.05: break
    theta = 0.5 * (lo + hi); tpu, post = build(theta)
    post = union([post, riser(post, fixed_parent)])
    return theta, tpu, post

report = {}
def save(group, name, mesh): mesh.export(os.path.join(OUT, group, name + ".stl"))

# ---------------- S3: lug hanging from the upper-arm root plate, posts on the S3 bracket sleeve (radial risers)
R0, R1, H0, H1 = 45.0, 51.0, 100.0, 113.0      # bracket web corners reach r 41.8 -> lug 3.2 mm outside
lug = sector(*AX["S3"], R0, R1, H0, H1, -10.0 - 5.0, -10.0 + 5.0)          # 8-ish mm lug centred at -10 deg
lug_top = sector(*AX["S3"], 30.0, R1, 113.0, 117.0, -10.0 - 5.0, -10.0 + 5.0)  # tab under the root plate (bolted, 2 x M3)
lug = union([lug, lug_top]); save("stops_M3", "s3_lug", lug)
sl, sh = rl.HARD["S3"]
for side, hard, guess in ((+1, sh, 97.0), (-1, sl, -97.0)):
    th, tpu, post = place("S3", lug, "lug", {k: v for k, v in G["M3"].items()}, G["M2"]["s3_bracket"], G["M2"], hard, side, R0, R1, H0 - 4.0, H1 - 4.0, 10.0, guess - 10.0)
    tag = "pos" if side > 0 else "neg"; save("stops_M2", f"s3_post_{tag}", post); save("stops_M2", f"tpu_s3_{tag}", tpu); report[f"S3_{tag}"] = dict(theta=round(th, 2), hard=hard)

# ---------------- E (rev 2026-10-13b, elbow ball): the crossbar (fork_bridge, rear, radius 48..60) strikes posts standing on the ball belt
# (R_E = 44): r 40..52, |y| <= 8, embedded in the solid stop root blocks of ua_lat/ua_med (azimuth 88..114 / -114..-88).
ER0, ER1 = 40.0, 52.0
el, eh = rl.HARD["E"]
moving_E = {k: v for k, v in G["ME"].items()}
for side, hard, guess in ((+1, eh, 93.0), (-1, el, -88.0)):
    th, tpu, post = place("E", moving_E["fork_bridge"], "fork_bridge", {k: v for k, v in moving_E.items() if k != "fork_bridge"},
                          G["M3"]["ua_med"], G["M3"], hard, side, ER0, ER1, -8.0, 8.0, 10.0, guess)
    tag = "pos" if side > 0 else "neg"; save("stops_M3", f"e_post_{tag}", post); save("stops_M3", f"tpu_e_{tag}", tpu); report[f"E_{tag}"] = dict(theta=round(th, 2), hard=hard)

# ---------------- W1: lug on the distal tube just past the root end, posts standing on the root's outer wall (riser only below the root top)
WR0, WR1, WH0, WH1 = 27.0, 36.0, 371.0, 379.0
w1lug = sector(*AX["W1"], WR0, WR1, WH0, WH1, 180.0 - 5.5, 180.0 + 5.5)
save("stops_MW1", "w1_lug", w1lug)
wl, wh = rl.HARD["W1"]
def w1_place(side, hard, guess):
    c, ax, ref = AX["W1"]; r0, r1 = 32.0, 39.0; rmid = 35.5; wdeg = np.degrees(9.0 / rmid); tdeg = np.degrees(T_TPU / rmid)
    def build(theta):
        a0, a1 = (theta, theta + tdeg) if side > 0 else (theta - tdeg, theta)
        b0, b1 = (theta + tdeg, theta + tdeg + wdeg) if side > 0 else (theta - tdeg - wdeg, theta - tdeg)
        tpu = sector(c, ax, ref, r0, r1, WH0 - 1.0, WH1 + 1.0, a0, a1)
        post = union([sector(c, ax, ref, r0, r1, 356.0, WH1 + 1.0, b0, b1), sector(c, ax, ref, 28.5, r1, 356.0, 366.5, min(a0, b0), max(a1, b1))])
        return tpu, post
    moving = dict(G["MW1"]); moving["lug"] = w1lug
    lo, hi = guess - 40.0, guess + 40.0
    for _ in range(30):
        mid = 0.5 * (lo + hi); tpu, post = build(mid)
        ca = first_contact("W1", moving, {"tpu": tpu}, np.arange(hard - side * 25.0, hard + side * 25.0, side * 0.25))
        if ca is None: ca = hard + side * 30.0
        if (ca - hard) * side > 0: hi, lo = (mid, lo) if side > 0 else (hi, mid)
        else: lo, hi = (mid, hi) if side > 0 else (lo, mid)
        if hi - lo < 0.05: break
    th = 0.5 * (lo + hi); tpu, post = build(th); return th, tpu, post
for side, hard, guess in ((+1, wh, 278.0), (-1, wl, 82.0)):
    th, tpu, post = w1_place(side, hard, guess)
    tag = "pos" if side > 0 else "neg"; save("stops_ME", f"w1_post_{tag}", post); save("stops_ME", f"tpu_w1_{tag}", tpu); report[f"W1_{tag}"] = dict(theta=round(th, 2), hard=hard)

# ---------------- W2: yoke cradle strikes posts on the wrist caps (radius 30..34 about the W2 axis, inside the |y| 29.5 slot walls)
w2l, w2h = rl.HARD["W2"]
for side, hard, guess in ((+1, w2h, 110.0), (-1, w2l, -100.0)):
    # The yoke arm discs sit at |y - D| 22.5..26 and the slot walls at 29.5: the posts live in the 26.5..29.0 gap, struck by the arm necks
    th, tpu, post = place("W2", G["MW2"]["w2_yoke"], "w2_yoke", {k: v for k, v in G["MW2"].items() if k != "w2_yoke"}, G["MW1"]["forearm_dist"], G["MW1"], hard, side,
                          23.0, 32.0, -26.0, -22.5, 8.0, guess)
    tag = "pos" if side > 0 else "neg"; save("stops_MW1", f"w2_post_{tag}", post); save("stops_MW1", f"tpu_w2_{tag}", tpu); report[f"W2_{tag}"] = dict(theta=round(th, 2), hard=hard)

# ---------------- W3: two lugs on the +x hand arm at +-100 deg (sideways, clear of the arm neck's +-38 deg and of wrist flexion),
# struck against posts on the proximal side (|angle| ~ 130..160 deg), each post tied to the yoke base plate by a box riser in -x
XA0, XA1 = 23.4, 26.6; WR0_, WR1_ = 19.5, 27.0
w3lugA = sector(*AX["W3"], WR0_ - 4.0, WR1_, XA0 - 0.1, XA1, 100.0 - 7.0, 100.0 + 7.0)
w3lugB = sector(*AX["W3"], WR0_ - 4.0, WR1_, XA0 - 0.1, XA1, -100.0 - 7.0, -100.0 + 7.0)
w3l, w3h = rl.HARD["W3"]
def w3_place(side, hard, guess, lug):
    c, ax, ref = AX["W3"]; r0, r1 = WR0_, WR1_; rmid = 0.5 * (r0 + r1); wdeg = np.degrees(8.0 / rmid); tdeg = np.degrees(T_TPU / rmid)
    def build(theta):
        a0, a1 = (theta, theta + tdeg) if side > 0 else (theta - tdeg, theta)
        b0, b1 = (theta + tdeg, theta + tdeg + wdeg) if side > 0 else (theta - tdeg - wdeg, theta - tdeg)
        tpu = sector(c, ax, ref, r0, r1, XA0, XA1, a0, a1)
        post = sector(c, ax, ref, r0, r1, XA0, XA1, b0, b1)
        bb = post.bounds; rise = trimesh.creation.box(bounds=[[11.0, bb[0][1], bb[0][2]], [XA0 + 0.2, bb[1][1], bb[1][2]]])
        return tpu, union([post, rise])
    moving = dict(G["MW3"]); moving["w3_lug"] = lug
    lo, hi = guess - 30.0, guess + 30.0
    for _ in range(30):
        mid = 0.5 * (lo + hi); tpu, post = build(mid)
        ca = first_contact("W3", moving, {"tpu": tpu}, np.arange(hard - side * 25.0, hard + side * 25.0, side * 0.25))
        if ca is None: ca = hard + side * 30.0
        if (ca - hard) * side > 0: hi, lo = (mid, lo) if side > 0 else (hi, mid)
        else: lo, hi = (mid, hi) if side > 0 else (lo, mid)
        if hi - lo < 0.05: break
    th = 0.5 * (lo + hi); tpu, post = build(th); return th, tpu, post
os.makedirs(os.path.join(OUT, "stops_MW3"), exist_ok=True)
for f in os.listdir(os.path.join(OUT, "stops_MW3")): os.remove(os.path.join(OUT, "stops_MW3", f))
w3lugA.export(os.path.join(OUT, "stops_MW3", "w3_lug_a.stl")); w3lugB.export(os.path.join(OUT, "stops_MW3", "w3_lug_b.stl"))
for side, hard, guess, lug in ((+1, w3h, 150.0, w3lugA), (-1, w3l, -140.0, w3lugB)):
    th, tpu, post = w3_place(side, hard, guess, lug)
    tag = "pos" if side > 0 else "neg"; save("stops_MW2", f"w3_post_{tag}", post); save("stops_MW2", f"tpu_w3_{tag}", tpu); report[f"W3_{tag}"] = dict(theta=round(th, 2), hard=hard)

json.dump(report, open(os.path.join(OUT, "stops_distal.json"), "w"), indent=1)
print(json.dumps(report, indent=1))
