"""Passive hard stops (ball-style joints) with biomimetic 'capsule' pads. Right arm, robot frame.
S1: lug hanging under the yoke disc (r46..58) + two posts on the S1 base/clamp top. Each post carries a TPU 95A capsule pad
    on the face the lug strikes: contact at the hard-stop angle (soft + 5), progressive compression to ~35 % strain at the
    AK70-10 peak torque (459 N on the pad at r 54).
S2: hull pads on the yoke (+S2 end) and clamp ear (-S2 end) struck by the S3-lid corner; each is now a rigid carrier plus a
    TPU facing swept from the contact angle.
Rev 2026-10-12c (I1): S1 hard min -20 -> +5, S2 hard min -60 -> -35 (head keep-out, see rom_limits.py); capsule pads added.
Writes stl/shoulder/stops/*.stl and stops/stops.json."""
import numpy as np, trimesh, os, json, sys
from trimesh.transformations import rotation_matrix as rot, translation_matrix as tr
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sweep_shoulder as ss, rom_limits as rl
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "shoulder", "stops"); os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT):
    if f.endswith(".stl"): os.remove(os.path.join(OUT, f))
S1_MIN, S1_MAX = map(float, rl.HARD["S1"])     # 5, 150
S2_MIN, S2_MAX = float(rl.HARD["S2"][0]), 92.0 # -35, 92 (92 verified; the yoke floor stops the lid at +94 anyway)
R = 54.75; LUG_W, POST_W = 8.0, 10.0     # lug centre radius: lug r 47.5..62 (3 mm off the AK70 case r 44.5)
RAD = 14.5                           # lug radial depth
RP, RADP = 57.25, 8.5                # posts + TPU: r 53..61.5 (3 mm outside the yoke disc r 50, 3 mm inside the ball ribs at r 65)
ZT, ZL0 = -62.0, -59.0               # base top; lug bottom (3 mm over it, gate >= 3 mm)
ZLUG1 = -53.0                        # lug merges into yoke disc (z -56..-49.9)
ZPOST1 = -53.2                       # post top: 3.2 mm under the yoke box floor (z -50) and the rib bridges; lug/TPU contact z -59..-53.2 (54 mm2)
T_PAD_S1 = 11.0                      # TPU 95A pad thickness along the strike direction (mm): 3.8 mm travel = 35 % strain
T_PAD_S2P, T_PAD_S2N = 7.5, 11.0     # S2+: only 2 deg to the yoke-floor contact at +94 -> 2.6 mm travel -> 7.5 mm pad
hl = np.degrees(np.arctan2(LUG_W / 2, RP)); hp = np.degrees(np.arctan2(POST_W / 2, RP)); GAPANG = 0.5
tA = np.degrees(T_PAD_S1 / RP)
alpha = 180.0 + (S1_MIN + S1_MAX) / 2.0            # lug angle in yoke frame so that travel centre faces -x
def at(g, r, z, rad, tan, h, z0):
    b = trimesh.creation.box(extents=[rad, tan, h])
    b.apply_transform(tr([r, 0, z0 + h / 2])); b.apply_transform(rot(np.radians(g), [0, 0, 1])); return b
lug = at(alpha, R, 0, RAD, LUG_W, ZLUG1 - ZL0, ZL0)
gA = alpha - S1_MIN + hl + hp + GAPANG; gB = alpha - S1_MAX - hl - hp - GAPANG      # where the old rigid posts' faces made contact
postA = at(gA + tA, RP, 0, RADP, POST_W, ZPOST1 - (ZT - 4.0), ZT - 4.0)
postB = at(gB - tA, RP, 0, RADP, POST_W, ZPOST1 - (ZT - 4.0), ZT - 4.0)
padA = at(gA - hp + tA / 2, RP, 0, RADP, T_PAD_S1, ZPOST1 - ZT - 0.0, ZT)            # TPU on post A's lug-side face
padB = at(gB + hp - tA / 2, RP, 0, RADP, T_PAD_S1, ZPOST1 - ZT - 0.0, ZT)
def lid_corner(lid, ang, below=True):
    m = lid.copy(); m.apply_transform(rot(np.radians(-ang), [0, 1, 0])); return m.vertices
def pad_neg(lid, ang):
    v = lid_corner(lid, ang); k = np.argsort(-(v[:, 0]))[:4]; v = v[k]
    v = v[np.argsort(v[:, 2])[:2]].copy(); v[:, 0] -= 0.6
    v[:, 1] = np.clip(v[:, 1], -19, -5)                                               # stands on the -y yoke ear only
    base = v.copy(); base[:, 0] = 57.0; base[:, 2] -= 10.0
    v2 = v.copy(); v2[:, 2] -= 10.0
    return trimesh.Trimesh(vertices=np.vstack([v, v2, base])).convex_hull
def pad_pos(lid, ang):
    """Wedge rooted in the yoke ear wall (x=-60.6, y -46..-4) in the path of the lid's lower corner."""
    pts = []
    for a_ in (ang, ang + 3.0):
        v = lid_corner(lid, a_); v = v[v[:, 2] < 0].copy(); v[:, 1] = np.clip(v[:, 1], -19, -5); v[:, 0] = np.maximum(v[:, 0], -80); pts.append(v)
    v = np.vstack(pts); root = v.copy(); root[:, 0] = -56.0
    return trimesh.Trimesh(vertices=np.vstack([v, root])).convex_hull
def tpu_sweep(lid, a0, a1, sel):
    """TPU facing = hull of the lid's striking corner swept from a0 (contact) to a1 (carrier face)."""
    pts = []
    for a_ in np.linspace(a0, a1, 6):
        v = lid_corner(lid, a_); v = v[sel(v)].copy(); v[:, 1] = np.clip(v[:, 1], -19, -5); pts.append(v)
    return trimesh.Trimesh(vertices=np.vstack(pts)).convex_hull
F, M1, M2 = ss.load_all()
lid = M2["s3_lid"]
rc = float(np.max(np.linalg.norm(lid.vertices[:, [0, 2]], axis=1)))              # lid corner radius about the S2 axis
spanP, spanN = np.degrees(T_PAD_S2P / rc), np.degrees(T_PAD_S2N / rc)
OFF_P, OFF_N = 5.6, -1.0                                                           # verified offsets from the rigid-only design
padP = pad_pos(lid, S2_MAX + OFF_P + spanP); padN = pad_neg(lid, S2_MIN + OFF_N - spanN)
tpuP = tpu_sweep(lid, S2_MAX + OFF_P - 5.6 + 0.05, S2_MAX + OFF_P + spanP, lambda v: (v[:, 2] < 0) & (v[:, 0] < -20))
tpuN = tpu_sweep(lid, S2_MIN - 0.05, S2_MIN - spanN, lambda v: (v[:, 0] > 20) & (v[:, 2] < np.sort(v[:, 2])[min(8, len(v) - 1)] + 6))
def lid_swept(a0, a1, step=1.0):
    """union of convex hulls of the lid between consecutive angles a0 -> a1 (volume the lid passes through before contact)"""
    angs = np.arange(a0, a1 + 1e-9, step * np.sign(a1 - a0)); hs = []
    for x, y in zip(angs[:-1], angs[1:]):
        hs.append(trimesh.Trimesh(vertices=np.vstack([lid_corner(lid, x), lid_corner(lid, y)])).convex_hull)
    return trimesh.boolean.union(hs, engine="manifold")
CLR_T = 0.4   # deg of free travel kept before TPU contact
tpuP = trimesh.boolean.difference([tpuP, padP, lid_swept(40.0, S2_MAX - CLR_T)], engine="manifold")
tpuN = trimesh.boolean.difference([tpuN, padN, lid_swept(20.0, S2_MIN + CLR_T)], engine="manifold")
tpuP = max(tpuP.split(only_watertight=False), key=lambda m: abs(m.volume)); tpuN = max(tpuN.split(only_watertight=False), key=lambda m: abs(m.volume))
parts = dict(s1_lug=lug, s1_post_a=postA, s1_post_b=postB, s2_pad_pos=padP, s2_pad_neg=padN,
             tpu_s1_a=padA, tpu_s1_b=padB, tpu_s2_pos=tpuP, tpu_s2_neg=tpuN)
for k, v in parts.items(): v.export(os.path.join(OUT, k + "_R.stl"))
info = dict(S1=[S1_MIN, S1_MAX], S2=[S2_MIN, S2_MAX], alpha_deg=alpha, post_a_deg=(gA + tA) % 360, post_b_deg=(gB - tA) % 360, R=R,
            lid_corner_radius=round(rc, 1), tpu_mm=dict(S1=T_PAD_S1, S2pos=T_PAD_S2P, S2neg=T_PAD_S2N),
            tpu_vol_cm3={k: round(v.volume / 1e3, 2) for k, v in parts.items() if k.startswith("tpu")})
json.dump(info, open(os.path.join(OUT, "stops.json"), "w"), indent=1)
print(json.dumps(info, indent=1))
