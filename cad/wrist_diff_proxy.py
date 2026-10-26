"""Proxy meshes (cones / cylinders / boxes, NOT tooth-accurate) for the bevel differential wrist + sweep clearance against the existing distal forearm tube.
Writes stl/wrist_diff/*.stl and stl/wrist_diff/wrist_diff_clearance.json. Single core, a few minutes at most.  Run: python3 wrist_diff_proxy.py

Layout (local frame, origin = wrist centre C = (0, D_OFF, Zc) in the MW1 frame; gear axis y, pinions on +-x, claw axis +z):
  side gears A (y<0), B (y>0): miter bevels m2.5 Z14, heel pitch plane at |y| = 17.5, face 8; hub + back crown (servo transfer) behind them (static, bodies of revolution)
  carrier ring: tube about y, R 21..27, y +-8 (carries the pinion shafts), rotates with theta
  2 pinions at x = +-17.5 (heel plane), shaft stubs out to |x| = 31
  hand bracket: plates at |x| = 31..35, spin with the pinion (phi), bolted to the existing hand (arm_kin MW3 group, W3 frame, rigidly moved with the pinion)
  2 STS3215 boxes (24.7 y x 45.2 x x 35 z), shafts along z at x = 0, y = +-21.5, forearm fixed, behind the cluster (front face Zc - sv_front; default 22, the sweep below also runs 45 and 52)."""
import os, sys, json, itertools, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as rot, translation_matrix as tr
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rom_limits as rl, arm_kin as ak
from wrist_diff_kin import bevel
OUT = os.path.join(HERE, "..", "stl", "wrist_diff"); os.makedirs(OUT, exist_ok=True)
D = ak.D; G = bevel(2.5, 14, 14, 8.0)
def frustum(r0, r1, h, axis):                      # solid of revolution about +axis from 0..h, radius r0 -> r1
    prof = np.array([[0, 0], [r0, 0], [r1, h], [0, h]]); m = trimesh.creation.revolve(prof, sections=48)   # revolves about z
    return orient(m, axis)
def orient(m, axis):
    if axis == "y": m.apply_transform(rot(-np.pi / 2, [1, 0, 0]))     # z -> y
    elif axis == "x": m.apply_transform(rot(np.pi / 2, [0, 1, 0]))    # z -> x
    return m
def build(Zc, sv_front=22.0):
    C = np.array([0, D, Zc]); Rg = G["m"] * G["Zg"] / 2; b = G["b"]; od = G["od_gear"] / 2; toe = Rg - b / np.sqrt(2)
    parts = {}
    for s, nm in ((-1, "A"), (1, "B")):
        # gear frustum: from toe plane |y|=toe (radius toe+) to heel plane |y|=Rg (radius od/2) then hub 8 mm radius out to |y| = Rg + 14, crown pinion-side disc r 17 thick 4
        g = frustum(toe + 0.5, od, Rg - toe, "y"); g.apply_translation([0, toe, 0])
        hub = trimesh.creation.cylinder(radius=8.0, height=14.0, sections=32); hub.apply_transform(rot(-np.pi / 2, [1, 0, 0])); hub.apply_translation([0, Rg + 7.0, 0])
        crown = trimesh.creation.cylinder(radius=17.0, height=4.0, sections=48); crown.apply_transform(rot(-np.pi / 2, [1, 0, 0])); crown.apply_translation([0, Rg + 2.0, 0])
        gm = trimesh.util.concatenate([g, hub, crown])
        if s < 0: gm.apply_transform(np.diag([1, -1, 1, 1]) @ np.eye(4)); gm.invert() if gm.volume < 0 else None
        gm.apply_translation(C); parts["side_gear_" + nm] = gm
    ring = trimesh.creation.annulus(r_min=21.0, r_max=27.0, height=16.0, sections=64); ring.apply_transform(rot(-np.pi / 2, [1, 0, 0])); ring.apply_translation(C)
    parts["carrier_ring"] = ring
    pins = []
    for s in (-1, 1):
        p = frustum(toe + 0.5, od, Rg - toe, "x"); p.apply_translation([toe, 0, 0])
        stub = trimesh.creation.cylinder(radius=4.0, height=31 - Rg + 4, sections=24); stub.apply_transform(rot(np.pi / 2, [0, 1, 0])); stub.apply_translation([(Rg + 31) / 2 - 2, 0, 0])
        pm = trimesh.util.concatenate([p, stub])
        if s < 0: pm.apply_transform(np.diag([-1, 1, 1, 1]) @ np.eye(4)); pm.invert() if pm.volume < 0 else None
        pm.apply_translation(C); pins.append(pm)
    parts["pinions"] = trimesh.util.concatenate(pins)
    plates = []
    for s in (-1, 1):
        pl = trimesh.creation.box(extents=[4.0, 60.0, 50.0]); pl.apply_translation([s * 33.0, 0, 15.0]); pl.apply_translation(C); plates.append(pl)
    bridge = trimesh.creation.box(extents=[70.0, 60.0, 4.0]); bridge.apply_translation([0, 0, 38.0]); bridge.apply_translation(C); plates.append(bridge)   # ties the plates to the hand tray rear (z = Zc + 38 -> hand tray at 439 for Zc 401..)
    parts["hand_bracket"] = trimesh.util.concatenate(plates)
    servos = []
    for y in (-21.5, 21.5):
        sv = trimesh.creation.box(extents=[45.2, 24.7, 35.0]); sv.apply_translation([0, y, -sv_front - 17.5]); sv.apply_translation(C); servos.append(sv)
    parts["servos"] = trimesh.util.concatenate(servos)
    return parts, C
def hand_mesh():
    Gk = ak.groups(); ms = [m for k, m in Gk["MW3"].items() if not k.startswith("w3_lug")]
    return trimesh.util.concatenate(ms).convex_hull      # convex hull: conservative (over-estimates the hand) and fast
def about(C, R3):
    T = np.eye(4); T[:3, :3] = R3; T[:3, 3] = C - R3 @ C; return T
def Rxy(W2, W3):
    from wrist_diff_kin import serial_R
    return serial_R(W2, W3)
def sweep(parts, C, hand, fore, step=20.0):
    from trimesh.collision import CollisionManager
    mf = CollisionManager(); mf.add_object("fore", fore)
    ms = CollisionManager(); ms.add_object("servos", parts["servos"])
    res = dict(worst_min_dist={}, n_collide={}, n=0); mover = {"carrier_ring": "th", "pinions": "th", "hand_bracket": "thph", "hand": "thph"}
    allm = dict(parts, hand=hand); w = {k: 1e9 for k in mover}; nc = {k: 0 for k in mover}; ncs = {k: 0 for k in mover}; ws = {k: 1e9 for k in mover}
    for W2 in (-70.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 70.0, 80.0):
        for W3 in (-20.0, -10.0, 0.0, 10.0, 20.0, 30.0):
            res["n"] += 1
            for k, kind in mover.items():
                R = Rxy(W2, W3) if kind == "thph" else Rxy(W2, 0.0)
                m = allm[k].copy(); m.apply_transform(about(C, R))
                mm = CollisionManager(); mm.add_object(k, m)
                d = mf.min_distance_single(m) ; w[k] = min(w[k], d); nc[k] += d <= 0
                d2 = ms.min_distance_single(m); ws[k] = min(ws[k], d2); ncs[k] += d2 <= 0
    res["worst_min_dist"] = w; res["n_collide"] = nc; res["vs_servos_min_dist"] = ws; res["vs_servos_n_collide"] = ncs
    return res
if __name__ == "__main__":
    fore0 = trimesh.load(os.path.join(HERE, "..", "stl", "distal", "forearm_dist_R.stl")); hand = hand_mesh(); out = {}
    for Zc, svf in ((417.0, 22.0), (417.0, 45.0), (417.0, 52.0)):
        parts, C = build(Zc, svf)
        for k, m in parts.items(): m.export(os.path.join(OUT, f"{k}_Zc{int(Zc)}_sv{int(svf)}.stl"))
        print('--- Zc', Zc, 'servo front face at Zc -', svf)
        static = {k: float(v.bounds[1][2]) for k, v in parts.items()}
        # forearm tube as is, then trimmed back at z_cut (the tube must end before the cluster)
        out[f'{Zc:.0f}_sv{svf:.0f}'] = {}
        for cut in (None, Zc - 30.0, Zc - 45.0, Zc - 52.0):
            fore = fore0 if cut is None else fore0.slice_plane([0, 0, cut], [0, 0, -1], cap=True)
            r = sweep(parts, C, hand, fore, step=20.0); out[f'{Zc:.0f}_sv{svf:.0f}']["as_is" if cut is None else f"tube_cut_at_z{cut:.0f}"] = r
            print(f"Zc {Zc:.0f} tube {'as is' if cut is None else 'cut at z=%.0f' % cut}: n={r['n']} min dist (mm) {{{', '.join(f'{k}: {v:.1f}' for k, v in r['worst_min_dist'].items())}}} collide {r['n_collide']} | vs servos {{{', '.join(f'{k}: {v:.1f}' for k, v in r['vs_servos_min_dist'].items())}}}")
        # static fit of the whole cluster to the as-is tube
        from trimesh.collision import CollisionManager
        mf = CollisionManager(); mf.add_object("fore", fore0)
        out[f'{Zc:.0f}_sv{svf:.0f}']["static_overlap_with_as_is_tube"] = {k: float(mf.min_distance_single(v)) for k, v in parts.items()}
        print("  static min dist to as-is tube (<=0 = intersects):", {k: round(v, 1) for k, v in out[f'{Zc:.0f}_sv{svf:.0f}']["static_overlap_with_as_is_tube"].items()})
    json.dump(out, open(os.path.join(OUT, "wrist_diff_clearance.json"), "w"), indent=1, default=float)
    print("wrote", os.path.join(OUT, "wrist_diff_clearance.json"))
