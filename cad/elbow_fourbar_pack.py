"""Packaging check for the elbow four-bar (design C_compact_inline from elbow_fourbar.py; run that first).
Proxy bars (8 mm thick, 10 mm wide, 5 mm end pad) and the relocated XM540 box are tested against the existing upper-arm shell (ua_lat / ua_med),
the fork group ME (forks, forearm root, stops) and each other over E -5..150 (1 deg) with python-fcl through trimesh.
Two layer stacks (y measured from the arm centre plane, +y lateral):
  S1 tight : crank y 26..34 (on the horn), coupler 34.5..42.5, lever 43..51 (lever bolted to the drive-cap neck on stand-offs)
  S2 clear : crank y 33..41, coupler 41.5..49.5, lever 50..58 (crank clears the drive cap and its forearm-mount neck, y <= 31.7, by 1.3 mm)
The lever is rigid with the forearm group (designed contact with fork_drive / root excluded); crank and coupler are not.
Writes stl/elbow_fourbar/*.stl and packaging.json (new files only).  Run: python3 elbow_fourbar_pack.py  (1 core, ~2 min)."""
import os, sys, json, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as rot, translation_matrix as tr
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "..", "stl", "elbow_fourbar")
import elbow_fourbar as fb, arm_kin as ak, rom_limits as rl

ZE = 230.0
FAM = os.environ.get("FB_FAMILY", "C_compact_inline")
D = json.load(open(os.path.join(OUT, "fourbar.json")))
P = D[FAM]["params"]; PAR = (P["a"], P["b"], P["c"], P["ground_g"], P["ground_dir_deg"], P["lever_beta_deg"])
S = P["branch"]; W, T, PAD = 10.0, 8.0, 5.0
STACKS = {"S1_tight": dict(crank=(26.0, 34.0), coupler=(34.5, 42.5), lever=(43.0, 51.0)),
          "S2_clear": dict(crank=(33.0, 41.0), coupler=(41.5, 49.5), lever=(50.0, 58.0))}

_R = fb.solve(PAR, fb.E_HARD, S)          # whole hard range at once (1 deg steps)
def bar(p0, p1, y0, y1):
    """box between two x-z points (relative to the elbow axis, z up), thickness y0..y1, centred on the segment, ends padded."""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float); d = p1 - p0; L = np.hypot(*d); psi = np.degrees(np.arctan2(d[0], d[1]))
    m = trimesh.creation.box(extents=(W, y1 - y0, L + 2 * PAD)); m.apply_translation((0, (y0 + y1) / 2, L / 2))      # axis along +z from 0..L (padded)
    m.apply_transform(tr((p0[0], 0, p0[1] + ZE)) @ rot(np.radians(psi), [0, 1, 0]))
    return m

def links(E, stack):
    r = _R; i = int(np.argmin(abs(fb.E_HARD - E))); O2 = r["O2"]; C = r["C"][i]; L = r["L"][i]
    st = STACKS[stack]
    return dict(crank=bar(O2, C, *st["crank"]), coupler=bar(C, L, *st["coupler"]), lever=bar((0, 0), L, *st["lever"])), O2

def Tm(E): return ak.about([0, 0, ZE], [0, 1, 0], -E)
def mgr(d):
    m = trimesh.collision.CollisionManager()
    for k, v in d.items(): m.add_object(k, v)
    return m

def run(stack, Es):
    G = ak.groups(); shell = {k: G["M3"][k] for k in ("ua_lat", "ua_med")}; fixed_extra = {k: v for k, v in G["M3"].items() if k.startswith(("e_post", "tpu_e", "s3_lug"))}
    fixed = dict(shell, **fixed_extra); ME = G["ME"]
    hits = {}; mind = {}
    for E in Es:
        ln, O2 = links(E, stack)
        # moving group (forearm side): ME meshes and the lever rotate about the elbow axis; crank / coupler are in world as built
        med = {k: v.copy() for k, v in ME.items()}
        for v in med.values(): v.apply_transform(Tm(E))
        lever = ln["lever"]
        mf = mgr(fixed); mm = mgr(med)
        pairs = [("crank", ln["crank"], mf, "fixed"), ("coupler", ln["coupler"], mf, "fixed"), ("lever", lever, mf, "fixed"),
                 ("crank", ln["crank"], mm, "ME"), ("coupler", ln["coupler"], mm, "ME"),
                 ("lever", lever, mm, "ME")]
        for nm, mesh, man, tag in pairs:
            c1 = trimesh.collision.CollisionManager(); c1.add_object(nm, mesh)
            hit, names = c1.in_collision_other(man, return_names=True)
            d = c1.min_distance_other(man)
            key = f"{nm}-{tag}"; mind[key] = min(mind.get(key, 1e9), d)
            if hit and not (nm == "lever" and tag == "ME"):                 # lever is rigid with the forearm group: designed contact
                for p in names: hits.setdefault(f"{nm}|{p[1]}", []).append(float(E))
        # crank vs coupler / lever (different layers; any hit is a real clash of the proxies)
        for a_, b_ in (("crank", "lever"), ("crank", "coupler"), ("coupler", "lever")):
            c1 = trimesh.collision.CollisionManager(); c1.add_object(a_, ln[a_]); c2 = trimesh.collision.CollisionManager(); c2.add_object(b_, ln[b_])
            d = c1.min_distance_other(c2); k = f"{a_}-{b_}"; mind[k] = min(mind.get(k, 1e9), d)
            if d <= 0.0: hits.setdefault(f"{a_}|{b_}", []).append(float(E))
    def rng(es):
        es = sorted(es); runs = []; s = es[0]; p = es[0]
        for e in es[1:]:
            if e - p > 1.5: runs.append((s, p)); s = e
            p = e
        runs.append((s, p)); return runs
    return {k: dict(n=len(v), ranges=rng(v)) for k, v in hits.items()}, {k: round(v, 2) for k, v in mind.items()}

def motor_change():
    G = ak.groups(); shell = trimesh.util.concatenate([G["M3"]["ua_lat"], G["M3"]["ua_med"]])
    O2 = fb.u(P["ground_dir_deg"]) * P["ground_g"]; orient = P["motor_orient"]
    hx, hz = (fb.XM_X / 2, fb.XM_Z / 2) if orient == 0 else (fb.XM_Z / 2, fb.XM_X / 2)
    cz = ZE + O2[1]; cx = O2[0]
    motor = trimesh.creation.box(extents=(2 * hx, fb.XM_D, 2 * hz)); motor.apply_translation((cx, 0, cz))
    pocket = trimesh.creation.box(extents=(2 * hx + 0.6, fb.XM_D + 0.6, 2 * hz + 0.6)); pocket.apply_translation((cx, 0, cz))
    old = G["M3"]["xm_elbow"]
    ua_vol = sum(abs(G["M3"][k].volume) for k in ("ua_lat", "ua_med"))
    res = dict(motor_center_xz=[round(float(cx), 2), round(float(cz), 2)], ua_shell_volume_mm3=round(float(ua_vol), 0))
    try:
        ov = trimesh.boolean.intersection([shell, pocket], engine="manifold")
        res["wall_in_new_pocket_mm3"] = round(float(abs(ov.volume)), 0); res["wall_in_new_pocket_pct_of_shell"] = round(100 * float(abs(ov.volume)) / ua_vol, 1)
    except Exception as e:
        res["boolean_error"] = str(e)[:120]
    # old elbow pocket: frame walls that become redundant (the box walls inside the ball around the old XM540)
    oldp = trimesh.creation.box(extents=(58.5 + 0.6, 44.0 + 0.6, 33.5 + 0.6)); oldp.apply_translation((0, 0, ZE))
    shell_ball = trimesh.creation.icosphere(subdivisions=4, radius=44.0); shell_ball.apply_translation((0, 0, ZE))
    try:
        ring = trimesh.boolean.intersection([shell, shell_ball], engine="manifold"); res["ball_belt_volume_mm3"] = round(float(abs(ring.volume)), 0)
        res["ball_belt_pct_of_shell"] = round(100 * abs(ring.volume) / ua_vol, 1)
    except Exception as e:
        res["ball_error"] = str(e)[:120]
    # tube clearance for the motor box: smallest wall-to-box gap along y over the motor z range (is the 44 mm depth available?)
    zs = np.linspace(cz - hz, cz + hz, 15); inner_y = []
    for z in zs:
        sec = shell.section(plane_origin=(0, 0, z), plane_normal=(0, 0, 1))
        if sec is None: inner_y.append(None); continue
        p = sec.vertices; inner_y.append((round(float(p[:, 1].min()), 1), round(float(p[:, 1].max()), 1), round(float(p[:, 0].min()), 1), round(float(p[:, 0].max()), 1)))
    res["shell_outer_extent_ymin_ymax_xmin_xmax_by_z"] = {f"{z:.0f}": v for z, v in zip(zs, inner_y)}
    motor.export(os.path.join(OUT, "xm540_relocated.stl"))
    return res, (cx, cz)

def seg_pt(p0, p1, q):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float); q = np.asarray(q, float); d = p1 - p0
    t = np.clip(np.dot(q - p0, d) / max(np.dot(d, d), 1e-9), 0, 1); return float(np.linalg.norm(p0 + t * d - q))

def layer_orders(pin_clear=9.0):
    """Which stacking orders (inner -> outer along +y) let every pin / hub / shaft pass the layers it crosses without meeting a bar?
    Pin axes: O2 (horn hub, from the horn face outwards to the crank layer), O4 (elbow shaft, from the drive cap to the lever layer),
    C (crank-coupler pin), L (coupler-lever pin). A pin crosses every layer that lies inner to the layer it ends in and outer to where it starts.
    pin_clear = bar half width 5 + pin/hub radius 4 (centre distance from the pin axis to the bar centreline)."""
    import itertools
    O2 = _R["O2"]; dist = {}
    for i in range(len(fb.E_HARD)):
        C = _R["C"][i]; L = _R["L"][i]
        seg = dict(crank=(O2, C), coupler=(C, L), lever=((0, 0), L)); pins = dict(O2=O2, O4=(0, 0), C=C, L=L)
        for ln, (p0, p1) in seg.items():
            for pn, q in pins.items():
                if (ln, pn) in (("crank", "O2"), ("crank", "C"), ("coupler", "C"), ("coupler", "L"), ("lever", "O4"), ("lever", "L")): continue     # own pivots
                dist[(ln, pn)] = min(dist.get((ln, pn), 1e9), seg_pt(p0, p1, q))
    out = []
    for order in itertools.permutations(("crank", "coupler", "lever")):
        pos = {n: i for i, n in enumerate(order)}
        # horn side: O2 hub starts at y=24.8 (inside everything) -> crosses layers inner to the crank; O4 shaft starts at the cap (inner to everything) -> crosses layers inner to the lever
        need = [("coupler", "O2")] if pos["coupler"] < pos["crank"] else []
        need += [("lever", "O2")] if pos["lever"] < pos["crank"] else []
        need += [("crank", "O4")] if pos["crank"] < pos["lever"] else []
        need += [("coupler", "O4")] if pos["coupler"] < pos["lever"] else []
        a, b = sorted((pos["crank"], pos["coupler"]))                       # C pin spans crank..coupler layers: crosses the layer between if any
        if b - a == 2 or (b - a == 1 and False): pass
        for n in order:
            if a < pos[n] < b and n != "crank" and n != "coupler": need.append((n, "C"))
        a, b = sorted((pos["coupler"], pos["lever"]))
        for n in order:
            if a < pos[n] < b and n != "coupler" and n != "lever": need.append((n, "L"))
        worst = min([dist[k] for k in need], default=1e9)
        out.append(dict(order_inner_to_outer=list(order), crossings=[f"{l} bar vs {p} axis" for l, p in need], min_centre_distance_mm=round(worst, 1) if need else None,
                        ok=bool(worst >= pin_clear)))
    return dict(pin_clear_mm=pin_clear, min_dist_bar_to_pin_mm={f"{k[0]}|{k[1]}": round(v, 1) for k, v in dist.items()}, orders=out)

if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    if "layers" in sys.argv:
        pk = json.load(open(os.path.join(OUT, "packaging.json"))); pk["layer_orders"] = layer_orders(); json.dump(pk, open(os.path.join(OUT, "packaging.json"), "w"), indent=1, default=str)
        print(json.dumps(pk["layer_orders"], indent=1)); sys.exit(0)
    res = dict(family=FAM, params=P)
    res["motor"], _ = motor_change(); print(json.dumps(res["motor"], indent=1))
    Es = np.arange(rl.HARD["E"][0], rl.HARD["E"][1] + 1e-9, 1.0)
    for stack in STACKS:
        hits, mind = run(stack, Es); res[stack] = dict(hits=hits, min_distance_mm=mind)
        print(stack, "hits:", json.dumps(hits), "\n   min dist:", mind, flush=True)
        for E in (0, 70, 145):
            ln, _ = links(E, stack); trimesh.util.concatenate(list(ln.values())).export(os.path.join(OUT, f"links_{stack}_E{E:03d}.stl"))
    # cable-loop anchors vs the relocated motor and the link stack
    ew = json.load(open(os.path.join(HERE, "..", "stl", "electronics", "wire_search_E_CB.json")))["found"]
    cx, cz = res["motor"]["motor_center_xz"]; hx = (fb.XM_X / 2) if P["motor_orient"] == 0 else fb.XM_Z / 2; hz = (fb.XM_Z / 2) if P["motor_orient"] == 0 else fb.XM_X / 2
    def boxd(p):
        d = np.maximum(np.abs(np.array(p) - np.array([cx, 0, cz])) - np.array([hx, fb.XM_D / 2, hz]), 0); return float(np.linalg.norm(d))
    res["cable_anchors"] = [dict(A=f["A"], B=f["B"], L=f["L"], A_to_new_motor_mm=round(boxd(f["A"]), 1), B_to_new_motor_mm=round(boxd(f["B"]), 1),
                                 chord_min_to_new_motor_mm=round(min(boxd(np.array(f["A"]) * (1 - t) + np.array(f["B"]) * t) for t in np.linspace(0, 1, 101)), 1),
                                 anchor_y_signs=[float(np.sign(f["A"][1])), float(np.sign(f["B"][1]))]) for f in ew]
    print(json.dumps(res["cable_anchors"], indent=1))
    json.dump(res, open(os.path.join(OUT, "packaging.json"), "w"), indent=1, default=str)
