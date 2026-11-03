"""MJ11 electrode-lead study (SENSING / HARNESS / WING, 2026-11-03).  Robot frame, right arm, mm.
Which atlas electrodes lie under the MJ11 pad / dish / liner / rail / bolt bosses, how far each one slides at full elevation, what the 3 liner lead
channels can and cannot do, service-loop and bend-radius numbers, and a lead + jack model for the renders.
usage: J1R2_ROOT=<work root> python3 mj11_leads.py   -> stl/j1r2_mounts/MJ11/leads.json, MJ11_leads_R.stl
Every number marked ASSUMED is an input, not a measurement (lead OD, jack size, follow directions); see specs/SPEC_j1r2_mj11_leads_shear.md."""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import shapely.geometry as sg
from shapely.ops import unary_union
import clib as L, mj_frame as M, mj_concepts as C, mj_concepts3 as K
OUT = os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11"); os.makedirs(OUT, exist_ok=True)
ATL = json.load(open(os.path.join(M.REPO, "stl", "shoulder", "emg_atlas.json")))
Y1, Z1, R = K.Y1, K.Z1, C.PAD_R
SPINE_Y = -150.0
ELEC_R = 6.0                                   # 12 mm electrode disc (muscle_atlas)
PIV = np.array([14.75, -116.22]); W_EFF = 0.44  # muscle_atlas.py rigid-blade bound (pivot y,z; effective angle rad)
# ---- footprints in the (y, z) plane
disc = sg.Point(Y1, Z1).buffer(R, 96)
liner_fp = disc.intersection(sg.box(K.RAIL_EDGE_Y, Z1 - 200, Y1 + 200, Z1 + 200))                 # dish + liner (stop 4 mm lateral of the rail)
pad_fp = disc.intersection(sg.box(K.PAD_TRIM_Y, Z1 - 200, Y1 + 200, Z1 + 200))                    # pad disc trimmed flat at y -121
rail_fp = sg.box(SPINE_Y + 12, -440, SPINE_Y + 50, -80)                                           # wing rail y_rel 12..50 (scapular_frame.RAIL)
bosses = [(y, z) for z in K.BOLT_Z for y in K.BOLT_Y]                                              # PLACEHOLDER positions
boss_fp = unary_union([sg.Point(y, z).buffer(5.0) for (y, z) in bosses])                          # D10 spacers
RIGID = dict(pad=pad_fp, dish_liner=liner_fp, rail=rail_fp, boss_spacers=boss_fp)
# ---- skin follow at full elevation: rigid-blade rotation about PIV (muscle_atlas.travel), blend 0 on the paraspinal strip and below the inferior angle
lm = ATL["landmarks"]; IA_z = lm["IA"][2]; SB = ATL["SB"]
def follow(y, z, sign=+1):
    w = float(np.clip((y - SPINE_Y - 10.0) / max(SB - 10.0, 1.0), 0, 1)); f = float(np.clip(1.0 - (IA_z - z) / 80.0, 0, 1)) if z < IA_z else 1.0
    r = np.array([y, z]) - PIV; mag = W_EFF * np.linalg.norm(r) * w * f
    t = sign * np.array([-r[1], r[0]]) / max(np.linalg.norm(r), 1e-9)                  # unit tangent of a rotation about PIV
    if sign > 0 and t[0] < 0: t = -t                                                      # +1 branch: choose the tangent that moves the blade LATERALLY (+y); -1 is the opposite
    elif sign < 0 and t[0] > 0: t = -t
    return mag, t * mag
# design travel used by SPEC_flex_electrode_leads section 2.3 (follow-fraction assumptions) and its slack rule 1.25 x travel
SPEC_TRAVEL = dict(trap_mid=56.0, rhomboid=62.0, trap_low=76.0, infraspin=70.0, teres_maj=96.0)   # teres_maj = "lateral border" row
# ---- lead and jack assumptions
LEAD_OD = 2.2                  # ASSUMED: 3-conductor shielded 28 AWG silicone jacket
BEND_K = 10.0                  # SPEC_flex_electrode_leads 2.2: bend radius >= 10 x OD
R_MIN = BEND_K * LEAD_OD
SLACK_F = 1.25
CH_W, CH_H = 8.0, K.LINER_T    # liner channel: 8 wide (mj_concepts3), full liner thickness 8
JACK = dict(y=-108.0, z=-272.0, size=(24.0, 12.0, 8.0))   # ASSUMED 16-way keyed header block on the rail rear face, 52 mm below the lowest boss (PLACEHOLDER part)
def ch_axis(psi): a = math.radians(psi); return np.array([math.cos(a), math.sin(a)])
def seg_dist(p, a, b):
    ab = b - a; t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1); q = a + t * ab; return float(np.linalg.norm(p - q)), q, float(t)
def run():
    res = dict(units="mm", pad=(Y1, Z1, R), notes={}, sites={}, channels={}, jack=JACK, lead=dict(OD=LEAD_OD, R_min=R_MIN))
    for name, s in ATL["sites"].items():
        if s["carrier"].startswith("clasp") or s["p"][0] < -170 or s["p"][1] > 10 or name in ("delt_mid", "delt_post", "delt_ant", "serratus", "pec_clav", "pec_stern"):
            continue
        ends = [np.array(s["a"][1:]), np.array(s["b"][1:])]; mid = 0.5 * (ends[0] + ends[1])
        rows = {}
        for k, geo in RIGID.items():
            ds = [float(geo.distance(sg.Point(*e)) - ELEC_R) for e in ends]          # edge-to-edge distance, electrode disc to the rigid footprint (negative = under it)
            ins = [bool(geo.contains(sg.Point(*e))) or d <= 0 for e, d in zip(ends, ds)]
            rows[k] = dict(edge_dist_mm=[round(max(d, -99), 1) for d in ds], discs_under=int(sum(ins)))
        mag, dvec = follow(mid[0], mid[1])
        under_liner = rows["dish_liner"]["discs_under"]
        rec = dict(mid_yz=[round(float(v), 1) for v in mid], pair_length=round(float(np.linalg.norm(ends[0] - ends[1])), 1), rigid=rows,
                   follow_bound_mm=round(mag, 1), follow_dir_yz=[round(float(v), 3) for v in (dvec / max(mag, 1e-9))], atlas_travel_mm=s.get("travel_mm"),
                   design_travel_mm=SPEC_TRAVEL.get(name))
        rec["under_rigid"] = sorted(k for k, v in rows.items() if v["discs_under"] > 0)
        rec["within_20mm_rigid"] = sorted(k for k, v in rows.items() if min(v["edge_dist_mm"]) < 20.0 and v["discs_under"] == 0)
        res["sites"][name] = rec
    # ---- channel capture: how long a lead that rides on the base layer stays inside its (fixed) 8 mm channel while the skin follows
    for psi in K.LEAD_PSI:
        ax = ch_axis(psi); a = np.array([Y1, Z1]) + 18 * ax; b = np.array([Y1, Z1]) + 80 * ax
        ln = sg.LineString([a, b]).intersection(liner_fp); L_in = float(ln.length)
        res["channels"][str(int(psi))] = dict(axis=[round(float(v), 3) for v in ax], length_in_liner=round(L_in, 1), width=CH_W, height=CH_H,
            exit_yz=[round(float(v), 1) for v in np.array(ln.coords[-1])] if L_in > 0 else None, sites_in_reach={})
        for name in ("infraspin", "teres_maj"):
            r_ = res["sites"][name]; m = np.array(r_["mid_yz"]); d, q, t = seg_dist(m, a, a + (np.array(ln.coords[-1]) - a))
            ang = {}
            for sg_ in (+1, -1):
                mag, dv = follow(m[0], m[1], sg_); cross = abs(dv[0] * ax[1] - dv[1] * ax[0])                 # follow component across the channel
                ang[str(sg_)] = dict(along=round(float(np.dot(dv, ax)), 1), across=round(float(cross), 1))
            res["channels"][str(int(psi))]["sites_in_reach"][name] = dict(dist_site_to_axis=round(d, 1), follow_components=ang)
    allow = (CH_W - LEAD_OD) / 2.0
    res["channel_lateral_allowance_mm"] = round(allow, 2)
    # ---- exits and free spans (lead couched in the base layer to an anchor just outside the liner, then a free span with a service loop to the jack)
    jack = np.array([JACK["y"], JACK["z"]]); exits = {psi: np.array(res["channels"][psi]["exit_yz"]) for psi in ("225", "270") if res["channels"][psi]["exit_yz"]}
    lead_rows = {}
    for name in ("infraspin", "teres_maj"):
        r_ = res["sites"][name]; m = np.array(r_["mid_yz"]); best = None
        for psi, ex in exits.items():
            Lc = float(np.linalg.norm(ex - m)); Lf = float(np.linalg.norm(jack - ex)); tot = Lc + Lf
            if best is None or tot < best["total"]: best = dict(channel=psi, exit=[round(float(v), 1) for v in ex], couched=round(Lc, 1), free_span=round(Lf, 1), total=round(tot, 1))
        mag_a = follow(*best["exit"])[0]                                                                   # the anchor sits on the moving base layer outside the rim
        trav = r_["design_travel_mm"]; slack = SLACK_F * trav
        best.update(anchor_follow_bound_mm=round(mag_a, 1), design_travel_mm=trav, slack_required=round(slack, 0), loop_R_from_slack=round(slack / (2 * math.pi), 1),
                    loop_R_used=round(max(slack / (2 * math.pi), R_MIN), 1), loop_len_at_R_used=round(2 * math.pi * max(slack / (2 * math.pi), R_MIN), 0),
                    loop_outer_diameter=round(2 * (max(slack / (2 * math.pi), R_MIN) + LEAD_OD / 2 + 3.0), 1))
        lead_rows[name] = best
    res["leads"] = lead_rows
    # a 4-half-turn serpentine of radius 18 (the spec's "+/-18 mm accordion") violates the bend rule
    res["spec_serpentine_R18_ok"] = bool(18.0 >= R_MIN); res["serpentine_slack_per_halfturn_at_Rmin"] = round((math.pi - 2) * R_MIN, 1)
    # electrode at the FULL design travel vs the channel: the lead dragged across the 8 mm channel
    res["fixed_channel_verdict"] = "FAIL: follow across the channel axis exceeds the %.1f mm allowance for both liner-covered pairs under either follow sign" % allow
    # ---- foam ridge: a lead of thickness t under the 8 mm foam, Winkler E_FOAM (project model, 0.15 MPa), extra local pressure = E t / 8
    res["ridge_kPa"] = {f"{t:.2f}mm": round(M.E_FOAM * 1e3 * t / CH_H, 1) for t in (0.35, 1.0, LEAD_OD)}
    json.dump(res, open(os.path.join(OUT, "leads.json"), "w"), indent=1)
    return res
# ------------------------------------------------------------------ geometry for the renders
def tube(pts, r, n=10):
    ms = []
    for a, b in zip(pts[:-1], pts[1:]):
        v = b - a; Lh = np.linalg.norm(v)
        if Lh < 1e-6: continue
        c = trimesh.creation.cylinder(r, Lh, sections=n); T = trimesh.geometry.align_vectors([0, 0, 1], v / Lh); T[:3, 3] = (a + b) / 2; c.apply_transform(T); ms.append(c)
        s = trimesh.creation.icosphere(1, r); s.apply_translation(b); ms.append(s)
    return trimesh.util.concatenate(ms)
def skin_x(y, z): return float(M.back_x(y, z))
def lead_geo(res):
    """base-layer couched run (skin + 1.2), free span with a service loop (skin + 4, loop plane parallel to the back), jack block on the rail rear face."""
    parts = {}
    for name, ld in res["leads"].items():
        m = np.array(res["sites"][name]["mid_yz"]); ex = np.array(ld["exit"]); jk = np.array([JACK["y"], JACK["z"]]); Rl = ld["loop_R_used"]
        pts2 = [m, ex]
        d = jk - ex; Ld = np.linalg.norm(d); u = d / Ld; n_ = np.array([-u[1], u[0]]) * (-1 if name == "teres_maj" else 1)
        # service loop: lead leaves the line, circles once (radius Rl) tangent to the line, returns; drawn as a circle on the free span
        c0 = ex + u * min(Ld * 0.55, Ld - 5.0) + n_ * Rl * 0.0
        loop = [c0 + Rl * np.array([math.cos(t), math.sin(t)]) * 1.0 + n_ * Rl for t in np.linspace(-math.pi / 2, 3 * math.pi / 2, 40)]
        run_ = [np.array(p) for p in pts2] + [c0] + loop + [c0, jk]
        P3 = [np.array([skin_x(p[0], p[1]) + (1.2 if i < 2 else 4.0), p[0], p[1]]) for i, p in enumerate(run_)]
        parts[name] = tube(P3, LEAD_OD / 2.0 if True else 1.0)
    jb = L.box(K.rail_x(-119, JACK["z"]) , K.rail_x(-119, JACK["z"]) + JACK["size"][2], JACK["y"] - JACK["size"][0] / 2, JACK["y"] + JACK["size"][0] / 2, JACK["z"] - JACK["size"][1] / 2, JACK["z"] + JACK["size"][1] / 2)
    parts["jack_block"] = jb
    return parts
if __name__ == "__main__":
    r = run(); print(json.dumps({k: r[k] for k in ("channels", "leads", "channel_lateral_allowance_mm", "ridge_kPa", "spec_serpentine_R18_ok", "serpentine_slack_per_halfturn_at_Rmin")}, indent=1))
    for n, s in r["sites"].items(): print("%-10s mid=%s follow=%s spec=%s under=%s near20=%s" % (n, s["mid_yz"], s["follow_bound_mm"], s["design_travel_mm"], s["under_rigid"], s["within_20mm_rigid"]))
    g = lead_geo(r)
    for k, m in g.items(): m.export(os.path.join(OUT, f"MJ11_leadmodel_{k}_R.stl"))
