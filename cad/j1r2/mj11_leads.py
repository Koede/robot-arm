"""MJ11 electrode relief slots + flex-lead routing model (SENSING / WING / SHOULDER, 2026-11-03).
Which atlas pairs lie under (or sweep under) the MJ11 dish liner, where each lead leaves the liner, how much service slack it needs.
Used by mj_concepts3.liner_mesh (geometry) and run as a script (analysis -> stl/j1r2_mounts/MJ11/leads.json).
usage: J1R2_ROOT=<work root> python3 mj11_leads.py
Frames: robot frame, mm, plan view = (y, z).  Skin motion model = muscle_atlas.travel (rigid-blade bound, pivot PIV, W_EFF) -> direction only here;
magnitude from stl/shoulder/emg_atlas.json 'travel_mm' (blended) and from the DESIGN_STATE s4 / SPEC_flex_electrode_leads s2.3 table (design values)."""
import os, sys, json, math
import numpy as np
from shapely.geometry import Point, LineString, Polygon, box as sbox
from shapely.ops import unary_union
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(os.path.dirname(HERE))
ATLAS = json.load(open(os.path.join(REPO, "stl", "shoulder", "emg_atlas.json")))
Y1, Z1, PAD_R, RAIL_EDGE_Y = -60.0, -190.0, 66.0, -96.0          # = mj_concepts3 (kept literal: this module is imported by it)
PIV = np.array([14.75, -116.22])                                  # muscle_atlas.PIV (y, z)
DISC_R = 6.0                                                      # 12 mm electrode disc
CLEAR = 1.5                                                       # slot margin around the swept footprint
ELEC_STACK_H = 3.5                                                # mm, electrode + 2 mm snap on the outer face: PLACEHOLDER (no vendor part selected)
RELIEF_DEPTH = 5.5                                                # skin-side relief in the 8 mm liner = stack 3.5 + 2.0 fit margin (mj_eval skin-mesh mismatch is -1.9 mm); 2.5 mm foam roof left
LEAD_OD = 1.6                                                     # mm, shielded 3-conductor 28 AWG silicone: assumption (SPEC_flex_electrode_leads s2.1, product not selected)
BEND_R = 10.0 * LEAD_OD                                           # >= 10 x OD (SPEC s2.2) = 16 mm
LOOP_R = 18.0                                                     # serpentine half-turn radius (SPEC s2.3 "+-18 mm")
CORRIDOR_W = 8.0                                                  # lead corridor width (the original channel width)
DESIGN_X = 1.25                                                   # slack / slot margin on skin travel (SPEC s2.3)
SPEC_TRAVEL = dict(trap_mid=56.0, rhomboid=62.0, trap_low=76.0, infraspin=70.0, teres_maj=96.0)   # SPEC_flex_electrode_leads s2.3 (follow fractions are assumptions); teres_maj = lateral border 96
KEYS = ("trap_mid", "trap_low", "rhomboid", "infraspin", "teres_maj", "lat_dorsi", "serratus")
OLD_CH_PSI = (225.0, 270.0, 315.0)
JACK_PSI = 240.0                                                  # jack plate on the lower-medial rim: arm-stage clearance >= 3 and skin gap >= 5 and rail / spine-bridge >= 3 only for psi 235..285 near the lead exits (jack_scan.json)
JACK_OFFSET = 15.0                                                # mm, header recess + strain relief

def domain_poly():
    return Point(Y1, Z1).buffer(PAD_R, 96).intersection(sbox(RAIL_EDGE_Y, Z1 - 200, Y1 + 200, Z1 + 200))
def dvec(y, z):
    """unit skin-displacement direction (y,z) at full elevation: rigid rotation about the atlas pivot, upward rotation moves points below the pivot lateral (+y)."""
    r = np.array([y, z]) - PIV; v = np.array([-r[1], r[0]]); return v / np.linalg.norm(v)
def pair(k):
    s = ATLAS["sites"][k]; a = np.array(s["a"][1:], float); b = np.array(s["b"][1:], float); return a, b, 0.5 * (a + b), s
def back_site(k):
    return ATLAS["sites"][k]["p"][0] > -170.0                 # x > -170: back-of-torso skin (flank/chest sites are not under the dish)
def jack_point():
    u = np.array([math.cos(math.radians(JACK_PSI)), math.sin(math.radians(JACK_PSI))]); return np.array([Y1, Z1]) + (PAD_R - 3.0) * u
def lead_path(k):
    """(Nx2 yz polyline) pair centre -> along -d to the liner boundary -> along the liner boundary (the 4 mm dish-rail gap on the medial side) the short way round -> jack point."""
    from shapely.ops import substring
    a, b, c, s = pair(k); d = dvec(*c); dom = domain_poly(); ring = dom.exterior
    ray = LineString([c, c - d * 250.0]).intersection(dom); g = ray if ray.geom_type == "LineString" else list(ray.geoms)[-1]
    ex = np.array(max(g.coords, key=lambda q: np.linalg.norm(np.array(q) - c))); j = jack_point()
    s0, s1, L_ = ring.project(Point(ex)), ring.project(Point(j)), ring.length
    fwd = (s1 - s0) % L_; bwd = (s0 - s1) % L_
    if fwd <= bwd: seg = substring(ring, s0, s0 + fwd) if s0 + fwd <= L_ else None
    else: seg = substring(ring, s0 - bwd, s0) if s0 - bwd >= 0 else None
    if seg is None:                                                          # wraps over the ring seam: join two pieces
        if fwd <= bwd: p1 = substring(ring, s0, L_); p2 = substring(ring, 0, (s0 + fwd) % L_); pts = list(p1.coords) + list(p2.coords)
        else: p1 = substring(ring, s0, 0); p2 = substring(ring, L_, s1); pts = list(p1.coords) + list(p2.coords)
    else: pts = list(seg.coords) if fwd <= bwd else list(seg.coords)[::-1]
    return np.vstack([c, ex, np.array(pts), j]), min(fwd, bwd)
def swept(k, travel, step=1.5):
    a, b, c, s = pair(k); d = dvec(*c)
    return unary_union([LineString([a + d * t, b + d * t]).buffer(DISC_R + CLEAR, 24) for t in np.arange(0.0, travel + 1e-6, step)])
def corridor(k):
    """lead corridor from the pair centre at park, along -d (the lateral half of the rim is swept by the arm stage: jack_scan.json), to beyond the liner rim, width CORRIDOR_W."""
    a, b, c, s = pair(k); d = dvec(*c); return LineString([c, c - d * 200.0]).buffer(CORRIDOR_W / 2.0, cap_style=2)
def slot_polys(mult=DESIGN_X, keys=("infraspin", "teres_maj", "rhomboid")):
    """name -> shapely polygon (plan view) of the skin-side relief cut in the liner: swept electrode footprint (atlas travel x mult) + lead corridor to the rim, clipped to the liner domain.
    rhomboid: footprint only (its lead leaves under the rail zone, not through the liner)."""
    dom = domain_poly(); out = {}
    for k in keys:
        sw = swept(k, ATLAS["sites"][k]["travel_mm"] * mult)
        if not sw.intersects(dom): continue
        poly = (sw if k == "rhomboid" else unary_union([sw, corridor(k)])).intersection(dom.buffer(2.0))
        if poly.area > 1.0: out[k] = poly
    return out

def analyse():
    dom = domain_poly(); res = dict(units="mm", pad=[Y1, Z1], R=PAD_R, rail_edge_y=RAIL_EDGE_Y, assumptions=dict(
        lead_od=LEAD_OD, bend_r_min=BEND_R, loop_r=LOOP_R, elec_stack_h_PLACEHOLDER=ELEC_STACK_H, relief_depth=RELIEF_DEPTH, design_margin=DESIGN_X,
        skin_model="muscle_atlas rigid-blade bound (PIV %s): direction; magnitude atlas travel_mm and SPEC s2.3 table" % PIV.tolist()), sites={})
    for k in KEYS:
        s = ATLAS["sites"][k]; a, b, c, _ = pair(k); d = dvec(*c); back = back_site(k)
        tr_atlas = s.get("travel_mm"); tr_spec = SPEC_TRAVEL.get(k, tr_atlas)
        r = dict(carrier=s["carrier"], on_back_skin=back, x_skin=round(s["p"][0], 1), centre_yz=np.round(c, 1).tolist(), under_rail_flag=bool(s.get("under_rail")), travel_atlas=tr_atlas, travel_spec=tr_spec,
                 d_dir=np.round(d, 3).tolist(), d_angle_psi=round(math.degrees(math.atan2(d[1], d[0])) % 360.0, 1))
        r["sd_disc_to_liner_edge_park"] = [round(min(PAD_R - math.hypot(p[0] - Y1, p[1] - Z1), p[0] - RAIL_EDGE_Y), 1) for p in (a, b)]     # + = inside the liner domain
        if back and tr_atlas is not None:
            for tag, tr in (("atlas", tr_atlas), ("spec", tr_spec)):
                sw = swept(k, tr); ov = sw.intersection(dom)
                r["swept_" + tag] = dict(area_mm2=round(sw.area), area_under_liner_mm2=round(ov.area), frac_under_liner=round(ov.area / sw.area, 3))
            hit = None
            for t in np.arange(0.0, max(tr_spec, 1) + 1e-6, 0.5):
                if LineString([a + d * t, b + d * t]).buffer(DISC_R).intersects(dom): hit = float(t); break
            r["travel_to_liner_contact"] = hit
            for sg, nm in ((-1.0, "minus_d"), (1.0, "plus_d")):
                ray = LineString([c, c + sg * d * 250.0]).intersection(dom)
                if ray.is_empty: continue
                g = ray if ray.geom_type == "LineString" else list(ray.geoms)[-1]; far = max(g.coords, key=lambda q: np.linalg.norm(np.array(q) - c)); p_out = np.array(far)
                psi_e = round(math.degrees(math.atan2(p_out[1] - Z1, p_out[0] - Y1)) % 360.0, 1); run = float(np.linalg.norm(p_out - c))
                r["exit_" + nm] = dict(yz=np.round(p_out, 1).tolist(), psi=psi_e, run_in_liner_mm=round(run, 1))
                if nm == "minus_d" and k in ("infraspin", "teres_maj"):
                    P_, bl = lead_path(k); tot = float(np.linalg.norm(np.diff(P_, axis=0), axis=1).sum())
                    r["exit_" + nm].update(boundary_to_jack_mm=round(bl, 1), path_to_jack_mm=round(tot + JACK_OFFSET, 1), path_yz=np.round(P_[::max(1, len(P_) // 12)], 1).tolist())
            r["old_channel_misalign_deg"] = {str(int(psi)): round(math.degrees(math.acos(float(np.clip(d @ np.array([math.cos(math.radians(psi)), math.sin(math.radians(psi))]), -1, 1)))), 1) for psi in OLD_CH_PSI}
            for tag, tr in (("atlas", tr_atlas), ("spec", tr_spec)):
                slack = DESIGN_X * tr; n = math.ceil(slack / ((math.pi - 2.0) * LOOP_R)); r["slack_" + tag] = dict(slack_mm=round(slack, 1), half_turns=n, loop_axial_mm=round(n * 2 * LOOP_R), loop_width_mm=round(2 * LOOP_R))
        res["sites"][k] = r
    sl = slot_polys(); cut = unary_union(list(sl.values())) if sl else Polygon()
    res["slots"] = {k: dict(area_mm2=round(p.area), bounds=np.round(p.bounds, 1).tolist()) for k, p in sl.items()}
    res["liner_area"] = dict(domain_mm2=round(dom.area), slots_union_mm2=round(cut.area), remaining_mm2=round(dom.area - cut.area), pressure_scale_same_load=round(dom.area / (dom.area - cut.area), 3))
    return res

if __name__ == "__main__":
    res = analyse(); out = os.path.join(REPO, "stl", "j1r2_mounts", "MJ11", "leads.json"); json.dump(res, open(out, "w"), indent=1)
    print(json.dumps(res, indent=1)); print("wrote", out)
