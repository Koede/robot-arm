"""Precise shoulder-girdle sEMG muscle atlas (SENSING, rev 2026-10-14).  Right side, robot frame, mm.

Every bipolar PAIR is defined by (1) a landmark rule Forest can reproduce with a tape measure, (2) a fibre direction, (3) an inter-electrode
distance IED (20 mm centre-to-centre of the 12 mm discs, SENIAM-typical), (4) a carrier (rigid clasp pad on a stable zone / compression base
layer).  Sites are ray-cast onto the wearer mesh, so measured landmarks in LANDMARK_OVERRIDE regenerate the whole map (a re-fit, not a redesign).

SOURCES.  SENIAM's own pages were unreachable (also in earlier sessions), so the site rules below are the textbook / secondary-table versions:
  [B18] BMC Musculoskelet Disord 2018 (PMC6106764): upper trapezius at 50 % of the C7-acromion line, along the line
  [B20] BMC Sports Sci Med Rehabil 2020          : upper trapezius 2 cm lateral to the C7-lateral-acromion midpoint, IED 20 mm
  [Cr]  Cram, Introduction to Surface EMG        : deltoid thirds, infraspinatus, lower trapezius, serratus anterior, latissimus, pectoralis major
  [IZ]  innervation-zone literature (Rainoldi/Merletti group): keep the pair away from the innervation zone (IZ), between IZ and tendon, IED << IZ-tendon distance
  [TB]  textbook rule, no source retrieved this session (flagged in the spec): trap_mid, rhomboid, teres_maj, pec_clav
SCAPULAR BONE LANDMARKS (spine root SSR, inferior angle IA) are not on the skin mesh; they are ESTIMATED from skin landmarks with the rules below
(LM_EST) and carry the flag 'estimated'.  The skin-travel figure is a RIGID-BLADE BOUND with a pivot calibrated to three figures in DESIGN_STATE s4.
"""
import os, sys, json, numpy as np, trimesh
from shapely.geometry import LineString, Point
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import bodyprobe as bp
OUT = os.path.join(HERE, "..", "stl", "shoulder")
lm = json.load(open(os.path.join(OUT, "proxies", "wearer_landmarks.json")))
SP = lm["spine_y_rel_P"]; GH = np.array(lm["gh_joint_rel_P"]); ACR = np.array(lm["acromion_top_rel_P"]); NECK = np.array(lm["neck_joint_rel_P"])
H = lm["height_mm"]; S = H / 1750.0
body = bp.load()
IED = 20.0
LANDMARK_OVERRIDE = {}          # name -> [x, y, z] skin-surface mm, from Forest's tape-measure fit (I7)

def unit(v): v = np.asarray(v, float); return v / np.linalg.norm(v)
def lerp(a, b, t): return np.asarray(a, float) + (np.asarray(b, float) - np.asarray(a, float)) * t
def hit(o, d):
    h = bp.cast(body, o, d); return None if h is None else h[0]
def back(y, z): return hit([200.0, y, z], [-1, 0, 0])            # skin of the back (from +x looking -x)
def front(y, z): return hit([-500.0, y, z], [1, 0, 0])           # skin of the chest

# torso cross-section polygon at height z (the arm is a separate polygon below the axilla)
def torso_poly(z):
    s = body.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1]); p2, T = s.to_2D()
    for pg in p2.polygons_full:
        c = trimesh.transform_points(np.array([[*pg.representative_point().coords[0], 0.0]]), T)[0]
        pts = trimesh.transform_points(np.c_[np.array(pg.exterior.coords), np.zeros(len(pg.exterior.coords))], T)
        from shapely.geometry import Polygon
        P = Polygon(pts[:, :2])          # (x, y) of the slice
        if P.contains(Point(-200.0, SP)): return P
    return None
def torso_side(z):
    """lateral skin y of the right flank and AP mid-depth x at height z (below the axilla, arm-free)"""
    P = torso_poly(z); x0, y0, x1, y1 = P.bounds; xm = 0.5 * (x0 + x1)
    ln = LineString([(xm, y0 - 10), (xm, y1 + 10)]).intersection(P)
    return float(ln.bounds[3]), xm

# ------------------------------------------------------------------------------------------------ landmarks
L = {}
L["C7"] = back(SP, NECK[2])                                      # ESTIMATED: neck-base joint height (+-15 mm)
L["T3"] = back(SP, L["C7"][2] - 75.0 * S)                        # ESTIMATED: 3 spinous levels x 25 mm
L["T7"] = back(SP, L["C7"][2] - 175.0 * S)                       # ESTIMATED: C7->T7 skin arc ~175 mm
L["ACR_top"] = ACR.copy(); L["ACR_lat"] = ACR + np.array([0.0, 4.0, -4.0])
SB = 62.0 * S                                                    # rest distance spinous line -> medial border (Kibler/LSST rest values 60-90 mm)
L["SSR"] = back(SP + 0.90 * SB, L["T3"][2])                      # ESTIMATED scapular-spine root
L["IA"] = back(SP + 1.15 * SB, L["T7"][2] + 8.0 * S)             # ESTIMATED inferior angle (T7-T8)
L["AA"] = hit([200.0, L["ACR_lat"][1] - 8.0, L["ACR_lat"][2] - 8.0], [-1, 0, 0])   # posterior acromial angle (skin)
L["NOTCH"] = front(SP, L["C7"][2] - 45.0 * S)                    # ESTIMATED suprasternal notch
ah = np.radians(lm["arm_hang_deg"]); epi_dir = unit([0, np.sin(ah), -np.cos(ah)])      # the wearer mesh is re-posed to a 10 deg hang (DESIGN_STATE s1)
L["EPI_lat"] = GH + epi_dir * (0.186 * H)                        # lateral epicondyle (humerus 0.186 H, Drillis-Contini), 10 deg hang
for k, v in LANDMARK_OVERRIDE.items(): L[k] = np.asarray(v, float)
LM_ESTIMATED = ["C7", "T3", "T7", "SSR", "IA", "NOTCH", "EPI_lat"]

# ------------------------------------------------------------------------------------------------ muscle table
M = {}
def add(name, anchor, outward, fibre, rule, carrier, dof, src, ied=IED, iz="", note="", flags=(), exact=False):
    d = unit(outward)
    if exact: p = np.asarray(anchor, float)                       # point already on the skin (torso-slice sites; a ray would hit the arm first)
    else: p = bp.cast(body, np.asarray(anchor, float) + d * 600.0, -d)[0]
    n = bp.smooth_normal(body, p, 14.0)
    f = unit(fibre); u = f - n * (f @ n); u = unit(u); v = np.cross(n, u)
    M[name] = dict(p=p, n=n, u=u, v=v, a=p - u * ied / 2, b=p + u * ied / 2, ied=ied, rule=rule, carrier=carrier, dof=dof, src=src, iz=iz, note=note, flags=list(flags))

# upper trapezius: 50 % C7 -> lateral acromion, along the line [B18]; the belly's IZ is near mid-belly, so the pair is centred 2 cm lateral of the midpoint [B20]
ut = lerp(L["C7"], L["ACR_lat"], 0.5); ut = ut + unit(L["ACR_lat"] - L["C7"]) * 20.0
add("trap_up", ut, (0.1, 0, 1), L["ACR_lat"] - L["C7"], "midpoint of C7->lateral acromion, +20 mm lateral (distal of the IZ) [B18][B20]; pair along the C7-acromion line",
    "clasp F1 (rigid pad, stable zone)", "scapular elevation (S2 up)", "[B18][B20]", iz="mid-belly IZ; pair lateral of it")
# middle trapezius: horizontal fibres at T3 level, midway between the T3 spinous process and the medial border [TB]
mt = np.array([0, lerp(SP, L["SSR"][1], 0.5), L["T3"][2] + 10.0 * S])
add("trap_mid", mt, (1, 0, 0), (0, 1, 0), "midway between the T3 spinous line and the medial border (root of the spine), +10 mm cranial; fibres horizontal [TB]",
    "base layer (under the rail foam)", "scapular retraction", "[TB]", flags=["unsourced"])
# lower trapezius: 1/3 of the way from the T7 spinous process to the scapular-spine root, fibres oblique to the root [Cr]
lt = lerp(L["T7"], L["SSR"], 1.0 / 3.0)
add("trap_low", lt, (1, 0, 0), L["SSR"] - lt, "1/3 of the way T7 spinous -> scapular-spine root (SSR); fibres run obliquely up and lateral to the root [Cr]",
    "base layer (under the rail foam)", "scapular depression / upward rotation", "[Cr]", flags=["bone landmark estimated"])
# rhomboid major: midway between the spinous line and the medial border, a third of the way down the border from the root; fibres down and lateral [TB]
rh = np.array([0, lerp(SP, L["SSR"][1] + (L["IA"][1] - L["SSR"][1]) / 3.0, 0.5), lerp(L["SSR"][2], L["IA"][2], 1.0 / 3.0)])
add("rhomboid", rh, (1, 0, 0), (0, 1, -0.55), "midway between spinous line and medial border, 1/3 down the border from the root; fibres down-and-lateral [TB]",
    "base layer (under the rail foam)", "scapular retraction", "[TB]", flags=["unsourced", "crosstalk mid/low trap"])
# infraspinatus: 40 mm below the scapular spine over the lateral infraspinous fossa, parallel to the spine [Cr]
inf = lerp(L["SSR"], L["AA"], 0.62) + np.array([0, 0, -40.0 * S])
add("infraspin", inf, (1, 0, 0), (0, 1, 0.25), "40 mm below the scapular spine, 62 % of the way SSR->AA (lateral fossa); fibres ~parallel to the spine [Cr]",
    "base layer", "shoulder external rotation", "[Cr]")
# teres major: 25 mm lateral and 20 mm cranial of the inferior angle on the lateral border [TB]
tm = L["IA"] + np.array([0, 28.0 * S, 22.0 * S])
add("teres_maj", tm, (1, 0, 0), (0, 1, 0.45), "lateral scapular border, 28 mm lateral and 22 mm cranial of the inferior angle; fibres up-and-lateral to the humeral crest [TB]",
    "base layer", "shoulder internal rotation / extension", "[TB]", flags=["unsourced", "crosstalk lat dorsi/infraspin"])
# latissimus dorsi: 40 mm below the inferior angle, halfway between the spine and the lateral edge of the torso, oblique ~45 deg [Cr]
ysd, xm = torso_side(L["IA"][2] - 40.0 * S)
ld = np.array([0, 0.5 * (SP + ysd), L["IA"][2] - 40.0 * S])
add("lat_dorsi", ld, (1, 0, 0), (0, 0.7, 0.7), "40 mm below the inferior angle, halfway between the spinous line and the lateral torso edge; fibres up-and-lateral ~45 deg [Cr]",
    "base layer", "shoulder extension / adduction", "[Cr]", flags=["bone landmark estimated"])
# serratus anterior: 7th intercostal space, mid-axillary line, anterior to the latissimus border, fibres along the rib slope [Cr]
zs = L["IA"][2] + 5.0 * S; ys, xs = torso_side(zs)
add("serratus", np.array([xs, ys, zs]), (0, 1, 0), (0.55, 0, 0.83), "7th intercostal space (inferior-angle height), mid-axillary line; fibres follow the rib slope (posterior-superior) [Cr]",
    "base layer (flank panel)", "scapular protraction", "[Cr]", flags=["bone landmark estimated"], exact=True)
# deltoid thirds
ad = np.array([ACR[0] - 20.0 * S, ACR[1] + 6.0, ACR[2] - 20.0 * S])
add("delt_ant", ad, (-1, 0, 0), epi_dir, "one finger-width (20 mm) distal and anterior to the acromion; fibres along the humerus [Cr]", "clasp F2 (rigid pad)", "shoulder flexion", "[Cr]",
    iz="deltoid has several IZs; pair is in the proximal-third belly above the tendon, flagged for the movement check")
md = lerp(L["ACR_lat"], L["EPI_lat"], 0.26) + np.array([0, 18.0 * S, 0])
add("delt_mid", md, (0, 1, 0), epi_dir, "26 % of the acromion->lateral-epicondyle line (~91 mm: pair spans 81-101 mm, distal of mid-belly, >20 mm proximal of the tuberosity at ~124 mm) [Cr][IZ]", "base layer sleeve", "shoulder abduction", "[Cr][IZ]")
pd_ = np.array([ACR[0] + 35.0 * S, ACR[1] + 6.0, ACR[2] - 22.0 * S])
add("delt_post", pd_, (1, 0, 0), epi_dir, "two finger-widths (35 mm) behind the angle of the acromion; fibres toward the humerus [Cr]", "base layer sleeve", "shoulder extension", "[Cr]")
# pectoralis major, two heads
nt = L["NOTCH"]; cl40 = lerp(nt, L["ACR_top"], 0.40) + np.array([0, 0, -25.0 * S])
add("pec_clav", cl40, (-1, 0, 0), (0, 1, -0.45), "clavicular head: 40 % along the clavicle from the sternal notch, 25 mm below it; fibres down-and-lateral to the humerus [TB]",
    "clasp F3 (rigid pad)", "shoulder flexion (clavicular head)", "[TB]", flags=["unsourced"])
yaal = torso_side(GH[2] - 100.0)[0] - 8.0 * S                    # anterior axillary line ~ flank edge
ps = np.array([0, yaal - 35.0 * S, GH[2] - 25.0 * S])
add("pec_stern", ps, (-1, 0, 0), (0, 1, -0.75), "sternal head: 35 mm medial to the anterior axillary line, at GH height - 25 mm; fibres oblique to the humerus [Cr]",
    "base layer (chest panel)", "shoulder adduction / internal rotation", "[Cr]")

# ------------------------------------------------------------------------------------------------ skin-travel bound (rigid blade)
PIV = np.array([14.75, -116.22]); W_EFF = 0.44          # (y, z) pivot and effective angle (rad): reproduces the project's 56/62/76 mm at (SP+45,-160),(SP+55,-205),(SP+55,-250)
def travel(p):
    """skin travel at full elevation (mm): rigid-blade bound, blended to 0 over the paraspinal strip (SP .. SP+SB) and 0 off the blade"""
    y, z = p[1], p[2]
    if p[0] < -170.0 or y > GH[1] + 5.0: return None                  # not back-of-blade skin (front, flank, arm): the blade model does not apply
    w = np.clip((y - SP - 10.0) / max(SB - 10.0, 1.0), 0.0, 1.0)
    f = np.clip(1.0 - (L["IA"][2] - z) / 80.0, 0.0, 1.0) if z < L["IA"][2] else 1.0
    return float(W_EFF * np.hypot(y - PIV[0], z - PIV[1]) * w * f)


# ------------------------------------------------------------------------------------------------ muscle REGIONS (rev 2026-10-14b)
# Each muscle gets a skin region = ellipse (length along the fibres x width across) centred on the rule-based belly point.  Dimensions are
# approximate surface-projection sizes of the belly (mm at S=1; scaled by S), [TB] -- NOT retrieved from a source, flagged in the spec.
# The pair is then slid inside its region to maximise the clearance to the neighbouring muscles' pairs (crosstalk), keeping a margin to the region edge.
REG = dict(trap_up=(150, 40), trap_mid=(90, 40), trap_low=(110, 40), rhomboid=(90, 35), infraspin=(110, 60), teres_maj=(80, 30), lat_dorsi=(160, 70),
           serratus=(100, 60), delt_ant=(100, 45), delt_mid=(110, 50), delt_post=(100, 45), pec_clav=(110, 40), pec_stern=(120, 60))
FIXED = {"trap_up", "delt_ant", "pec_clav", "serratus"}          # clasp facets (pressure design) and the exact flank-slice site do not slide
def skin_from(q, n):
    h = bp.cast(body, np.asarray(q, float) + n * 60.0, -n); return None if h is None else h[0]
def reframe(k, p):
    m = M[k]; n = bp.smooth_normal(body, p, 14.0); f = m["u"]; u = unit(f - n * (f @ n)); v = np.cross(n, u)
    m.update(p=p, n=n, u=u, v=v, a=p - u * m["ied"] / 2, b=p + u * m["ied"] / 2)
def region_outline(k, nseg=28):
    m = M[k]; l, w = REG[k]; l *= S; w *= S; pts = []
    c0, u, v, n = m["region_c"], m["u"], m["v"], m["n"]
    for t in np.linspace(0, 2 * np.pi, nseg, endpoint=False):
        tgt = c0 + u * (l / 2) * np.cos(t) + v * (w / 2) * np.sin(t); q = skin_from(tgt, n)
        pts.append(q if (q is not None and np.linalg.norm(q - tgt) < 10.0) else tgt)     # off-body or wrong-surface hits fall back to the tangent-plane point
    return np.array(pts)
def in_region(k, q):
    m = M[k]; l, w = REG[k]; d = q - m["region_c"]; return (d @ m["u"] / (l * S / 2)) ** 2 + (d @ m["v"] / (w * S / 2)) ** 2
def optimise_sites(iters=4):
    for k in M: M[k]["region_c"] = np.asarray(M[k].get("rc0", M[k]["p"]), float).copy()
    log = {}
    for _ in range(iters):
        for k in M:
            if k in FIXED: continue
            m = M[k]; l, w = REG[k]; c0, u, v, n = m["region_c"], m["u"], m["v"], m["n"]
            others = [M[j]["p"] for j in M if j != k]; best = None
            for su in np.linspace(-1, 1, 21):
                for sv in np.linspace(-1, 1, 9):
                    q = skin_from(c0 + u * su * (l * S / 2 - m["ied"] / 2 - 8.0) + v * sv * (w * S / 2 - 8.0), n)
                    if q is None or in_region(k, q) > 0.5: continue            # pair centre in the inner half of the region
                    if q[0] > -150 and q[1] < SP + 20.0 * S and k not in ('pec_stern',): continue   # not on the spinous line / aponeurosis (back sites stay >= 20 mm off the midline)
                    # both electrodes must stay inside the region
                    if max(in_region(k, q - u * m["ied"] / 2), in_region(k, q + u * m["ied"] / 2)) > 1.0: continue
                    dmin = min(float(np.linalg.norm(q - o)) for o in others)
                    off = float(np.linalg.norm(q - c0)); rail = SP + 6 <= q[1] <= SP + 50 and -400 <= q[2] <= -80
                    mov = (travel(q) or 0.0)
                    score = min(dmin, 50.0) - 0.6 * off - (6.0 if rail else 0.0) - 0.05 * mov
                    if best is None or score > best[0]: best = (score, q, dmin, off)
            if best is not None: reframe(k, best[1]); log[k] = (round(best[2], 1), round(best[3], 1))
    return log

if __name__ == "__main__":
    cj0 = lambda x: np.round(x, 1).tolist()
    # ---- facet audit: the clasp pads are fixed by the load/pressure design (F1 margin 0.7 %), so the three clasp channels sit WHERE THE FACET IS;
    #      the rule-ideal site is kept for comparison and the deviation is reported (and gated: <= 12 mm and <= 45 deg to the fibre line)
    if "--rules" in sys.argv:   # pre-clasp pass: publish the rule-ideal sites so clasp_shoulder.py can seat F2/F3 on them (atlas = single source of truth)
        json.dump({ch: dict(p=np.round(M[ch]["p"], 1).tolist(), u=np.round(M[ch]["u"], 3).tolist(), n=np.round(M[ch]["n"], 3).tolist()) for ch in ("trap_up", "delt_ant", "pec_clav")},
                  open(os.path.join(OUT, "emg_rules.json"), "w"), indent=1); print("wrote emg_rules.json"); sys.exit(0)
    FAC = json.load(open(os.path.join(OUT, "clasp_sites.json"))); AUD = {}
    for ch, fk in (("trap_up", "F1_upper_trap"), ("delt_ant", "F2_ant_delt"), ("pec_clav", "F3_pec")):
        f = FAC[fk]; fp, fu, fn = np.array(f["p"]), np.array(f["u"]), np.array(f["n"]); m = M[ch]
        dev = float(np.linalg.norm(fp - m["p"])); ang = float(np.degrees(np.arccos(min(1.0, abs(fu @ m["u"])))))
        AUD[ch] = dict(facet=fk, ideal_p=cj0(m["p"]), ideal_u=cj0(m["u"]), facet_p=cj0(fp), facet_u=cj0(fu), deviation_mm=round(dev, 1), axis_angle_deg=round(ang, 1))
        m.update(p=fp, u=fu, n=fn, a=fp - fu * IED / 2, b=fp + fu * IED / 2)
    ln = unit(L["ACR_lat"] - L["C7"]); rel = M["trap_up"]["p"] - L["C7"]
    AUD["trap_up"]["frac_C7_to_acromion"] = round(float(rel @ ln / np.linalg.norm(L["ACR_lat"] - L["C7"])), 3)
    AUD["trap_up"]["perp_offset_mm"] = round(float(np.linalg.norm(rel - (rel @ ln) * ln)), 1)
    cl = L["ACR_top"] - L["NOTCH"]; r3 = M["pec_clav"]["p"] - L["NOTCH"]; t3 = float(r3 @ cl / (cl @ cl))
    AUD["pec_clav"]["frac_clavicle_from_notch"] = round(t3, 3); AUD["pec_clav"]["below_clavicle_mm"] = round(float(-(r3 - t3 * cl)[2]), 1)
    for ch in ("trap_up", "delt_ant", "pec_clav"): M[ch]["rc0"] = np.array(AUD[ch]["ideal_p"])
    pre = {k: M[k]["p"].copy() for k in M}
    SLIDE = optimise_sites()
    for k in M:
        M[k]["slide_mm"] = round(float(np.linalg.norm(M[k]["p"] - pre[k])), 1); M[k]["region_len_mm"], M[k]["region_wid_mm"] = round(REG[k][0] * S, 0), round(REG[k][1] * S, 0)
        M[k]["region_outline"] = np.round(region_outline(k), 1).tolist(); M[k]["region_c"] = np.round(M[k]["region_c"], 1).tolist()
        M[k]["in_region"] = bool(in_region(k, M[k]["p"]) <= 1.0)
    for ch in ("trap_up", "delt_ant", "pec_clav"):
        AUD[ch]["facet_in_region"] = bool(in_region(ch, M[ch]["p"]) <= 1.0); AUD[ch]["region_r"] = round(float(np.sqrt(in_region(ch, M[ch]["p"]))), 2)
    # ---- rail footprint: which sites sit under the wing rail foam (y SP+6..SP+50, z -400..-80)
    for k, m in M.items(): m["under_rail"] = bool(SP + 6 <= m["p"][1] <= SP + 50 and -400 <= m["p"][2] <= -80)
    names = list(M)
    for k in names: M[k]["travel_mm"] = (0.0 if M[k]["carrier"].startswith("clasp") else (None if travel(M[k]["p"]) is None else round(travel(M[k]["p"]), 1)))
    C = np.array([M[k]["p"] for k in names]); D = np.linalg.norm(C[:, None] - C[None], axis=2); np.fill_diagonal(D, 1e9)
    for i, k in enumerate(names):
        j = int(np.argmin(D[i])); M[k]["nearest"] = names[j]; M[k]["nearest_mm"] = round(float(D[i, j]), 1)
    cj = lambda x: np.round(x, 1).tolist() if isinstance(x, np.ndarray) else x
    json.dump(dict(landmarks={k: cj(v) for k, v in L.items()}, estimated=LM_ESTIMATED, S=S, SB=SB, facet_audit=AUD,
                   sites={k: {kk: cj(vv) for kk, vv in m.items() if kk != "rc0"} for k, m in M.items()}), open(os.path.join(OUT, "emg_atlas.json"), "w"), indent=1)
    for k, v in L.items(): print("LM %-8s %s%s" % (k, np.round(v, 1), "  (estimated)" if k in LM_ESTIMATED else ""))
    for k, m in M.items():
        print("%-10s p=%-26s u=%-22s trav=%s nearest=%s %.0f mm  %s" % (k, np.round(m["p"], 1), np.round(m["u"], 2), str(m["travel_mm"]), m["nearest"], m["nearest_mm"], m["carrier"]))
