"""Faceted shoulder clasp with inward-facing sEMG sensor pads (right shoulder, robot frame, mm).
Frame: origin P = shoulder centre of the S1/S2/S3 stage, +x rearward, +y wearer's right, +z up (right-handed).
The clasp does NOT carry the robot arm. Load path = backplate -> hip belt. The clasp only locates the pads on the
shoulder girdle/deltoid, rides on the plate through two bridge struts, and is pulled onto the body by a 32 mm (1-1/4 in) webbing strap.
Each facet is a FLAT plate tangent to the body at its pad site (outward normal = area-weighted mesh normal), standing G off the skin.
Left shoulder = mirror in y (export *_L).
Run:  python3 clasp_shoulder.py [--no_bosses]   (the no-boss variant is used for the beam-to-skin clearance check)"""
import numpy as np, cadquery as cq, os, json, sys
import trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodyprobe as bp

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "stl", "shoulder")
BOSSES = "--no_bosses" not in sys.argv
# ---- rev 2026-10-14 (strap + contoured-facet sprint)
STRAP_W = float(os.environ.get("STRAP_W", 32.0))     # front strap webbing width (was 20): p = T kappa / w -> 32 mm cuts the line pressure by 37 %
SLOT_L, SLOT_T = STRAP_W + 3.0, 3.2                  # slot 35.0 x 3.2 for 32 x 1.2 webbing (1.5 mm clearance each side; 3.2 = two layers of the sewn loop + 0.8)
FLAT_FACETS = os.environ.get("FLAT_FACETS", "0") == "1"      # 1 = old flat load plates (for the before/after runs)
ROLL_H, ROLL_W = float(os.environ.get("ROLL_H", 3.0)), float(os.environ.get("ROLL_W", 8.0))   # rim curls away from the skin by ROLL_H over ROLL_W
SKIRT_W, SKIRT_T = float(os.environ.get("SKIRT_W", 6.0)), 1.5                                 # TPU 85A skirt, bonded to the skin face of the rim
CONTOUR = ("F1_upper_trap", "F3_pec", "F4_acromion", "F2_ant_delt") if os.environ.get("CONTOUR_F3", "1") == "1" else ("F1_upper_trap", "F4_acromion", "F2_ant_delt")     # load pads built as contoured, edge-rolled plates
PLATE_CFG = {   # reshaped load plates: (w along u, h along v, centre offset cu, cv in the pad frame), mm.  Electrode bosses stay at their atlas positions (pad-frame origin).
    "F4_acromion": tuple(float(q) for q in os.environ.get("F4_PLATE", "50x62x0x2").split("x")),    # was 48x48x0x0: grows lateral/medial; AC foam relief; F2 gives way at its F4 corner
    "F2_ant_delt": tuple(float(q) for q in os.environ.get("F2_PLATE", "70x50x0x0").split("x")),    # unchanged outline (the forearm sweeps within ~3 mm of its +u/+v ends); trimmed at its F4 corner
    "F3_pec": tuple(float(q) for q in os.environ.get("F3_PLATE", "80x56x0x0").split("x")),         # was 70x50: absorbs the area F2 gives up (front group F2 + F3 >= 6753 mm2 for the 2-D gate)
}               # F4 load plate reshaped: grows lateral over the acromion body, AC joint relieved
AVOID_MARGIN = 4.5                                           # F2 outline is trimmed where it would come closer than this to the (already built) F1/F3/F4 plates incl. electrode bosses (contoured plates wrap closer than the old flat ones: gap was 3.2 mm, now overlapping)
AC_RELIEF_R = 12.0                                           # F4: the load foam is die-cut in a disc of this radius round the AC joint proxy (plate stays continuous; 5 mm landmark tolerance + 7 mm joint half-width)
STRUCT = "--facets_only" not in sys.argv     # facets_only: export just the pad plates/bosses/cradle; the organic connector is grown by organic_connector.py

# ---- pad stack (all datum = inner face of the facet plate, measured toward the skin)
G = 7.0            # nominal skin-to-datum gap at the pad centre (rev 2026-10-12c: 5 -> 7 so every point under the enlarged flat
                   # load pads keeps >= 2 mm of conformal foam; foam pad 7 mm at the centre, cut to the skin, 2..7 mm)
PLATE_T = 3.5      # facet plate thickness
BOSS_H, BOSS_D = 5.0, 17.0     # electrode boss: height toward skin, diameter (rev: 3 -> 5 with G 7, electrode face still 0.5 mm into the skin)
FACET_FILLET = 12.0            # rev 2026-10-12c: corner radius of the facet plates (removes the corners that touched the skin)
FILLET_SITE = {"F1_upper_trap": 20.0}   # F1 sits on the curved trapezius ridge: larger corners keep >= 2 mm foam under the whole plate
POCKET_D, POCKET_DEPTH = 12.4, 4.0   # pocket for a 12 mm disc electrode (0.2 mm radial clearance), depth from boss tip
# stack in the pocket: 3.5 mm closed-cell foam + 2.0 mm electrode = 5.5 mm -> face stands 1.5 mm proud of the boss tip
WIRE_D = 2.6       # wire hole through plate behind each pocket
PAIR_PITCH = 20.0  # electrode centre spacing for the bipolar pairs (along the muscle fibre direction)
BEAM_W, BEAM_T = 13.0, 5.5          # ring ribbons (top): low profile
ARC_W, ARC_T = 16.0, 6.5            # rear arc ribbons (load path)
BRIDGE_W, BRIDGE_T = 18.0, 6.0      # bridge ribbons to the chassis plate
CL, CH, CT = 48.0, 58.0, 4.0        # axilla cradle: x length, z height, plate thickness (rev 2026-10-13: 56x50 -> 48x58, same area; the arm closes in at the front corner)
PLATE_X = -88.0    # front face of the backplate; bridge feet seat here

body = bp.load()
lm = json.load(open(os.path.join(OUT, "proxies", "wearer_landmarks.json")))
gh = np.array(lm["gh_joint_rel_P"])

# ---- pad sites: ray from outside along 'd', fibre direction f (projected into the facet plane)
SITES = {
    # name: (ray origin, ray dir, fibre dir, electrodes, plate WxH, role)
    # rev 2026-10-12c (I2): load-bearing pads enlarged so the harness meets the 14.7 kPa sustained target (harness_loads.py)
    "F1_upper_trap":  ([-165, -62, 120], [0, 0, -1], [0, 1, 0],   2, (64, 46), "sEMG upper trapezius (shoulder elevation); TOP load pad"),
    "F2_ant_delt":    ([-420, gh[1] + 5, gh[2] + 25], [1, 0, 0], [0, 0, 1], 2, (70, 50), "sEMG anterior deltoid (flexion/protraction); FRONT load pad"),
    "F3_pec":         ([-420, -40, gh[2] + 10], [1, 0, 0], [0, 1, 0.3], 2, (70, 50), "sEMG upper pectoralis / clavicular (protraction); FRONT load pad"),
    "F4_acromion":    ([-190, 16, 120], [0, 0, -1], [1, 0, 0], 1, (48, 48), "REFERENCE electrode on bone (acromion); TOP load pad"),
    "R1_scapula":     ([60, -35, -165], [-1, 0, 0], [0, 0, 1], 0, (34, 24), "standoff node over the moving scapula (v3: lifted off skin)"),
    "R2_post_axilla": ([60, -25, -205], [-1, 0, 0], [0, 0, 1], 0, (34, 24), "standoff node near the posterior axillary fold (v3: lifted off skin)"),
    "F5_mid_trap":   ([60, -30, -125], [-1, 0, 0], [0, 1, 0], 0, (46, 30), "standoff node over the scapular spine (v3: lifted off skin; mid-trap EMG moved to the base layer)"),
}
# rev 2026-10-14b: the muscle atlas is the single source of truth for the sEMG pad sites. F2/F3 seat on the atlas rule points (emg_rules.json,
# written by `muscle_atlas.py --rules`); F1 stays on its load-design position (top pad, 0.7 % pressure margin). USE_ATLAS=0 restores the old sites.
_rp = os.path.join(OUT, "emg_rules.json")
if os.path.exists(_rp) and os.environ.get("USE_ATLAS", "1") == "1":
    _R = json.load(open(_rp))
    for _k, _ch in (("F2_ant_delt", "delt_ant"), ("F3_pec", "pec_clav")):
        _o, _d, _f, _ne, _wh, _role = SITES[_k]; _p = _R[_ch]["p"]; _u = _R[_ch]["u"]
        SITES[_k] = ([-420, _p[1], _p[2]], [1, 0, 0], _u, _ne, _wh, _role + " [atlas-seated]")
sites = {}
for k, (o, d, f, ne, wh, role) in SITES.items():
    hit = bp.cast(body, o, d)
    p = hit[0]; n = bp.smooth_normal(body, p, 16.0)
    f = np.array(f, float); u = f - n * (f @ n); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    sites[k] = dict(p=p, n=n, u=u, v=v, ne=ne, wh=wh, role=role)

def refine(s, iters=5):
    """least-squares plane through ray hits over the electrode discs (so the flat facet follows the local skin, not just one tangent)"""
    for _ in range(iters):
        n, u, v, p = s["n"], s["u"], s["v"], s["p"]
        offs = [0.0] if s["ne"] <= 1 else [-PAIR_PITCH / 2, PAIR_PITCH / 2]
        pts = []
        for c in offs:
            for du, dv in ((0, 0), (7, 0), (-7, 0), (0, 7), (0, -7)):
                q = p + u * (c + du) + v * dv
                r = bp.cast(body, q + n * 60, -n)
                if r is not None: pts.append(r[0])
        pts = np.array(pts); cen = pts.mean(0)
        nn = np.linalg.svd(pts - cen)[2][-1]
        if nn @ n < 0: nn = -nn
        # keep the datum centre on the old normal line through the fitted centroid
        s["p"] = cen; s["n"] = nn / np.linalg.norm(nn)
        u2 = s["u"] - s["n"] * (s["u"] @ s["n"]); s["u"] = u2 / np.linalg.norm(u2); s["v"] = np.cross(s["n"], s["u"])
    # slide the plane along n so the MEAN gap over the electrode centres equals G
    n, u = s["n"], s["u"]; gs = []
    for c in offs:
        q = s["p"] + n * G + u * c
        r = bp.cast(body, q + n * 60, -n)
        if r is not None: gs.append(np.linalg.norm(r[0] - q) * np.sign((r[0] - q) @ -n))
    s["p"] = s["p"] - n * (np.mean(gs) - G)
    return s
for _k in sites: refine(sites[_k]); sites[_k]['p'] = np.asarray(sites[_k]['p'])
# v3 (third-shoulder update): the wearer's scapula slides 5-10 cm under these sites at full arm elevation (McClure 2001),
# so they no longer touch the skin. They stay as structural nodes of the rear arc, lifted along their normal.
SITE_LIFT = {"R1_scapula": 12.0, "R2_post_axilla": 12.0, "F5_mid_trap": 12.0}
for _k, _l in SITE_LIFT.items():
    sites[_k]["p"] = sites[_k]["p"] + sites[_k]["n"] * _l; sites[_k]["lift"] = _l

def plane(origin, xdir, normal):
    return cq.Plane(origin=tuple(map(float, origin)), xDir=tuple(map(float, xdir)), normal=tuple(map(float, normal)))

def facet(s):
    n, u = s["n"], s["u"]
    W, H = s["wh"]
    o_in = s["p"] + n * G                      # datum plane, G off the skin
    pl = cq.Workplane(plane(o_in, u, n)).rect(W, H).extrude(PLATE_T)          # plate occupies G .. G+T (outward)
    pl = pl.edges(cq.selectors.ParallelDirSelector(cq.Vector(*map(float, n)))).fillet(min(s.get("fillet", FACET_FILLET), min(W, H) / 2 - 1.0))
    if BOSSES and s["ne"] > 0:
        offs = [0.0] if s["ne"] == 1 else [-PAIR_PITCH / 2, PAIR_PITCH / 2]
        for c in offs:
            wp = cq.Workplane(plane(o_in + u * c, u, n))
            boss = wp.circle(BOSS_D / 2).extrude(-BOSS_H)                       # toward skin
            pl = pl.union(boss)
            pocket = wp.workplane(offset=-BOSS_H).circle(POCKET_D / 2).extrude(POCKET_DEPTH)  # from boss tip back into plate
            pl = pl.cut(pocket)
            wire = wp.circle(WIRE_D / 2).extrude(PLATE_T + 1.0)
            pl = pl.cut(wire)
    # webbing slot for the strap on the pec facet (F3: 32 mm webbing, 1.2 mm thick -> 35 x 3.2) and the F5 node (22 x 3, no strap routed through it)
    if s.get("strap"):
        if s.get("slot_edge") == "+u":      # F3: the strap arrives from the +u end (cradle is at +112 mm along u, +23 mm along v): long axis along v, 4.5 mm in from the +u edge
            sl = cq.Workplane(plane(o_in + u * (s["wh"][0] / 2 - 4.5), u, n)).rect(s["slot_t"], s["slot_l"]).extrude(PLATE_T + 1)
        else:
            sl = cq.Workplane(plane(o_in + v_off(s) , u, n)).rect(s.get("slot_l", 22.0), s.get("slot_t", 3.0)).extrude(PLATE_T + 1)
        pl = pl.cut(sl)
    return pl

def v_off(s):
    return s["v"] * (s["wh"][1] / 2 - 4.5)

for _k in sites: sites[_k]["fillet"] = FILLET_SITE.get(_k, FACET_FILLET)
if not FLAT_FACETS:
    for _k, _c in PLATE_CFG.items(): sites[_k]["wh"] = (_c[0], _c[1])

sites["F3_pec"]["strap"] = True; sites["F3_pec"]["slot_l"] = SLOT_L; sites["F3_pec"]["slot_t"] = SLOT_T; sites["F3_pec"]["slot_edge"] = "+u"
sites["F5_mid_trap"]["strap"] = True

def node(s, lift):                 # point on the outward normal axis of a facet
    return s["p"] + s["n"] * (G + PLATE_T + lift)

def beam(a, b, w, t, nref):
    a = np.asarray(a, float); b = np.asarray(b, float)
    L = np.linalg.norm(b - a); dvec = (b - a) / L
    x = np.cross(dvec, nref); x /= np.linalg.norm(x)      # beam width direction (tangent to the skin)
    EXT = 3.0   # overlap each end into its neighbour (joints are plain overlaps, no tangent seams)
    return cq.Workplane(plane(a - dvec * EXT, x, dvec)).rect(w, t).extrude(L + 2 * EXT)

_lo = np.min([v["p"] for v in sites.values()], 0) - 80 - np.array([0, 0, 60.0]); _hi = np.max([v["p"] for v in sites.values()], 0) + 80   # rev 2026-10-13: reach down past the cradle
_keep = np.all((body.triangles_center > _lo) & (body.triangles_center < _hi), axis=1)
local = body.submesh([np.where(_keep)[0]], append=True)
NEED = BEAM_T / 2 + 3.0        # centre-line clearance so the beam underside stays >= 3 mm off the skin

def route(a, b, need=None):
    need = NEED if need is None else need
    """polyline a -> ... -> b whose centre line stays >= need from the skin; waypoints pushed along the skin normal"""
    pts = [np.asarray(a, float), np.asarray(b, float)]
    for _ in range(12):
        changed = False; out = [pts[0]]
        for p, q in zip(pts[:-1], pts[1:]):
            ts = np.linspace(0, 1, 30)[1:-1]; Q = p[None] + ts[:, None] * (q - p)[None]
            cp, d, fi = trimesh.proximity.closest_point(local, Q)
            sg = np.einsum("ij,ij->i", Q - cp, local.face_normals[fi]); dist = np.where(sg >= 0, d, -d)
            i = np.argmin(dist)
            if dist[i] < need - 0.2:
                nrm = local.face_normals[fi[i]]; out.append(Q[i] + nrm * (need - dist[i] + 1.5)); changed = True
            out.append(q)
        pts = out
        if not changed: break
    clean = [pts[0]]
    for q in pts[1:]:
        if np.linalg.norm(q - clean[-1]) > 2.0: clean.append(q)
    if np.linalg.norm(clean[-1] - pts[-1]) > 0: clean[-1] = pts[-1]
    return clean

def pillar(s, h=7.5, d=14.0):
    o = s["p"] + s["n"] * (G + PLATE_T - 0.5)
    return cq.Workplane(plane(o, s["u"], s["n"])).circle(d / 2).extrude(h)

LIFT = BEAM_T / 2 - 0.5     # beam centre this far above the facet's outer face -> 0.5 mm overlap with the plate
S = sites
# ---- contoured, edge-rolled load plates (facet.py: contoured_pad).  F1/F2 keep their outline and electrode bosses; F4 is reshaped and gets the AC-joint foam relief.
import facet as fc
CONT_BUILT = {}                         # site -> dict(world mesh, skirt mesh, stats)
PAD_NPZ = {}
if not FLAT_FACETS:
    _lmk = json.load(open(os.path.join(OUT, "emg_atlas.json")))["landmarks"] if os.path.exists(os.path.join(OUT, "emg_atlas.json")) else {}
    for _k in CONTOUR:
        _s = sites[_k]; _p, _n, _u, _v = [np.asarray(_s[q], float) for q in ("p", "n", "u", "v")]
        _W, _H = _s["wh"]; _offs = [] if _s["ne"] == 0 else ([0.0] if _s["ne"] == 1 else [-PAIR_PITCH / 2, PAIR_PITCH / 2])
        _cu, _cv = PLATE_CFG[_k][2:4] if _k in PLATE_CFG else (0.0, 0.0)
        _rel, _hal, _ac = [], [], None
        if _k == "F4_acromion" and "ACR_top" in _lmk:               # AC joint proxy = atlas ACR_top (estimated landmark); foam die-cut disc (plate stays continuous)
            _d = np.asarray(_lmk["ACR_top"], float) - _p; _ac = (float(_d @ _u), float(_d @ _v))
            _rel = [(_ac[0], _ac[1], AC_RELIEF_R)]
        _r = fc.contoured_pad(body, _p, _n, _u, _v, _W, _H, _s["fillet"], G, PLATE_T, cu=_cu, cv=_cv, foam_relief=_rel, roll_H=ROLL_H, roll_W=ROLL_W,
                              bosses=([(c, 0.0) for c in _offs] if BOSSES else []), boss_d=BOSS_D, boss_h=BOSS_H, pocket_d=POCKET_D, pocket_depth=POCKET_DEPTH, wire_d=WIRE_D,
                              skirt_w=SKIRT_W, skirt_t=SKIRT_T, avoid=[v_["mesh"] for v_ in CONT_BUILT.values()] if _k == "F2_ant_delt" else (), avoid_margin=AVOID_MARGIN,
                              cut_boxes=([((_W / 2 - 4.5, 0.0, 12.0), (SLOT_T, SLOT_L, 40.0))] if _s.get("slot_edge") == "+u" else []))
        _T = np.eye(4); _T[:3, 0], _T[:3, 1], _T[:3, 2], _T[:3, 3] = _u, _v, _n, _p
        _mw = _r["mesh"].copy(); _mw.apply_transform(_T)
        if _mw.volume < 0: _mw.invert()
        _sk = None
        if _r["skirt"] is not None: _sk = _r["skirt"].copy(); _sk.apply_transform(_T)
        CONT_BUILT[_k] = dict(mesh=_mw, skirt=_sk, stats=_r["stats"], ac=_ac)
        _g = _r["grid"]
        for _nm in ("a", "b", "t", "inside", "inside0", "rise", "zin", "skirt_mask", "skirt_t"):
            if _nm in _g: PAD_NPZ["%s_%s" % (_k, _nm)] = _g[_nm]
        _s["area_mm2"] = round(_r["stats"]["area_mm2"], 1); _s["contour"] = dict(roll_H=ROLL_H, roll_W=ROLL_W, skirt_w=SKIRT_W, skirt_t=SKIRT_T, centre_uv=[_cu, _cv],
                                                                           foam_relief=_rel, ac_uv=_ac, stats={q: ([round(float(x_), 2) for x_ in w_] if isinstance(w_, list) else round(float(w_), 2)) for q, w_ in _r["stats"].items()})
        print("contoured %s: area %.0f mm2, foam t %.1f..%.1f (mean %.1f), z_in %.1f..%.1f%s" % (_k, _r["stats"]["area_mm2"], _r["stats"]["foam_t_min"], _r["stats"]["foam_t_max"],
              _r["stats"]["foam_t_mean"], *_r["stats"]["z_in_range"], "" if _ac is None else ", AC joint at (%.1f, %.1f) in the pad frame" % _ac))
ring = ["F5_mid_trap", "F1_upper_trap", "F4_acromion", "F2_ant_delt", "F3_pec"]
frame = None
def add(w):
    global frame
    frame = w if frame is None else frame.union(w)
def adds(w):
    if STRUCT: add(w)

for k in ring:
    if k not in CONT_BUILT: add(facet(S[k]))
    adds(pillar(S[k]))
# ring beams (open C: no beam F3->F5, the C opens toward the armpit side)
def run(a, b, w=BEAM_W, th=BEAM_T):
    na = S[a]["n"] + S[b]["n"]; na /= np.linalg.norm(na)
    path = route(node(S[a], LIFT), node(S[b], LIFT), need=th / 2 + 3.0)
    for p, q in zip(path[:-1], path[1:]): adds(beam(p, q, w, th, na))
    return path
paths = {}
for a, b in zip(ring[:-1], ring[1:]): paths[a + ">" + b] = run(a, b)
# stiffening beam F5 -> F4 (closes the rear triangle F5-F1-F4)
paths["F5>F4"] = run("F5_mid_trap", "F4_acromion")

# ---- rear arc: F5 -> R1 (back) -> R2 (posterior axillary fold) -> axilla cradle; with the top ring this closes ~270 deg round the shoulder
for k in ("R1_scapula", "R2_post_axilla"): add(facet(S[k])); adds(pillar(S[k]))
paths["arc F5>R1"] = run("F5_mid_trap", "R1_scapula", ARC_W, ARC_T)
paths["arc R1>R2"] = run("R1_scapula", "R2_post_axilla", ARC_W, ARC_T)
# ---- axilla cradle placement from the mesh (rev 2026-10-13: the corrected body has a real axilla; the old landmark
# gh + (0,-30,-70) sat inside the chest wall). Sections at x = A0 +- CL/2 give the chest wall (torso lateral surface) and the
# arm's medial surface vs z. Apex = highest z where they separate. The cradle top sits APEX_DROP below the apex (keeps the
# rigid plate off the axillary vault / neurovascular bundle), the plate is tilted about x to follow the gap, and its medial face
# stands G (foam) off the chest wall. Its lateral face faces the arm and is foam-covered too (the arm rests on it when adducted).
APEX_DROP = 15.0
def _cross(z, x):
    sec = body.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1]); ys = []
    if sec is None: return ys
    for e in sec.discrete:
        for p_, q_ in zip(e[:-1], e[1:]):
            if (p_[0] - x) * (q_[0] - x) < 0: ys.append(p_[1] + (x - p_[0]) / (q_[0] - p_[0]) * (q_[1] - p_[1]))
    return sorted(y for y in ys if y > gh[1] - 120)
A0x = gh[0]; XS = np.linspace(A0x - CL / 2, A0x + CL / 2, 5)
apex = None
for z in np.arange(gh[2] - 20, gh[2] - 160, -2.5):
    c = _cross(z, A0x)
    if len(c) >= 3 and c[1] - c[0] > 3.0: apex = z; break
ZT = apex - APEX_DROP; ZC = ZT - CH / 2
zs = np.linspace(ZT, ZT - CH, 6); wall, arm = [], []
for z in zs:
    w_, a_ = [], []
    for x in XS:
        c = _cross(z, x)
        if len(c) >= 3: w_.append(c[0]); a_.append(c[1])
        elif c: w_.append(c[0])
    wall.append(max(w_)); arm.append(min(a_) if a_ else np.inf)
wall, arm = np.array(wall), np.array(arm)
mid = 0.5 * (wall + np.where(np.isfinite(arm), arm, wall + 40))
slope = np.polyfit(zs, mid, 1)[0]; TILT = float(np.degrees(np.arctan(slope)))        # +: plate top leans laterally
vw = wall - slope * (zs - ZC); va = arm - slope * (zs - ZC)                          # offsets normal-ish to the tilted plate
A = np.array([A0x, float(vw.max() + G + 3.0 + CT / 2), ZC])
CR = trimesh.transformations.rotation_matrix(np.radians(-TILT), [1, 0, 0], A)       # tilt about x through A
ARM_GAP = float(va.min() - (A[1] + CT / 2))
print("axilla apex z %.1f, cradle centre %s, tilt %.1f deg, lateral face to arm %.1f mm" % (apex, A.round(1).tolist(), TILT, ARM_GAP))
def slab(l, h, t_, r, y0):
    return cq.Workplane("XZ").rect(l, h).extrude(-t_).edges("|Y").fillet(r).translate((A[0], y0, A[2])).rotate(tuple(A), tuple(A + [1, 0, 0]), -TILT)
def ctf(v):            # cradle-local offset -> robot frame
    return (CR @ np.r_[A + np.asarray(v, float), 1.0])[:3]
if BOSSES:
    cr = slab(CL, CH, CT, 5.0, A[1] - CT / 2)
    rim = slab(CL, CH, 3.0, 5.0, A[1] - CT / 2 - 3.0).cut(slab(CL - 6, CH - 6, 3.0, 2.0, A[1] - CT / 2 - 3.0))
    cr = cr.union(rim)
    cr = cr.cut(cq.Workplane("XY").box(SLOT_T, CT + 8, SLOT_L).translate((A[0] - (CL / 2 - 6), A[1] - 1.5, A[2])).rotate(tuple(A), tuple(A + [1, 0, 0]), -TILT))   # FRONT strap slot only (rear end is the arc)
    add(cr)
    r2 = node(S["R2_post_axilla"], ARC_T / 2 - 0.5)
    rear = ctf([CL / 2 - 2.0, 0.0, 0.0])     # rev 2026-10-13: arc lands on the cradle rear edge in its mid-plane (keeps off chest wall and arm)
    # rev 2026-10-13: the corrected body has a real axilla, so the arc no longer needs a skin-touching "padded tail". It runs from
    # R2 (posterior axillary fold, lifted) down behind the arm-torso gap and forward INTO the gap to the cradle's rear edge.
    # Segment 1 (over the moving fold) keeps >= 10 mm under-side clearance; segment 2 (in the gap) keeps >= 3 mm round the
    # ribbon (centre >= ARC_W/2 + 3, since the 16 mm width stands vertical there). Only the last CRADLE_TAIL mm (the joint
    # into the cradle edge) is exempt. Waypoint by deterministic grid search, shortest feasible path.
    CRADLE_TAIL = 12.0
    def _clr(a, b, tail=0.0):
        n = max(10, int(np.linalg.norm(b - a) / 2)); Q = a[None] + np.linspace(0, 1, n)[:, None] * (b - a)[None]
        Q = Q[np.linalg.norm(Q - rear, axis=1) >= tail]
        if len(Q) == 0: return 99.0
        cp, d, fi = trimesh.proximity.closest_point(local, Q); sg = np.einsum("ij,ij->i", Q - cp, local.face_normals[fi]); return float(np.where(sg >= 0, d, -d).min())
    best = None; NEED1, NEED2 = ARC_T / 2 + 10.0, ARC_W / 2 + 3.0
    for wx in np.arange(rear[0] - 10, rear[0] + 81, 5.0):
        for wy in np.arange(rear[1] - 40, rear[1] + 21, 5.0):
            for wz in np.arange(rear[2] - 20, r2[2] + 11, 5.0):
                w_ = np.array([wx, wy, wz]); L_ = np.linalg.norm(w_ - r2) + np.linalg.norm(rear - w_)
                if best is not None and L_ >= best[1]: continue
                c1 = _clr(r2, w_)
                if c1 < NEED1: continue
                c2 = _clr(w_, rear, CRADLE_TAIL)
                if c2 >= NEED2: best = (w_, L_, min(c1, c2))
    pa = [r2, best[0], rear] if best is not None else [r2, rear]
    print("arc R2>cradle waypoint", None if best is None else best[0].round(1).tolist(), "centre-line clearance", None if best is None else round(best[2], 1))
    for p_, q_ in zip(pa[:-1], pa[1:]): adds(beam(p_, q_, ARC_W, ARC_T, np.array([0.0, 1.0, 0.0])))
    paths["arc R2>cradle"] = pa
    json.dump(dict(A=A.round(1).tolist(), CL=CL, CH=CH, CT=CT, TILT=round(TILT, 2), apex_z=round(float(apex), 1), arm_gap=round(ARM_GAP, 1),
                   front_slot=ctf([-(CL / 2 - 6), 0, 0]).round(1).tolist(), strap_w=STRAP_W, slot_l=SLOT_L, slot_t=SLOT_T),
              open(os.path.join(OUT, "clasp_cradle.json"), "w"))

# ---- bridge struts to the chassis plate: 4 supports (was 2). Feet seat on the plate front face x = -88, 2 x M4 slots each.
# (site, foot y, foot z): None -> foot sits straight behind the node. R1 uses a lower, more medial foot to open up the support spread.
FEET_SPEC = [("F5_mid_trap", None, None), ("F1_upper_trap", -45.0, None), ("F4_acromion", None, None), ("R1_scapula", -62.0, -132.0)]
feet = []
for k, fy, fz in FEET_SPEC:
    a = node(S[k], LIFT if not k.startswith("R") else ARC_T / 2 - 0.5)
    fy = a[1] if fy is None else fy; fz = a[2] if fz is None else fz
    foot_c = np.array([PLATE_X - 3.0, fy, fz])
    path = route(a, foot_c, need=BRIDGE_T / 2 + 3.0)
    for p, q in zip(path[:-1], path[1:]): adds(beam(p, q, BRIDGE_W, BRIDGE_T, np.array([0.0, 1.0, 0.0]) if abs(q[2] - p[2]) < abs(q[0] - p[0]) else np.array([1.0, 0.0, 0.0])))
    paths["bridge_" + k] = path
    foot = cq.Workplane("XY").box(6.0, 32.0, 40.0).translate((PLATE_X - 3.0, fy, fz))
    for dz in (-10.0, 10.0):
        slot = cq.Workplane("YZ").slot2D(10.0, 4.5, 90).extrude(8).translate((PLATE_X - 7.0, fy, fz + dz))
        foot = foot.cut(slot)
    adds(foot); feet.append((k, [0.0, float(fy), float(fz)]))

name = ("clasp_facets_R" if not STRUCT else "clasp_frame_R_blocky") + ("" if BOSSES else "_nobosses")
p = os.path.join(OUT, name + ".stl"); cq.exporters.export(frame, p, tolerance=0.03, angularTolerance=0.05)
if CONT_BUILT:                       # merge the contoured plates into the exported frame (manifold union)
    _m0 = trimesh.load(p)
    _mu = trimesh.boolean.union([_m0] + [v_["mesh"] for v_ in CONT_BUILT.values()], engine="manifold")
    if _mu.volume < 0: _mu.invert()
    _mu.export(p); print("frame + contoured plates: watertight", _mu.is_watertight, "vol %.1f cm3" % (_mu.volume / 1e3))
    if BOSSES:
        os.makedirs(os.path.join(OUT, "skirts"), exist_ok=True)
        for _k, v_ in CONT_BUILT.items():
            if v_["skirt"] is not None:
                v_["skirt"].export(os.path.join(OUT, "skirts", "tpu_skirt_%s_R.stl" % _k.split("_")[0]))
                _l = v_["skirt"].copy(); _l.apply_transform(np.diag([1, -1, 1, 1.0]))
                if _l.volume < 0: _l.invert()
                _l.export(os.path.join(OUT, "skirts", "tpu_skirt_%s_L.stl" % _k.split("_")[0]))
        np.savez_compressed(os.path.join(OUT, "pad_foam.npz"), **PAD_NPZ)
if BOSSES and STRUCT:
    m = trimesh.load(p); m.apply_transform(np.diag([1, -1, 1, 1.0]))
    if m.volume < 0: m.invert()
    m.export(os.path.join(OUT, "clasp_frame_L.stl"))
if BOSSES:
    json.dump({k: dict(p=v["p"].round(1).tolist(), n=v["n"].round(3).tolist(), u=v["u"].round(3).tolist(), role=v["role"], electrodes=v["ne"], plate=v["wh"], fillet=v["fillet"],
                       **{q: v[q] for q in ("area_mm2", "contour", "slot_l", "slot_t", "slot_edge") if q in v})
               for k, v in S.items()}, open(os.path.join(OUT, "clasp_sites.json"), "w"), indent=1)
print(name, "ok", "feet at", feet)
if BOSSES: json.dump({k: dict(y=v[1], z=v[2], x_seat=PLATE_X, bolts_dz=[-10.0, 10.0]) for k, v in feet}, open(os.path.join(OUT, "clasp_feet.json"), "w"), indent=1)
if BOSSES: json.dump({k: [np.round(q, 1).tolist() for q in v] for k, v in paths.items()}, open(os.path.join(OUT, "clasp_paths.json"), "w"), indent=1)
print({k: len(v) - 1 for k, v in paths.items()}, "segments per run")
