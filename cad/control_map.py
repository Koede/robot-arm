"""Homologous EMG -> joint control for the Third Shoulder (closes DESIGN_STATE open item 5; extends the residual concept).

Forest's decision (2026-10-12): every third-arm joint is driven by the electrode over the muscle that moves the SAME joint
in his own arm. Flex the biceps -> the robot elbow flexes; anterior deltoid -> robot shoulder swings forward; etc.

Three operating modes (selected by a clasp gesture, see SPEC_emg_joint_map.md):
  MIRROR    robot joint follows the homologous muscle drive; your own arm moving is allowed (both arms move).
  ISOMETRIC robot joint is driven only by effort your own joint does NOT express as motion (residual EMG, own joint
            still per IMU). Use with the hand braced/resting or loaded; lets your own arm keep working (augmentation).
  SCAPULA   the existing scapular-residual mode (shrug/retract/rotate -> S1/S2/S3), arms fully free.
Always on: gravity-explained EMG is subtracted (holding your arm out does not move the robot), co-contraction sets
joint stiffness (impedance like a muscle), bell-shaped velocity shaping, tip-speed cap, slow soft-limit approach, and a
300 ms signal dead-man.

Shoulder mapping is done in TASK space: each muscle pair commands an angular velocity about a fixed body axis
(flexion/extension, abduction/adduction, axial rotation) and the S1-S2-S3 rates come from a damped-least-squares inverse
of the chain Jacobian (biomimetic_shoulder.py). That keeps "anterior deltoid = forward" true in every pose, which joint-to-
joint mapping cannot do because the robot's S1/S2 axes are not the human's anatomical axes.

rev 2026-10-13f (unify): the arm is 575 mm to the claw tip (578 mm from P with the 57 mm forearm offset), so the per-joint speed caps are
recomputed from the real levers (S1/S2 49.5 deg/s, S3/E 81.5 deg/s; the old 57.3 deg/s gave 0.57 m/s at the tip) and a combined tip-speed
bound (sum lever_j |qdot_j| <= 0.5 m/s, triangle inequality, conservative) scales all joints together. Mirrors firmware control.h.

rev 2026-10-14 (precise muscle map, cad/muscle_atlas.py): every shoulder-girdle site is now a landmark-referenced bipolar pair (stl/shoulder/emg_atlas.json).
The single pectoralis channel split in two (pec_clav on clasp F3 = clavicular head, pec_stern = sternal head, base layer) and teres major was added:
23 of 24 ADS inputs used. HOMOLOGY: clavicular head -> flexion; sternal head -> adduction + internal rotation; teres major -> internal rotation.

rev 2026-10-14 (Track D port, closes DESIGN_STATE open item 10; reconciled with rev d):
  [D1] The conditioning test (scenario F, "swing lateral from the arm-up rest pose") now runs with the CURRENT speed limits: per-joint caps VMAX
       (S1/S2 49.5, S3 81.5 deg/s, scaled together) AND the combined tip-speed limiter. The old flat 90 deg/s cap is kept as sim("legacy90")
       / --legacy-cond so the old numbers (park 1.18 deg/s) still reproduce. Because the limiter now dominates the magnitude, the pass/fail criterion
       of the test is the DIRECTION error (angle between delivered and wanted angular velocity, first 300 ms) plus the delivered speed fraction.
  [D2] Optional decoupling hook Controller(decoupler=callable): applied to the EMG envelope AFTER the per-sample validity check (saturation / flat-line
       detection stays on the raw channels) and BEFORE gravity subtraction / gating. Default None = unchanged behaviour (used by emg_calib.py).
  [D3] Claw G sign fixed (firmware-integration finding): HOMOLOGY["G"] = (["edc"], ["fds"]) so FDS closes (G toward 0 %) and EDC opens (G toward 100 %);
       g_sign_check() runs both directions from park (G 50) and is part of the JSON summary. Firmware headers are NOT changed here (control.h must match).
  The tip-speed limiter is still the TRIANGLE bound. An exact-Jacobian tip speed was analysed (cad/cond_exact_vs_triangle.py, SPEC_control_rerun.md s3)
  and is a PROPOSAL only; it is NOT applied here or in control.h.

Run: python3 cad/control_map.py [--legacy-cond]  -> renders/control_sim.png, stl/control/control_sim[_legacycond].json
Needs stl/shoulder/keepout_map.npz (from cad/keepout_map.py; only head_scale() uses it, scenarios A-E, the claw check and the conditioning test never call it with motion near
the head). If the mesh pipeline is not available, the map can be rebuilt (1 mm quantised) from firmware/third_shoulder/keepout_table.h, which gen_tables.py writes from it.
"""
import numpy as np, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
import biomimetic_shoulder as bs

# ------------------------------------------------------------------------------------------------ channel map (per side)
# site: landmark rule of the placement (full numbers: stl/shoulder/emg_atlas.json, SPEC_emg_joint_map.md s2, sources and confidence: SPEC_emg_sources.md);
# carrier: where the electrode lives (base layer = compression garment, textile/dry; clasp = rigid pad on a stable facet).
# ia = ADS1299 input index + 1 (daisy order: dev1 1-8, dev2 9-16, dev3 17-24).
# SENIAM pages were unreachable: shoulder sites are textbook / secondary-table rules (sources and flags in muscle_atlas.py); arm/forearm sites are unchanged.
CHANNELS = {
 # shoulder (humerothoracic) pairs
 "delt_ant":  dict(site="anterior deltoid, 1 finger-width distal/anterior to acromion; pair on clasp F2 (seated on the atlas rule point, 1.3 mm off, rev c)", carrier="clasp F2", ia=1),
 "delt_mid":  dict(site="middle deltoid, 26 % of acromion -> lateral epicondyle (~91 mm), pair 81-101 mm, distal of mid-belly IZ, proximal of tendon", carrier="base-layer sleeve", ia=2),
 "delt_post": dict(site="posterior deltoid, 2 finger-widths (35 mm) behind the acromial angle", carrier="base-layer sleeve", ia=3),
 "pec_clav":  dict(site="pectoralis major CLAVICULAR head, 40 % along notch -> clavicle, ~25 mm below it (clasp F3 seated on the rule point, 1.0 mm off, rev c)", carrier="clasp F3", ia=4),
 "lat_dorsi": dict(site="latissimus dorsi, 40 mm below the inferior angle, halfway between spine and lateral torso edge, fibres ~45 deg up-lateral", carrier="base layer", ia=5),
 "infraspin": dict(site="infraspinatus, 40 mm below the scapular spine, 62 % from the spine root to the acromial angle, parallel to the spine", carrier="base layer", ia=6),
 # elbow / forearm / wrist / hand (unchanged)
 "biceps":    dict(site="biceps brachii, line medial acromion -> cubital fossa, distal 1/3", carrier="base-layer sleeve", ia=7),
 "triceps":   dict(site="triceps long/lateral head, 50% acromion -> olecranon, 2 cm lateral", carrier="base-layer sleeve", ia=8),
 "pron_teres":dict(site="pronator teres, 3-4 cm distal to medial epicondyle, anterior", carrier="base-layer sleeve", ia=9),
 "supinator": dict(site="supinator/brachioradialis proxy, 3 cm distal lateral epicondyle (crosstalk risk)", carrier="base-layer sleeve", ia=10),
 "fcr":       dict(site="flexor carpi radialis, 1/3 medial epicondyle -> radial styloid", carrier="base-layer sleeve", ia=11),
 "fcu":       dict(site="flexor carpi ulnaris, ulnar border proximal 1/3", carrier="base-layer sleeve", ia=12),
 "ecr":       dict(site="extensor carpi radialis longus/brevis, 5 cm distal lateral epicondyle", carrier="base-layer sleeve", ia=13),
 "ecu":       dict(site="extensor carpi ulnaris, dorsal ulnar border proximal 1/3", carrier="base-layer sleeve", ia=14),
 "fds":       dict(site="flexor digitorum superficialis, mid volar forearm", carrier="base-layer sleeve", ia=15),
 "edc":       dict(site="extensor digitorum, mid dorsal forearm", carrier="base-layer sleeve", ia=16),
 # scapular (existing SCAPULA mode, DESIGN_STATE s6)
 "trap_up":   dict(site="upper trapezius, 56 % of C7 -> lateral acromion (rules: 50 % [B18] .. +20 mm [B20]), perpendicular offset 3.4 mm; pair on clasp F1", carrier="clasp F1", ia=17),
 "trap_mid":  dict(site="middle trapezius, midway between T3 spinous line and the medial border, +10 mm cranial, fibres horizontal (under the rail foam; medial disc straddles the rev d rail edge)", carrier="base layer", ia=18),
 "trap_low":  dict(site="lower trapezius, 1/3 of the way T7 spinous -> scapular-spine root, oblique up-lateral (under the rail foam)", carrier="base layer", ia=19),
 "rhomboid":  dict(site="rhomboid major, midway spinous line - medial border, 1/3 down the border from the root, fibres down-lateral (under the rail foam)", carrier="base layer", ia=20),
 "serratus":  dict(site="serratus anterior, 7th intercostal space at inferior-angle height, mid-axillary line (flank panel)", carrier="base layer", ia=21),
 # rev 2026-10-14 additions
 "pec_stern": dict(site="pectoralis major STERNAL head, 35 mm medial to the anterior axillary line, GH height - 25 mm (chest panel)", carrier="base layer", ia=22),
 "teres_maj": dict(site="teres major, lateral scapular border, 28 mm lateral and 22 mm cranial of the inferior angle, fibres up-lateral", carrier="base layer", ia=23),
}
# 23 used + 1 spare = 24 = three ADS1299 per side, daisy-chained on the side's existing SPI bus (pin map unchanged).

# Robot DOF <- (agonist set, antagonist set). Shoulder DOFs are task-space angular velocity components.
HOMOLOGY = {
 "sh_flex": (["delt_ant", "pec_clav"], ["delt_post", "lat_dorsi"]),   # forward / back swing (clavicular head flexes)
 "sh_abd":  (["delt_mid"], ["lat_dorsi", "pec_stern"]),                # lateral / medial swing (sternal head adducts)
 "sh_rot":  (["infraspin"], ["pec_stern", "teres_maj"]),              # external / internal rotation (subscapularis is deep: sternal pec + teres major)
 "E":  (["biceps"], ["triceps"]),
 "W1": (["supinator"], ["pron_teres"]),
 "W2": (["fcr", "fcu"], ["ecr", "ecu"]),                             # flexion / extension
 "W3": (["fcr", "ecr"], ["fcu", "ecu"]),                             # radial / ulnar deviation
 "G":  (["edc"], ["fds"]),                                           # rev 2026-10-14 sign fix: EDC (finger extensors) OPENS (G -> 100 %), FDS (finger flexors) CLOSES (G -> 0 %).
                                                                     # The pre-fix order (["fds"], ["edc"]) made a positive net drive = FDS > EDC raise G, i.e. squeezing
                                                                     # your fingers OPENED the claw (claw: 0 % closed .. 100 % open, rev 13f). Checked by g_sign_check().
}
# Body-frame axes for the shoulder task components, signed so that from the up-rest pose the agonist moves the robot arm
# the way the wearer means it: anterior deltoid -> forward over the shoulder (-x), middle deltoid -> lateral (+y).
SH_AXES = {"sh_flex": np.array([0., -1., 0.]), "sh_abd": np.array([-1., 0., 0.])}   # sh_rot uses the current arm axis

# Robot soft limits (deg) and speed caps from the 0.5 m/s tip cap: cap = v_tip / lever arm to the tip.
import rom_limits as rl
SOFT = dict(rl.SOFT)       # rev 2026-10-12c: single source of limits (S1 10..140, S2 -30..88, E 0..145)
# rev 2026-10-13f: levers to the CLAW tip (z 575): S1/S2 578 mm (575 along the arm + 57 mm forearm offset), S3 and E 350 mm (forearm perpendicular
# to the upper arm, 345 mm + offset), W1/W2 165 mm, W3 120 mm; G is the claw (% per s; fingertip ~0.13 m/s at 90 %/s).
LEVER_M = {"S1": 0.578, "S2": 0.578, "S3": 0.35, "E": 0.35, "W1": 0.165, "W2": 0.165, "W3": 0.12, "G": 0.0}
V_TIP = 0.5
VMAX = {k: (90.0 if L == 0.0 else min(np.floor(np.degrees(V_TIP / L) * 2) / 2, 90.0)) for k, L in LEVER_M.items()}   # deg/s, floored to 0.5, also capped at 90
def tip_bound(v_deg_s, names=("S1", "S2", "S3", "E", "W1", "W2", "W3")):
    """Triangle-inequality bound on the tip speed (m/s): sum lever_j |qdot_j|. Conservative; mirrors control.h."""
    return float(sum(LEVER_M[n] * abs(np.radians(v)) for n, v in zip(names, v_deg_s)))
SLOW_ZONE = 15.0         # deg: speed ramps to zero over the last 15 deg before a soft limit
DEADBAND = 0.05          # fraction of MVC
GAIN = 1.0 / 0.30        # 30 % MVC residual effort -> full joint speed cap
K0, KC = 2.0, 18.0       # joint stiffness (N m/rad) at rest and per unit co-contraction (normalised); AK70 MIT-mode kp
DEADMAN_S = 0.300
ONSET_CONFIRM = 0.150    # s: ISOMETRIC residual must persist with the own joint still before the robot moves
GATE_HOLD = 0.300        # s: after own-joint motion stops, keep that DOF gated this long

HEAD_HARD, HEAD_SLOW = 60.0, 160.0                              # mm: stop / start slowing
# rev 2026-10-12c: mesh-based keep-out. keepout_map.npz = straight-arm clearance to the whole wearer mesh (head, neck, shoulder) on a
# 2.5 deg (S1, S2) grid; bilinear interpolation. rev 2026-10-13f: the capsule is r 32 from P to the claw tip + 15 mm (z 575 + 15).
_KO = np.load(os.path.join(ROOT, "stl", "shoulder", "keepout_map.npz"))
from scipy.interpolate import RegularGridInterpolator
_KOI = RegularGridInterpolator((_KO["S1"], _KO["S2"]), _KO["C"], bounds_error=False, fill_value=-50.0)

def head_clearance(q, R0=None, L=590.0):
    """Worst-case (straight arm) clearance to the wearer, mm. Elbow/wrist folding only shortens reach toward the body."""
    return float(_KOI([[q[0], q[1]]])[0])

def head_scale(q, qd_sh, R0, dt=0.01):
    """Speed scale for the shoulder: free when moving away from the head, ramps 1 -> 0 between +160 and +60 mm otherwise."""
    c0 = head_clearance(q, R0); c1 = head_clearance(np.asarray(q[:3]) + qd_sh * dt, R0)
    if c1 >= c0: return 1.0
    return float(np.clip((c0 - HEAD_HARD) / (HEAD_SLOW - HEAD_HARD), 0, 1))

class Shaper:
    """Critically damped 2nd-order filter on the velocity command: bell-shaped speed profiles (minimum-jerk-like),
    no step changes in motor torque. Natural frequency 2.5 Hz."""
    def __init__(self, n, wn=2 * np.pi * 2.5): self.x = np.zeros(n); self.v = np.zeros(n); self.wn = wn
    def step(self, u, dt):
        a = self.wn ** 2 * (u - self.x) - 2 * self.wn * self.v
        self.v += a * dt; self.x += self.v * dt; return self.x.copy()

# ---- rev f (2026-10-17): S2 gravity-balancer keep-out, mirrors firmware control.h bal_emax_r / bal_guard. The claw at deep elbow fold with a bent wrist can
# reach the balancer (tube, swivel, cord): rule E <= E_bal(S3, W2) from stl/balancer/balancer_elbow_keepout.json, any S1 / S2 / W1 / W3, applied in EVERY tier.
# Same sampling-margin rule as firmware/gen_tables.py: a cell whose min clearance at E_max is < 3.5 mm loses one 2.5 deg step. The guard only blocks / slows rates.
BAL_MARGIN_MM, BAL_STEP_DEG, BAL_E_FREE = 3.5, 2.5, 145.0
_BK = json.load(open(os.path.join(ROOT, "stl", "balancer", "balancer_elbow_keepout.json")))["table"]
BAL_S3 = sorted({c["S3"] for c in _BK.values()}); BAL_W2E = sorted({c["W2_bin"][0] for c in _BK.values()} | {c["W2_bin"][1] for c in _BK.values()})
BAL_E = np.full((len(BAL_S3), len(BAL_W2E) - 1), BAL_E_FREE)
for c in _BK.values():
    i, k = BAL_S3.index(c["S3"]), BAL_W2E.index(c["W2_bin"][0])
    BAL_E[i, k] = c["E_max_deg"] - (BAL_STEP_DEG if c["min_mm_at_E_max"] < BAL_MARGIN_MM else 0.0)
BAL_E_MIN = float(BAL_E.min())

def bal_emax_r(s3a, s3b, w2a, w2b):
    """Conservative range look-up (firmware bal_emax_r): MIN over the S3 rows from the last row strictly below min(s3) to the first strictly above max(s3),
    x every W2 bin the closed range touches. Never interpolated upward; outside the table the edge row / bin applies; NaN -> worst cell."""
    if any(x != x for x in (s3a, s3b, w2a, w2b)): return BAL_E_MIN
    a0, a1 = min(s3a, s3b), max(s3a, s3b)
    b0 = float(np.clip(min(w2a, w2b), BAL_W2E[0], BAL_W2E[-1])); b1 = float(np.clip(max(w2a, w2b), BAL_W2E[0], BAL_W2E[-1]))
    i0 = 0
    while i0 + 1 < len(BAL_S3) and BAL_S3[i0 + 1] < a0: i0 += 1
    i1 = len(BAL_S3) - 1
    while i1 - 1 >= 0 and BAL_S3[i1 - 1] > a1: i1 -= 1
    i1 = max(i1, i0); m = BAL_E_FREE
    for i in range(i0, i1 + 1):
        for k in range(len(BAL_W2E) - 1):
            if BAL_W2E[k + 1] >= b0 and BAL_W2E[k] <= b1: m = min(m, BAL_E[i, k])
    return float(m)

def bal_emax(s3, w2): return bal_emax_r(s3, s3, w2, w2)

def bal_guard(q, v, dt, look=0.05, slow=15.0):
    """q, v: dicts of joint angle / commanded rate (deg, deg/s). Only zeroes or scales rates of S3, E+ and W2 (never moves a reference).
    Legal pose: E may rise only to E_bal at the current (S3, W2) (the elbow limit with slow approach is applied by the caller); S3 / W2 rates that would move
    into a cell whose E_bal is below the elbow angle are zeroed, and slowed over `slow` deg before such a cell. Illegal pose: E may only fold down."""
    h = max(look, dt); e = q["E"]; now = bal_emax(q["S3"], q["W2"])
    if e > now:
        if v["E"] > 0: v["E"] = 0.0
        if bal_emax_r(q["S3"], q["S3"] + v["S3"] * h, q["W2"], q["W2"]) < now: v["S3"] = 0.0
        if bal_emax_r(q["S3"], q["S3"], q["W2"], q["W2"] + v["W2"] * h) < now: v["W2"] = 0.0
        return v
    e_top = max(e, min(e + v["E"] * h, now))
    for j in ("S3", "W2"):
        if v[j] == 0: continue
        lim = lambda a, b: bal_emax_r(q["S3"], q["S3"] + a, q["W2"], q["W2"] + b)
        if lim(v["S3"] * h if j == "S3" else 0, v["W2"] * h if j == "W2" else 0) < e_top: v[j] = 0.0; continue
        sg = 1.0 if v[j] > 0 else -1.0; d = 0.5
        while d <= slow + 1e-3:
            if lim(sg * d if j == "S3" else 0, sg * d if j == "W2" else 0) < e_top:
                v[j] *= float(np.clip((d - 0.5) / slow, 0.0, 1.0)); break
            d += 0.5
    return v

def pair_drive(res, dof):
    ag, an = HOMOLOGY[dof]
    a = np.mean([res[c] for c in ag]); b = np.mean([res[c] for c in an])
    net = a - b; cc = min(a, b)                      # net drive, co-contraction
    u = np.sign(net) * max(0.0, abs(net) - DEADBAND) * GAIN
    return float(np.clip(u, -1, 1)), float(cc)

def limit_scale(q, qd, lo, hi):
    """Slow approach: scale speed toward a limit to zero across SLOW_ZONE; motion away from the limit is free."""
    if qd > 0: return float(np.clip((hi - q) / SLOW_ZONE, 0, 1))
    if qd < 0: return float(np.clip((q - lo) / SLOW_ZONE, 0, 1))
    return 1.0

class Controller:
    def __init__(self, mode="ISOMETRIC", mount=(0.0, 0.0), decoupler=None):
        self.mode = mode; self.R0 = bs.mount(*mount)
        self.q = dict(rl.PARK); self.q["E"] = 30.0          # start from the park pose (off the S2 = 0 singularity)
        self.shape = Shaper(8); self.bad_t = 0.0; self.stopped = False; self.stop_time = None
        self.onset = {}; self.last_move = {}; self.decoupler = decoupler
    def residual(self, t, dt, emg, expected, own_speed, own_gate_dofs):
        """EMG minus what gravity (always) and own motion (ISOMETRIC only) explain. own_speed: deg/s per human DOF.
        ISOMETRIC: a DOF's channels pass only if the own joint has been still (< 5 deg/s) for GATE_HOLD and the residual has
        persisted ONSET_CONFIRM; own movement starts ~50 ms after EMG onset (electromechanical delay), so a natural movement
        is caught before the robot responds."""
        res = {k: max(0.0, emg[k] - expected.get(k, 0.0)) for k in emg}
        if self.mode == "ISOMETRIC":
            for dof, chans in own_gate_dofs.items():
                if abs(own_speed.get(dof, 0.0)) > 5.0: self.last_move[dof] = t
                still = (t - self.last_move.get(dof, -1e9)) > GATE_HOLD
                active = max(res[c] for c in chans) > DEADBAND
                self.onset[dof] = (self.onset.get(dof, 0.0) + dt) if (active and still) else 0.0
                if not (still and self.onset[dof] >= ONSET_CONFIRM):
                    for c in chans: res[c] = 0.0
        return res
    def step(self, t, dt, emg, expected, own_speed, sig_ok):
        # ---- per-sample channel validity: a saturated (lift-off, rail) or flat channel is dropped IMMEDIATELY so it can never
        #      drive motion during the detection window; the dead-man then stops everything within 300 ms
        bad = [k for k, e in emg.items() if e >= 0.98 or e <= 0.0]
        if bad:
            emg = dict(emg); sig_ok = False
            for k in bad: emg[k] = 0.0
        if self.decoupler is not None and not bad: emg = self.decoupler(emg)     # [D2] crosstalk decoupling, after the validity check
        # ---- dead-man: any channel saturated / flat, or IMU stale -> stop within 300 ms (100 ms detect + 200 ms ramp)
        self.bad_t = self.bad_t + dt if not sig_ok else 0.0
        if self.bad_t >= 0.100 and not self.stopped: self.stopped, self.stop_time = True, t
        gate_map = {"sh_flex": HOMOLOGY["sh_flex"][0] + HOMOLOGY["sh_flex"][1], "sh_abd": HOMOLOGY["sh_abd"][0] + HOMOLOGY["sh_abd"][1],
                    "sh_rot": HOMOLOGY["sh_rot"][0] + HOMOLOGY["sh_rot"][1], "E": ["biceps", "triceps"]}
        res = self.residual(t, dt, emg, expected, own_speed, gate_map)
        u = {d: pair_drive(res, d) for d in HOMOLOGY}
        # ---- shoulder: task-space angular velocity -> joint rates (DLS)
        arm = bs.fk((self.q["S1"], self.q["S2"], self.q["S3"]), self.R0) @ bs.Z
        w = (u["sh_flex"][0] * SH_AXES["sh_flex"] + u["sh_abd"][0] * SH_AXES["sh_abd"] + u["sh_rot"][0] * arm) * np.radians(VMAX["S2"])
        J = bs.jacobian((self.q["S1"], self.q["S2"], self.q["S3"]), self.R0)
        qd_sh = np.degrees(bs.dls_inverse(J) @ w)
        cmd = np.array([qd_sh[0], qd_sh[1], qd_sh[2], u["E"][0] * VMAX["E"], u["W1"][0] * VMAX["W1"], u["W2"][0] * VMAX["W2"], u["W3"][0] * VMAX["W3"], u["G"][0] * VMAX["G"]])
        names = ["S1", "S2", "S3", "E", "W1", "W2", "W3", "G"]
        # per-joint speed cap (shoulder scaled together to keep direction)
        s = max(1.0, max(abs(cmd[i]) / VMAX[n] for i, n in enumerate(names[:3]))); cmd[:3] /= s
        for i, n in enumerate(names[3:], 3): cmd[i] = np.clip(cmd[i], -VMAX[n], VMAX[n])
        if self.stopped: cmd[:] = 0.0
        v = self.shape.step(cmd, dt)
        if self.stopped and self.stop_time is not None and t - self.stop_time > 0.2:   # 200 ms ramp done -> hold (brake / kp hold)
            v[:] = 0.0; self.shape.v[:] = 0; self.shape.x[:] = 0
        qsh = np.array([self.q["S1"], self.q["S2"], self.q["S3"]]); v[:3] *= head_scale(qsh, v[:3], self.R0)
        vb = bal_guard(self.q, {n: float(v[i]) for i, n in enumerate(names)}, dt)            # balancer keep-out E <= E_bal(S3, W2), every tier
        for i, n in enumerate(names): v[i] = vb[n]
        b = tip_bound(v[:7])                                  # combined tip-speed cap (all arm joints scaled together), TRIANGLE bound
        if b > V_TIP: v[:7] *= V_TIP / b
        for i, n in enumerate(names):
            hi_n = max(self.q[n], min(SOFT[n][1], bal_emax(self.q["S3"], self.q["W2"]))) if n == "E" else SOFT[n][1]   # E slows into E_bal like a soft limit
            v[i] *= limit_scale(self.q[n], v[i], SOFT[n][0], hi_n)
            self.q[n] = float(np.clip(self.q[n] + v[i] * dt, SOFT[n][0], hi_n))        # hi_n = SOFT hi, or max(q, min(SOFT hi, E_bal)) for E
        stiff = {d: K0 + KC * u[d][1] for d in HOMOLOGY}
        return v, stiff, qd_sh

# ------------------------------------------------------------------------------------------------ simulation
def sim(cond_cap="current"):
    """cond_cap: "current" = conditioning test with the real per-joint caps + combined limiter [D1]; "legacy90" = the pre-rev-13f flat 90 deg/s cap."""
    dt = 0.002; T = np.arange(0, 12, dt); rng = np.random.default_rng(1)
    # own elbow model (Forest): I qdd = k(a_bi - a_tri) - tau_g(q) - b qd ; arm can be braced on a table (qd forced 0)
    def run(mode, script):
        ctl = Controller(mode); q_h = 30.0; qd_h = 0.0; log = []
        for t in T:
            sc = script(t)
            a_bi, a_tri, braced = sc["bi"], sc["tri"], sc.get("braced", False)
            tau_g = 0.10 * np.cos(np.radians(q_h))              # gravity-hold demand, fraction of MVC (forearm horizontal ~10 %)
            net = a_bi - a_tri - tau_g
            if braced: qd_h = 0.0
            else:
                qdd = 900.0 * net - 12.0 * qd_h; qd_h += qdd * dt; q_h = float(np.clip(q_h + qd_h * dt, 0, 140))
                if q_h in (0, 140): qd_h = 0
            emg = {k: 0.01 + 0.005 * abs(rng.standard_normal()) for k in CHANNELS}
            emg["biceps"] = a_bi + tau_g * 0 + 0.01 * abs(rng.standard_normal()) + (tau_g if not braced else 0.0) * 0
            emg["triceps"] = a_tri + 0.01 * abs(rng.standard_normal())
            # gravity-explained expectation from the IMU pose (calibrated linear model; here exact + 10 % error)
            expected = {"biceps": 1.1 * tau_g if not braced else 0.0}
            ok = sc.get("sig_ok", True)
            if not ok: emg["biceps"] = 1.0          # electrode lift-off -> rail saturation
            v, stiff, _ = ctl.step(t, dt, emg, expected, {"E": qd_h}, ok)
            log.append((t, q_h, qd_h, ctl.q["E"], v[3], stiff["E"], ctl.stopped))
        return np.array(log, dtype=float)
    hold = lambda t: 0.10 * np.cos(np.radians(30))            # baseline biceps to hold own forearm
    scenarios = {
     "A_mirror_flex": ("MIRROR", lambda t: dict(bi=hold(t) + (0.25 if 1 < t < 3 else 0.0), tri=0.0 + (0.20 if 3.5 < t < 5.5 else 0.0))),
     "B_iso_natural_move": ("ISOMETRIC", lambda t: dict(bi=hold(t) + (0.25 if 1 < t < 3 else 0.0), tri=(0.20 if 3.5 < t < 5.5 else 0.0))),
     "C_iso_braced_push": ("ISOMETRIC", lambda t: dict(bi=(0.30 if 1 < t < 7 else 0.02), tri=0.02, braced=True)),
     "D_cocontract": ("ISOMETRIC", lambda t: dict(bi=(0.35 if 1 < t < 4 else 0.02), tri=(0.35 if 1 < t < 4 else 0.02), braced=True)),
     "E_deadman": ("ISOMETRIC", lambda t: dict(bi=0.12, tri=0.02, braced=True, sig_ok=not (2.5 < t < 3.1))),
    }
    out = {k: run(*v) for k, v in scenarios.items()}
    # ---- shoulder: homologous abduction intent from the up-rest pose [D1: joint rates limited like the real controller]
    sh = {}
    cap = np.array([VMAX["S1"], VMAX["S2"], VMAX["S3"]]); lev = np.array([LEVER_M["S1"], LEVER_M["S2"], LEVER_M["S3"]])
    for label, mnt, off in (("old_rest_S2_0", (0, 0), 0.0), ("near_rest_S2_3", (0, 0), 3.0), ("park_S2_25", (0, 0), 25.0)):
        R0 = bs.mount(*mnt); q = np.array([rl.PARK["S1"], off, 0.0]); rows = []
        for t in np.arange(0, 1.5, dt):
            w_des = np.radians(40) * SH_AXES["sh_abd"]            # ask 40 deg/s lateral swing
            J = bs.jacobian(q, R0); qd = np.degrees(bs.dls_inverse(J) @ w_des)
            b_ = 0.0
            if cond_cap == "legacy90":
                s = max(1.0, np.abs(qd).max() / 90.0); qd /= s     # pre-13f actuator speed cap 90 deg/s (conditioning test, not the tip cap)
            else:
                s = max(1.0, float(np.max(np.abs(qd) / cap))); qd /= s                       # per-joint caps (shoulder scaled together, as in Controller.step)
                b_ = float(np.sum(lev * np.abs(np.radians(qd))))                              # combined tip-speed limiter (triangle bound)
                if b_ > V_TIP: qd *= V_TIP / b_
            w_act = J @ np.radians(qd); err = np.degrees(np.linalg.norm(w_act - w_des))
            nw, nd = np.linalg.norm(w_act), np.linalg.norm(w_des)
            dang = float(np.degrees(np.arccos(np.clip(w_act @ w_des / max(nw * nd, 1e-12), -1, 1)))) if nw > 1e-9 else 90.0
            q = q + qd * dt; rows.append((t, np.abs(qd).max(), err, dang, nw / nd, b_))
        sh[label] = np.array(rows)
    return T, out, sh

def g_sign_check(dt=0.002):
    """[D3] claw direction: MIRROR mode from park (G 50); FDS 30 % MVC for 1 s, rest, then EDC 30 % MVC for 1 s. FDS must close (G falls), EDC must open (G rises)."""
    ctl = Controller("MIRROR"); g0 = ctl.q["G"]; rng = np.random.default_rng(2); log = []
    for t in np.arange(0, 3.0, dt):
        emg = {k: 0.01 + 0.005 * abs(rng.standard_normal()) for k in CHANNELS}
        if 0.0 <= t < 1.0: emg["fds"] = 0.30
        if 1.5 <= t < 2.5: emg["edc"] = 0.30
        ctl.step(t, dt, emg, {}, {}, True); log.append((t, ctl.q["G"]))
    L = np.array(log); g_fds = float(L[L[:, 0] < 1.5, 1][-1]); g_edc = float(L[:, 1][-1])
    return dict(G_start_pct=g0, G_after_FDS_pct=round(g_fds, 1), G_after_EDC_pct=round(g_edc, 1),
                FDS_closes=bool(g_fds < g0 - 1.0), EDC_opens=bool(g_edc > g_fds + 1.0))

def balancer_check(dt=0.002):
    """[rev f] balancer keep-out E <= E_bal(S3, W2): (a) driven fold in MIRROR at S3 -70 / W2 -50 stops at E_bal with a slow approach, no overshoot;
    (b) guard unit cases: slow zone before a stricter S3 row, block at the boundary, illegal pose can only fold down."""
    ctl = Controller("MIRROR"); ctl.q["S3"], ctl.q["W2"], ctl.q["E"] = -70.0, -50.0, 30.0
    ebal = bal_emax(-70.0, -50.0); rng = np.random.default_rng(3); emax = 0.0; e_log = []
    for t in np.arange(0, 12.0, dt):
        emg = {k: 0.01 + 0.005 * abs(rng.standard_normal()) for k in CHANNELS}; emg["biceps"] = 0.5
        v, _, _ = ctl.step(t, dt, emg, {}, {}, True); emax = max(emax, ctl.q["E"]); e_log.append(ctl.q["E"])
    approach = float(np.mean(np.abs(np.diff(e_log[-int(2.0 / dt):])) / dt))     # deg/s over the last 2 s (should be ~0: stopped at the limit)
    base = dict(S1=0, S2=0, S3=0, E=0, W1=0, W2=0, W3=0, G=0)
    q1 = dict(base, E=130.0, S3=-45.0, W2=-50.0); v1 = dict(base, S3=-30.0); r1 = bal_guard(q1, dict(v1), dt)["S3"]
    q2 = dict(base, E=130.0, S3=-49.9, W2=-50.0); r2 = bal_guard(q2, dict(v1), dt)["S3"]
    q3 = dict(base, E=140.0, S3=-70.0, W2=-50.0); r3 = bal_guard(q3, dict(base, E=20.0, S3=-30.0, W2=-30.0), dt); r3b = bal_guard(q3, dict(base, E=-20.0), dt)
    return dict(E_bal_at_S3m70_W2m50_deg=ebal, E_max_reached_deg=round(emax, 2), E_speed_last2s_deg_s=round(approach, 3),
                stops_at_E_bal=bool(emax <= ebal + 1e-6 and emax >= ebal - 2.0), table_min_E_deg=BAL_E_MIN,
                slow_zone_scale=round(r1 / -30.0, 3), slow_zone_ok=bool(0.0 < r1 / -30.0 < 1.0), blocked_at_boundary=bool(r2 == 0.0),
                illegal_pose_E_up_blocked=bool(r3["E"] == 0.0), illegal_pose_S3_W2_not_stricter=bool(r3["S3"] == 0.0 or bal_emax_r(-70, -70 + r3["S3"] * 0.05, -50, -50) >= bal_emax(-70, -50)),
                illegal_pose_E_fold_down_passes=bool(r3b["E"] == -20.0))

def summarise(T, out, sh, legacy=False):
    summ = {}
    kc = 2 if legacy else 3      # tracking criterion: legacy = |w_act - w_des| < 2 deg/s (old test); current = direction angle < 2 deg (magnitude is set by the limiter)
    for k, L in out.items():
        t, qh, qdh, qr, vr, kst, stp = L.T
        summ[k] = dict(own_elbow_travel_deg=round(float(qh.max() - qh.min()), 1), robot_elbow_travel_deg=round(float(qr.max() - qr.min()), 1),
                       robot_peak_speed_deg_s=round(float(np.abs(vr).max()), 1), peak_stiffness_Nm_rad=round(float(kst.max()), 1))
        if k == "E_deadman":
            i0 = np.searchsorted(t, 2.5); i1 = np.searchsorted(t, 3.1)
            moving = np.where(np.abs(vr[i0:i1]) > 0.5)[0]
            summ[k]["speed_at_fault_deg_s"] = round(float(abs(vr[i0])), 1)
            summ[k]["time_to_stop_ms"] = round(float(t[i0 + moving[-1]] - 2.5) * 1000, 0) if len(moving) else 0.0
            summ[k]["restarts_after_signal_returns"] = bool(np.abs(vr[i1:]).max() > 0.5)
    for k, L in sh.items():
        summ["shoulder_abd_from_rest_" + k] = dict(peak_joint_speed_deg_s=round(float(L[:, 1].max()), 1),
            mean_dir_error_deg_s_first_300ms=round(float(L[L[:, 0] < 0.3, 2].mean()), 2),
            mean_dir_angle_deg_first_300ms=round(float(L[L[:, 0] < 0.3, 3].mean()), 2), mean_speed_fraction_first_300ms=round(float(L[L[:, 0] < 0.3, 4].mean()), 3),
            tip_bound_unscaled_peak_m_s=round(float(L[:, 5].max()), 3),
            time_to_track_within_2deg_s=(round(float(L[np.argmax(L[:, kc] < 2.0), 0]), 3) if (L[:, kc] < 2.0).any() else "never (stuck in singularity)"),
            track_criterion=("|w_act - w_des| < 2 deg/s" if legacy else "direction angle < 2 deg"))
    summ["G_claw_sign_check"] = g_sign_check()
    summ["balancer_keepout_check"] = balancer_check()
    summ["speed_caps_deg_s"] = {k: round(v, 1) for k, v in VMAX.items()}
    summ["tip_speed_levers_m"] = LEVER_M
    return summ

if __name__ == "__main__":
    legacy = "--legacy-cond" in sys.argv
    T, out, sh = sim("legacy90" if legacy else "current")
    summ = summarise(T, out, sh, legacy)
    os.makedirs(os.path.join(ROOT, "stl", "control"), exist_ok=True)
    json.dump(summ, open(os.path.join(ROOT, "stl", "control", "control_sim%s.json" % ("_legacycond" if legacy else "")), "w"), indent=1)
    print(json.dumps(summ, indent=1))

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 3, figsize=(15, 7.5)); axs = axs.ravel()
    titles = {"A_mirror_flex": "A  MIRROR: you flex/extend your elbow\n→ robot elbow follows (both move)",
              "B_iso_natural_move": "B  ISOMETRIC: same natural movement\n→ robot gated (your arm is busy)",
              "C_iso_braced_push": "C  ISOMETRIC: hand braced, push biceps 30 % MVC\n→ robot elbow flexes, slows into the soft limit",
              "D_cocontract": "D  co-contraction biceps + triceps\n→ no motion, stiffness rises",
              "E_deadman": "E  electrode lift-off at t = 2.5 s\n→ stop, hold; no auto-restart"}
    for ax, (k, L) in zip(axs, out.items()):
        t, qh, qdh, qr, vr, kst, stp = L.T
        ax.plot(t, qh, color="#64748b", lw=2, label="your elbow (IMU)"); ax.plot(t, qr, color="#2563eb", lw=2, label="robot elbow E")
        ax2 = ax.twinx(); ax2.plot(t, vr, color="#f59e0b", lw=1, alpha=.8); ax2.set_ylabel("robot E speed (deg/s)", color="#f59e0b", fontsize=8)
        if k == "D_cocontract": ax2.plot(t, kst, color="#16a34a", lw=1.5); ax2.set_ylabel("speed / stiffness N·m/rad", fontsize=8)
        if k == "E_deadman": ax.axvspan(2.5, 3.1, color="#dc2626", alpha=.15)
        ax.set_title(titles[k], fontsize=9); ax.set_xlabel("t (s)"); ax.set_ylabel("angle (deg)"); ax.grid(alpha=.3)
        if k == "A_mirror_flex": ax.legend(fontsize=8, loc="upper right")
    ax = axs[5]
    for k, L in sh.items(): ax.plot(L[:, 0], L[:, 3], lw=2, label=k.replace("_", " "))
    ax.set_title("F  'swing lateral' intent: shoulder direction error\n(DLS inverse, %s)" % ("legacy flat 90 deg/s cap" if legacy else "current caps + tip limiter"), fontsize=9)
    ax.set_xlabel("t (s)"); ax.set_ylabel("∠(ω delivered, ω wanted) (deg)"); ax.legend(fontsize=8); ax.grid(alpha=.3)
    os.makedirs(os.path.join(ROOT, "renders"), exist_ok=True)
    plt.tight_layout(); plt.savefig(os.path.join(ROOT, "renders", "control_sim%s.png" % ("_legacycond" if legacy else "")), dpi=115)
