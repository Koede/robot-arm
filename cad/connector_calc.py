"""Hand-calc + scikit-fem study for the three connector concepts (A dual-pin dovetail splice, B bayonet-dovetail quick-connect, C cross-spring hinge).
mm, N, MPa.  Writes stl/connectors/connector_calc.json.  Run: python3 connector_calc.py  (seconds, 1 core).

EVERY MATERIAL ALLOWABLE BELOW IS AN "UNVERIFIED PLACEHOLDER": there is no PA12-CF / PA-CF / TPU 95A datasheet in the repo and none was retrieved.
They are round numbers of the right order for FDM PA-CF, TPU 95A and hardened dowels, used to rank the concepts; replace them with the filament
vendor's datasheet or with coupon tests (see the report, section Tests) before any number here is used for a go/no-go on a printed part.

Load case (A):  limit moment M = 24.8 N m (plate_pocket.py: AK70 peak = S1/S2 torque, same value the chassis plate is checked for).  Lever = distance
from the shoulder centre P (robot-frame origin, the S1/S2/S3 intersection) to the splice frame 'at' in stl/shoulder/pauldron/pauldron.json.  The arm
splice then sees a transverse force V = M / lever (conservative: the whole peak torque is assumed reacted through ONE splice; in the real design the
3 plate struts take the load path and the wrap-arms only locate the clasp, so this is an upper bound).  Bound case 2 adds the bending moment
V * (splice-to-clasp-pad distance) that V would produce at the splice if it acted at the pad."""
import os, json, math
import numpy as np
import scipy.spatial
from skfem import MeshTri, Basis, ElementTriP2, ElementVector, asm, solve, condense, LinearForm
from skfem.models.elasticity import linear_elasticity, lame_parameters

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, "..", "stl")
OUT = os.path.join(STL, "connectors")
os.makedirs(OUT, exist_ok=True)

# ------------------------------------------------------------------ UNVERIFIED PLACEHOLDERS -------------------------------------
PH = dict(
    note="UNVERIFIED PLACEHOLDER: all values in this dict. No datasheet in the repo; round numbers for FDM PA-CF / TPU 95A / hardened steel dowel.",
    PACF_UTS_XY=60.0,          # MPa tensile, in-plane (along the roads)
    PACF_UTS_Z=25.0,           # MPa tensile across layers
    PACF_BEARING_XY=45.0,      # MPa allowable bearing on a printed hole wall (in-plane)
    PACF_SHEAR_XY=30.0,        # MPa
    PACF_E_FLEX=4000.0,        # MPa flexural modulus, in-plane roads (thin leaf)
    PACF_NU=0.35,
    PACF_SN_SLOPE=0.09,        # S-N rule of thumb: sigma_max,allow(N) = UTS * (1 - slope * log10 N); ~0.46*UTS at 1e6 (NOT a vendor value)
    STEEL_DOWEL_TAU=300.0,     # MPa allowable shear for a hardened d2.5 dowel (conservative round number)
    BOLT_TAU=270.0,            # MPa allowable shear, M4 A2-70 assumed (0.6 * Rp0.2 450 per ISO 3506 class 70 as I recall it; the BOM may specify 8.8)
    TPU_E=26.0, TPU_NU=0.45, TPU_MU_ON_PACF=0.5, TPU_STRAIN_FATIGUE=0.05,
)

# ------------------------------------------------------------------ geometry from the repo ---------------------------------------
P = json.load(open(os.path.join(STL, "shoulder", "pauldron", "pauldron.json")))
spl = P["splices"]; pads = P["clasp_pads"]
M_LIMIT = 24.8e3     # N mm, plate_pocket.py


def splice_levers():
    out = {}
    pad_of = {"arm_med": "F1", "arm_lat": "F4"}
    for k, s in spl.items():
        at = np.array(s["at"]); pad = np.array(pads[pad_of[k]])
        out[k] = dict(at=s["at"], lever_P_to_splice=round(float(np.linalg.norm(at)), 1), pad=pad_of[k], lever_splice_to_pad=round(float(np.linalg.norm(pad - at)), 1))
    return out


# existing joint: 2 x M4 x 16 half-lap, 9 mm bridge (4.5 + 4.5), bolts at +-6 mm along the splice axis, hole d4.5 (pauldron.py: radius 2.25)
M4 = dict(shank_area=math.pi * 4 ** 2 / 4, thread_stress_area=8.78, d=4.0, t_half=4.5, pitch=12.0, hole_d=4.5, strut_w=18.0, strut_t=9.0)
# A: dowels d2.5 along w, pitch 9, cheek widths from connector_concepts.py geometry
A_ = dict(d=2.5, pitch=9.0, x=(3.5, 12.5), root_w=8.0, ang=10.0, strut_w=18.0, strut_t=9.0)


def a_tongue_w(x): return A_["root_w"] + 2 * x * math.tan(math.radians(A_["ang"]))


def joint_loads():
    lev = splice_levers()
    res = {}
    for k, l in lev.items():
        V = M_LIMIT / l["lever_P_to_splice"]
        Ms = V * l["lever_splice_to_pad"]
        res[k] = dict(l, V_N=round(V, 1), M_splice_Nmm_if_V_acts_at_pad=round(Ms, 0))
    return res


def hand_calc_A(V, Ms):
    r = {}
    # ---- existing 2 x M4 half-lap: single shear at the lap interface, bolts share V
    per = V / 2
    r["M4_halflap"] = dict(
        bolt_shear_MPa=round(per / M4["thread_stress_area"], 1),
        bolt_shear_SF=round(PH["BOLT_TAU"] / (per / M4["thread_stress_area"]), 1),
        bearing_on_PACF_MPa=round(per / (M4["d"] * M4["t_half"]), 1),
        bearing_SF=round(PH["PACF_BEARING_XY"] / (per / (M4["d"] * M4["t_half"])), 1),
        one_bolt_lost_bearing_MPa=round(V / (M4["d"] * M4["t_half"]), 1),
        one_bolt_lost_bearing_SF=round(PH["PACF_BEARING_XY"] / (V / (M4["d"] * M4["t_half"])), 1),
        bound_case_couple_per_bolt_N=round(Ms / M4["pitch"], 0),
        bound_case_bearing_MPa=round(Ms / M4["pitch"] / (M4["d"] * M4["t_half"]), 1),
        bound_case_bolt_shear_MPa=round(Ms / M4["pitch"] / M4["thread_stress_area"], 1),
        bound_case_bearing_SF=round(PH["PACF_BEARING_XY"] / (Ms / M4["pitch"] / (M4["d"] * M4["t_half"])), 2),
        note="shank-in-shear would give 1.4x lower stress; thread stress area used. Preload friction not counted. Section modulus at the seam: two 4.5 mm halves = %.1f mm3 (non-composite) .. %.1f (composite)" % (2 * 18 * 4.5 ** 2 / 6, 18 * 9 ** 2 / 6))
    # ---- A: two d2.5 steel dowels in double shear along w
    a_pin = math.pi * A_["d"] ** 2 / 4
    sh = lambda F: F / (2 * a_pin)
    bear_tongue = lambda F, x: F / (A_["d"] * a_tongue_w(x))
    cheek = lambda x: (A_["strut_w"] - a_tongue_w(x)) / 2
    bear_cheek = lambda F, x: F / (2 * A_["d"] * cheek(x))
    xs = A_["x"]
    r["A_dual_pin"] = dict(
        pin_shear_MPa=round(sh(V / 2), 1), pin_shear_SF=round(PH["STEEL_DOWEL_TAU"] / sh(V / 2), 1),
        bearing_in_tongue_MPa=[round(bear_tongue(V / 2, x), 1) for x in xs],
        bearing_in_cheeks_MPa=[round(bear_cheek(V / 2, x), 1) for x in xs],
        cheek_thickness_mm=[round(cheek(x), 2) for x in xs],
        bearing_SF_worst=round(PH["PACF_BEARING_XY"] / max([bear_tongue(V / 2, x) for x in xs] + [bear_cheek(V / 2, x) for x in xs]), 1),
        one_pin_loose_pin_shear_MPa=round(sh(V), 1), one_pin_loose_bearing_worst_MPa=round(max(bear_tongue(V, x) for x in xs), 1),
        one_pin_loose_bearing_SF=round(PH["PACF_BEARING_XY"] / max(bear_tongue(V, x) for x in xs), 1),
        one_pin_loose_joint_still_held_in_t_and_w="yes: dovetail flanks lock t and w, the remaining pin still blocks z' and the w-slide",
        bound_case_couple_per_pin_N=round(Ms / A_["pitch"], 0),
        bound_case_tongue_bearing_MPa=round(max(bear_tongue(Ms / A_["pitch"], x) for x in xs), 1),
        bound_case_bearing_SF=round(PH["PACF_BEARING_XY"] / max(bear_tongue(Ms / A_["pitch"], x) for x in xs), 2),
        bound_case_pin_shear_MPa=round(sh(Ms / A_["pitch"]), 1),
        seam_section_modulus_tongue_neck_mm3=round(A_["root_w"] * A_["strut_t"] ** 2 / 6, 1),
        note="pins sit 9 mm apart (M4 bolts 12 mm): the moment couple on the pins is 1.33x the bolt couple; the tongue neck (8 x 9) is 44 % of the strut section modulus (243 mm3)")
    r["strut_bending_reference"] = dict(
        strut_section_modulus_mm3=18 * 81 / 6, sigma_bound_case_MPa=round(Ms / (18 * 81 / 6), 1),
        sigma_over_PACF_UTS_XY=round(Ms / (18 * 81 / 6) / PH["PACF_UTS_XY"], 2),
        note="if the wrap-arm really carried V at the pad, the unspliced 18 x 9 strut itself would be over the placeholder strength: the bound case is not a design load, it shows that the splice is not the weak link in that case")
    return r


# ------------------------------------------------------------------ scikit-fem: bearing / tear-out of a pin hole in a plate ------
def bearing_fe(width, edge, thick, hole_d, force, length=36.0, nring=48, hmax=0.7):
    """Plane-stress plate [0,length] x [0,width], hole of diameter hole_d at (edge, width/2); x = length end clamped; the force acts on the hole's
    x<0-facing half (cosine-distributed nodal forces) toward the free edge x = 0 (tear-out direction, worst in-plane case).  Returns the nominal
    bearing stress F/(d t) and the peak von Mises sampled at element centroids between 1.2 r and 2 r (clear of the point-load singularity)."""
    r = hole_d / 2
    pts = []
    for k in range(nring):
        a = 2 * math.pi * k / nring
        for rr in (r, 1.25 * r, 1.7 * r, 2.4 * r):
            pts.append((edge + rr * math.cos(a), width / 2 + rr * math.sin(a)))
    xs = np.unique(np.concatenate([np.arange(0, length + 1e-9, hmax * 2), [length]])); ys = np.unique(np.concatenate([np.arange(0, width + 1e-9, hmax * 2), [width]]))
    for x in xs:
        for y in ys:
            if math.hypot(x - edge, y - width / 2) > 2.4 * r + 0.5 * hmax: pts.append((x, y))
    pts = np.array(pts); tri = scipy.spatial.Delaunay(pts)
    cen = pts[tri.simplices].mean(1)
    keep = np.hypot(cen[:, 0] - edge, cen[:, 1] - width / 2) > r * 0.999
    m = MeshTri(pts.T.copy(), tri.simplices[keep].T.copy())
    el = ElementVector(ElementTriP2()); basis = Basis(m, el)
    lam, mu = lame_parameters(PH["PACF_E_FLEX"], PH["PACF_NU"])
    K = asm(linear_elasticity(lam, mu), basis)
    # nodal forces on hole boundary dofs of the P2 basis: take the vertex nodes on the hole circle that face -x
    rad = np.hypot(basis.doflocs[0] - edge, basis.doflocs[1] - width / 2)
    on = np.where(np.abs(rad - r) < 1e-6 * r + 1e-9)[0]
    ang = np.arctan2(basis.doflocs[1][on] - width / 2, basis.doflocs[0][on] - edge)
    w = np.cos(ang - math.pi)                       # facing -x
    sel = w > 0
    wts = w[sel] / w[sel].sum()
    f = np.zeros(basis.N)
    xdofs = on[sel]
    # for ElementVector, dof index layout: component 0 dofs are basis.nodal_dofs[0] (+ facet dofs); doflocs are shared per vector comp
    f[xdofs] = -force * wts                          # x-comp force toward -x (towards the free edge)
    fixed = basis.get_dofs(lambda x: x[0] > length - 1e-6).all()
    u = solve(*condense(K, f, D=fixed))
    # stress at centroids
    from skfem import Functional
    grad = basis.interpolate(u).grad                  # (2,2,nel,nqp)
    e = 0.5 * (grad + grad.transpose(1, 0, 2, 3))
    tr = e[0, 0] + e[1, 1]
    sxx = 2 * mu * e[0, 0] + lam * tr; syy = 2 * mu * e[1, 1] + lam * tr; sxy = 2 * mu * e[0, 1]
    vm = np.sqrt(sxx ** 2 - sxx * syy + syy ** 2 + 3 * sxy ** 2).mean(-1)          # plane-stress-ish (lam from 3D Lame -> plane strain; see note)
    c = m.p[:, m.t].mean(1)
    rc = np.hypot(c[0] - edge, c[1] - width / 2)
    band = (rc > 1.2 * r) & (rc < 2.0 * r)
    nominal = force / (hole_d * thick)
    # FE is 2D unit-thickness: scale stress by 1/thick
    peak = float(vm[band].max()) / thick
    return dict(width=width, edge=edge, thick=thick, hole_d=hole_d, force_N=force, nominal_bearing_MPa=round(nominal, 2),
               fe_peak_vm_in_band_MPa=round(peak, 2), ratio_peak_to_nominal=round(peak / nominal, 2), n_elem=int(m.t.shape[1]))


def fe_bearing_cases(V):
    out = {}
    # A cheek (arm member): 2.2..3.2 mm thick cheeks seen edge-on as a plate in the t-z' plane: width 9 (z'), pin at 3.5 from the seam face, d2.4 press hole
    out["A_cheek_pin1_x3.5"] = bearing_fe(width=9.0, edge=3.5, thick=2.9, hole_d=2.4, force=V / 2 / 2)       # half of a pin's load per cheek
    out["A_cheek_pin2_x12.5"] = bearing_fe(width=9.0, edge=12.5, thick=2.9, hole_d=2.4, force=V / 2 / 2)
    # existing: half-lap plate in the t-w plane: 18 wide, 4.5 thick, bolt hole d4.5 at 6 mm from the lap end (pitch 12 -> bolt at 6 from seam, lap +-12)
    out["M4_halflap_hole_edge6"] = bearing_fe(width=18.0, edge=6.0, thick=4.5, hole_d=4.5, force=V / 2)
    out["note"] = ("In-plane bearing / tear-out slice with the force toward the nearest free edge (worst in-plane direction). The real pin loads act mostly out of this plane "
                   "(z'); the slice is a check on edge distance and hole stress concentration, not a full 3D model. FE stress uses plane strain Lame constants "
                   "(skfem linear_elasticity default) - conservative by ~10 %. Peak is sampled 1.2-2 r from the hole (point-load singularity excluded), so it is a "
                   "lower bound of the hole-edge value.")
    return out


# ------------------------------------------------------------------ B: bayonet + TPU detent ---------------------------------------
def calc_B():
    E, mu_f = PH["TPU_E"], PH["TPU_MU_ON_PACF"]
    d, ltot, prot = 3.0, 6.5, 0.6                      # TPU pin d3, free length 6.5 (8 - 1.5 dome seat), nose protrusion 0.6
    A = math.pi * d ** 2 / 4
    k = E * A / ltot                                    # unconfined axial stiffness (conservative low: the pin is also supported by its hole walls)
    N = k * prot
    out = dict(tpu_pin_d=d, free_len=ltot, protrusion=prot, stiffness_N_per_mm=round(k, 1), normal_force_N=round(N, 1), compression_strain=round(prot / ltot, 3))
    def ramp(alpha_deg):
        a = math.radians(alpha_deg)
        return N * (math.sin(a) + mu_f * math.cos(a)) / (math.cos(a) - mu_f * math.sin(a))
    r_lug = 12.0; n_det = 2
    for name, al in (("lock_in_30deg_ramp", 30.0), ("unlock_45deg_ramp", 45.0)):
        F = ramp(al)
        out[name] = dict(tangential_force_per_detent_N=round(F, 1), torque_Nm=round(n_det * F * r_lug / 1e3, 2), alpha_deg=al)
    out["holding_torque_to_unlock_Nm"] = out["unlock_45deg_ramp"]["torque_Nm"]
    out["tangential_hold_at_lug_radius_N"] = round(n_det * ramp(45.0), 0)
    # axial pull-out (the bayonet itself): lug as a cantilever 3 mm radial x 3 mm axial x ~6.6 mm arc, load across layers (print axis = stem axis)
    arc = math.radians(36.0) * 10.5
    Zs = arc * 3.0 ** 2 / 6
    sig_per_N = (0.5 * 1.5) / Zs                        # F/2 per lug at 1.5 mm arm
    sz = PH["PACF_UTS_Z"] * 0.5                         # fatigue/knock-down on the weak direction: placeholder x0.5
    out["axial_pullout"] = dict(lug_arc_mm=round(arc, 1), root_bending_MPa_per_N=round(sig_per_N, 4), Z_allow_MPa=sz, F_pullout_limit_N=round(sz / sig_per_N, 0),
                                F_pullout_static_N=round(PH["PACF_UTS_Z"] / sig_per_N, 0),
                                shear_across_layers_alt=("lug rooted horizontally: load goes through the layer adhesion; if printed with the stem axis in the bed plane "
                                                         "(on its side) the lug bending is in-plane and the limit rises to ~%d N (XY strength)" % round(PH["PACF_UTS_XY"] * 0.5 / sig_per_N)))
    out["play"] = dict(axial_play_mm=2 * 0.25, note="channel is lug + 2 clr: 0.5 mm axial rattle at 0.25; a 0.5 mm TPU wave washer / O-ring under the base is needed to preload the lugs against the channel roof (not modelled)")
    out["chamfer_note"] = "the ramp angles assume 30/45 deg chamfers on the lug edges and a dome nose; the coupon CAD has square lug edges (add in detail design)"
    return out


# ------------------------------------------------------------------ C: cross-spring flexure --------------------------------------
def leaf_stress(E, nu, t, L, theta_deg):
    th = math.radians(theta_deg)
    Ep = E / (1 - nu ** 2)                              # wide leaf: plane strain across the width
    eps = t * th / (2 * L)
    return eps, Ep * eps


def fe_leaf(E, nu, t, L, theta_deg=3.0):
    """Linear FE check of the S-bend assumption: strip L x t, left end clamped, right end rotated by theta (small), free to translate transversally.
    Returns peak fibre stress FE vs the closed form E' t theta / (2 L)."""
    nx, ny = 60, 6
    m = MeshTri.init_tensor(np.linspace(0, L, nx + 1), np.linspace(0, t, ny + 1))
    basis = Basis(m, ElementVector(ElementTriP2()))
    Ep = E
    lam, mu = lame_parameters(E, nu)
    K = asm(linear_elasticity(lam, mu), basis)
    th = math.radians(theta_deg)
    x = basis.doflocs
    u = np.zeros(basis.N)
    left = basis.get_dofs(lambda p: p[0] < 1e-9).all()
    right_x = basis.get_dofs(lambda p: p[0] > L - 1e-9, skip=["u^2"]).all()          # only the x component prescribed on the right face (rotation)
    u[right_x] = -th * (x[1][right_x] - t / 2)
    D = np.concatenate([left, right_x])
    sol = solve(*condense(K, np.zeros(basis.N), x=u, D=D))
    g = basis.interpolate(sol).grad
    e = 0.5 * (g + g.transpose(1, 0, 2, 3)); tr = e[0, 0] + e[1, 1]
    sxx = (2 * mu * e[0, 0] + lam * tr).mean(-1)
    cen_x = m.p[0, m.t].mean(0)
    mid = (cen_x > 0.3 * L) & (cen_x < 0.7 * L)                  # away from the clamp stress concentration
    peak = float(np.abs(sxx[mid]).max())
    Ep_ = E / (1 - nu ** 2)
    closed = Ep_ * t * th / (2 * L)
    return dict(theta_deg=theta_deg, fe_sigma_x_MPa=round(peak, 3), closed_form_MPa=round(closed, 3), ratio=round(peak / closed, 3))


def calc_C():
    L = math.hypot(2 * 4.0, 12.0)
    th30 = 30.0
    out = dict(leaf_len=round(L, 2), theta_deg=th30, note="stress = E' t theta / (2 L), uniform-curvature S-bend of a clamped-clamped leaf whose end block rotates by theta (E' = E/(1-nu^2)). Axial stretch of the leaves (second order) and the crossing offset are ignored.")
    for mat, E, nu, t in (("PACF", PH["PACF_E_FLEX"], PH["PACF_NU"], 0.8), ("PACF_t0.5", PH["PACF_E_FLEX"], PH["PACF_NU"], 0.5), ("TPU", PH["TPU_E"], PH["TPU_NU"], 1.2)):
        eps, sig = leaf_stress(E, nu, t, L, th30)
        d = dict(t=t, strain_pct=round(eps * 100, 3), sigma_MPa=round(sig, 2))
        if mat.startswith("PACF"):
            ratio = sig / PH["PACF_UTS_XY"]
            d["sigma_over_UTS_XY"] = round(ratio, 2)
            if ratio >= 1: d["cycles"] = "static failure at first stroke (stress >= placeholder UTS)"
            else: d["cycles_log10_UNVERIFIED_PLACEHOLDER"] = round((1 - ratio) / PH["PACF_SN_SLOPE"], 1)
            # max angle for 1e5 cycles under the same S-N rule
            sig_allow = PH["PACF_UTS_XY"] * (1 - PH["PACF_SN_SLOPE"] * 5)
            Ep = E / (1 - nu ** 2)
            d["max_theta_deg_for_1e5_cycles"] = round(math.degrees(sig_allow * 2 * L / (Ep * t)), 1)
            d["leaf_len_needed_for_30deg_1e5_mm"] = round(Ep * t * math.radians(th30) / (2 * sig_allow), 1)
        else:
            d["cycles"] = "UNVERIFIED PLACEHOLDER: strain %.1f %% vs %.0f %% working strain assumed for >1e6 cycles in 95A TPU" % (eps * 100, PH["TPU_STRAIN_FATIGUE"] * 100)
            # buckling of the leaf as a column (clamped-clamped, K = 0.5), per leaf, b = 10
            I = 10.0 * t ** 3 / 12; Pcr = math.pi ** 2 * E * I / (0.5 * L) ** 2
            d["leaf_buckling_load_N_each"] = round(Pcr, 1)
            d["hinge_stiffness_Nmm_per_rad"] = round(2 * E / (1 - nu ** 2) * I / L, 1)
        out[mat] = d
    # PA-CF buckling for comparison
    t = 0.8; I = 10.0 * t ** 3 / 12; out["PACF"]["leaf_buckling_load_N_each"] = round(math.pi ** 2 * PH["PACF_E_FLEX"] * I / (0.5 * L) ** 2, 1)
    out["PACF"]["hinge_stiffness_Nmm_per_rad"] = round(2 * PH["PACF_E_FLEX"] / (1 - PH["PACF_NU"] ** 2) * I / L, 1)
    out["pinned_reference"] = "d3 steel pin in d3.1 running hole (claw_hand.py): no fatigue limit in this range, 0.1 mm radial play, survives the full stroke"
    out["fe_check_linear_small_angle"] = {mat: fe_leaf(E / 1.0, nu, t, L, 3.0) for mat, E, nu, t in (("PACF", PH["PACF_E_FLEX"], PH["PACF_NU"], 0.8),)}
    return out


def run():
    lev = joint_loads()
    worst = max(lev.values(), key=lambda d: d["V_N"])
    res = dict(units="mm, N, MPa", placeholders=PH, M_limit_Nmm=M_LIMIT, M_source="cad/plate_pocket.py: 24.8 N m (AK70 peak, S1/S2) limit moment about each axis",
               lever_source="stl/shoulder/pauldron/pauldron.json 'splices' (frame 'at'); lever = |at| from P = origin; P is the S1/S2/S3 intersection", splice_loads=lev,
               governing_splice=[k for k, v in lev.items() if v is worst][0])
    res["A_vs_M4"] = hand_calc_A(worst["V_N"], worst["M_splice_Nmm_if_V_acts_at_pad"])
    res["A_vs_M4_fe_bearing"] = fe_bearing_cases(worst["V_N"])
    res["B"] = calc_B()
    res["C"] = calc_C()
    with open(os.path.join(OUT, "connector_calc.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps({k: res[k] for k in ("splice_loads", "governing_splice", "A_vs_M4", "A_vs_M4_fe_bearing", "B", "C")}, indent=1))


if __name__ == "__main__":
    run()
