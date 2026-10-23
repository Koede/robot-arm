"""Biomimetic shoulder study (session 2026-10-12b).

Why: the robot shoulder is S1 about z, S2 about y, S3 about the arm axis, and the arm points UP at S2 = 0.
That rest pose is exactly the gimbal singularity of the chain (S1 axis parallel to S3 axis). A human shoulder
has no mechanical singularity: whatever direction the brain asks for, the joint can rotate that way at once.
So a homologous controller ("flex the anterior deltoid -> arm goes forward") would need unbounded S1 speed
near rest. This script:
  1. builds the angular-velocity Jacobian of the S1-S2-S3 chain for an arbitrary fixed mount rotation R0,
  2. scores the inverse condition number 1/kappa over the arm directions the current soft limits allow,
  3. grid-searches a mount tilt (beta about y, gamma about x) that moves the singular directions to places the
     arm never goes (into the wearer's back, straight up-and-back), with the S1 motor body kept clear of the back,
  4. sizes the compliant "capsule" stop pads (progressive TPU pads in front of the existing rigid stops).
Frame (DESIGN_STATE §2): origin P, +x away from the back, +y wearer's right, +z up. Body at -x. Wearer faces -x.
REV 2026-10-12c: the tilt recommendation (beta 60, gamma -20) is WITHDRAWN. The S2 joint cannot swing the arm more than ~92 deg
from the S1 axis (yoke floor / S1 motor), so any tilt that moves the singular direction off 'up' makes part of the workspace
unreachable (60/-20: 21 %, forward-over-shoulder needs 147 deg). Each candidate now reports reach_fail. The adopted fix is a
park pose at S2 = 25 deg (rom_limits.PARK) plus the damped-least-squares inverse for passing through vertical.
Run: python3 cad/biomimetic_shoulder.py   -> stl/control/biomimetic_shoulder.json, renders/shoulder_singularity.png
"""
import numpy as np, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")

def Rx(a):
    c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rz(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
Z = np.array([0., 0., 1.]); Y = np.array([0., 1., 0.])

def mount(beta_deg=0.0, gamma_deg=0.0):
    """Fixed mount rotation about P. beta tilts the S1 axis toward +x (away from the back); gamma toward -y (medial)."""
    return Rx(np.radians(gamma_deg)) @ Ry(np.radians(beta_deg))

def fk(q_deg, R0=np.eye(3)):
    """Orientation of the arm frame. q = (S1, S2, S3) deg, project sign conventions."""
    t1, t2, t3 = np.radians(q_deg)
    return R0 @ Rz(-t1) @ Ry(-t2) @ Rz(t3)

def jacobian(q_deg, R0=np.eye(3)):
    """Spatial angular-velocity Jacobian: omega = J @ qdot (rad/s)."""
    t1, t2, _ = np.radians(q_deg)
    c1 = -(R0 @ Z)
    c2 = -(R0 @ Rz(-t1) @ Y)
    c3 = R0 @ Rz(-t1) @ Ry(-t2) @ Z
    return np.column_stack([c1, c2, c3])

def inv_cond_dir(d, a):
    """1/kappa of the chain when the arm points along unit d and the S1 axis is unit a.
    Columns: a, (a x d)/|a x d| (S2 axis is perpendicular to both), d."""
    cr = np.cross(a, d); n = np.linalg.norm(cr)
    if n < 1e-9: return 0.0
    s = np.linalg.svd(np.column_stack([a, cr / n, d]), compute_uv=False)
    return s[-1] / s[0]

def inv_cond_dirs(D, a):
    """Vectorised inv_cond_dir over an (N,3) array of arm directions."""
    cr = np.cross(np.broadcast_to(a, D.shape), D); n = np.linalg.norm(cr, axis=1)
    ok = n > 1e-9; out = np.zeros(len(D))
    M = np.stack([np.broadcast_to(a, D.shape)[ok], cr[ok] / n[ok, None], D[ok]], axis=2)
    s = np.linalg.svd(M, compute_uv=False); out[ok] = s[:, -1] / s[:, 0]
    return out

def dls_inverse(J, lam=0.08):
    """Damped least-squares (singularity-robust) inverse, used by control_map.py."""
    return J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(3))

# ---------------------------------------------------------------- workspace: arm directions inside today's soft limits
S1_SOFT, S2_SOFT = (-15, 140), (-55, 88)          # DESIGN_STATE §3
def workspace_dirs(step=2.5):
    D = []
    for t1 in np.arange(S1_SOFT[0], S1_SOFT[1] + 1e-6, step):
        for t2 in np.arange(S2_SOFT[0], S2_SOFT[1] + 1e-6, step):
            D.append(fk((t1, t2, 0)) @ Z)
    D = np.array(D)
    # de-duplicate the polar cluster at S2 ~ 0 by snapping to a 1-deg direction grid (uniform-ish area weighting)
    key = np.round(D * 60).astype(int); _, idx = np.unique(key, axis=0, return_index=True)
    return D[np.sort(idx)]

# S1 actuator body (AK70-10, verified Ø89 x 50.25): today the output face is at -56 along the S1 axis below P.
MOTOR_R, MOTOR_T0, MOTOR_T1 = 44.5, 56.0, 106.25
PLATE_FRONT_X = -88.0                              # chassis plate front face (DESIGN_STATE §5)
def motor_min_x(a):
    """Most negative x reached by the S1 motor cylinder whose axis runs from P along -a."""
    ax = a[0]; radial = MOTOR_R * np.sqrt(max(0.0, 1 - ax * ax))
    return min(-t * ax for t in (MOTOR_T0, MOTOR_T1)) - radial

def score(beta, gamma, D):
    a = mount(beta, gamma) @ Z
    return inv_cond_dirs(D, a), a

REC_BETA, REC_GAMMA = 0.0, 0.0       # adopted: no tilt (tilts fail reachability); park off the singularity instead
S2_REACH = 92.0                       # deg: max arm angle from the S1 axis (verified S2 hard stop / yoke floor)

def workspace_weighted(s2hi=88.0, step=2.5):
    """Arm directions inside S1 soft limits x S2 -55..s2hi; forward-over poses (S2 >= 50) weighted x2 (reaching in front)."""
    D, W = [], []
    for t1 in np.arange(S1_SOFT[0], S1_SOFT[1] + 1e-6, step):
        for t2 in np.arange(S2_SOFT[0], s2hi + 1e-6, step):
            D.append(fk((t1, t2, 0)) @ Z); W.append(2.0 if t2 >= 50 else 1.0)
    D, W = np.array(D), np.array(W); key = np.round(D * 60).astype(int); _, i = np.unique(key, axis=0, return_index=True); i = np.sort(i)
    return D[i], W[i]

def wpct(x, w, p):
    o = np.argsort(x); c = np.cumsum(w[o]) / w.sum(); return float(x[o][min(len(x) - 1, np.searchsorted(c, p / 100))])

if __name__ == "__main__":
    WS = {"A_current_S2_88": workspace_weighted(88.0), "B_future_S2_110": workspace_weighted(110.0)}
    D, Wt = WS["A_current_S2_88"]
    ic0, a0 = score(0, 0, D)
    table = []
    for beta, gamma in ((0, 0), (30, 0), (45, 0), (60, 0), (45, -20), (60, -20), (75, -20), (90, 0)):
        row = dict(beta=beta, gamma=gamma)
        for k, (Dk, Wk) in WS.items():
            ic, a = score(beta, gamma, Dk)
            row[k] = dict(p5w=round(wpct(ic, Wk, 5), 3), min=round(float(ic.min()), 3), frac_lt_0p2=round(float((ic < 0.2).mean()), 3))
        Dk = WS["A_current_S2_88"][0]; ang = np.degrees(np.arccos(np.clip(Dk @ a, -1, 1)))
        row["reach_fail_frac"] = round(float((ang > S2_REACH).mean()), 3); row["S2_needed_max"] = round(float(ang.max()), 1)
        row["axis"] = a.round(3).tolist(); row["motor_min_x_mm"] = round(motor_min_x(a), 1); row["P_shift_mm"] = round(max(0.0, PLATE_FRONT_X - motor_min_x(a)), 1)
        table.append(row)
    best = next(r for r in table if r["beta"] == REC_BETA and r["gamma"] == REC_GAMMA)
    icb, ab = score(REC_BETA, REC_GAMMA, D); best = dict(best, mn=float(icb.min()), p5=wpct(icb, Wt, 5), p_shift=best["P_shift_mm"])
    res = [dict(beta=float(b_), gamma=REC_GAMMA, p5=wpct(score(b_, REC_GAMMA, D)[0], Wt, 5), p_shift=max(0.0, PLATE_FRONT_X - motor_min_x(score(b_, REC_GAMMA, D)[1]))) for b_ in np.arange(-10, 90.1, 2.5)]
    # ----------------------------------------------------- joint-speed blow-up for a homologous "flex forward" intent
    # ask for omega = 60 deg/s about the wearer's mediolateral axis (+y -> arm swings forward over, project sign) near rest
    def qdot_needed(R0, q):
        J = jacobian(q, R0); w = np.radians(60) * np.array([0, -1.0, 0])     # forward flexion of an arm pointing up
        w2 = np.radians(60) * np.array([1.0, 0, 0])                           # abduction-like: about the AP axis
        out = []
        for ww in (w, w2):
            try: qd = np.linalg.solve(J, ww)
            except np.linalg.LinAlgError: qd = np.full(3, np.inf)
            out.append(float(np.degrees(np.abs(qd).max())))
        return out
    blow = []
    for s2 in (0.5, 2, 5, 10, 20, 45):
        blow.append(dict(S2=s2, flex_deg_s=qdot_needed(np.eye(3), (0, s2, 0))[0], abd_deg_s=qdot_needed(np.eye(3), (0, s2, 0))[1]))

    # ----------------------------------------------------- compliant capsule pads (in front of the rigid stops)
    # Arm beyond S2 incl. payload 1.63 kg (DESIGN_STATE §3); treat as distributed over 0..0.5 m with 0.5 kg at the tip.
    m_arm, m_tip, L = 1.13, 0.50, 0.50
    I_P = m_arm * L ** 2 / 3 + m_tip * L ** 2                     # kg m^2 about P (rod + tip mass)
    w_cap = 0.5 / L                                               # rad/s at the 0.5 m/s tip cap
    KE = 0.5 * I_P * w_cap ** 2                                   # J
    tau_pk = 24.8                                                 # AK70-10 peak, N m (verified)
    r_post = 0.054                                                # S1 stop post radius, m (SPEC_ball_stops)
    dtheta = np.radians(4.0)                                      # pad travel: contact at soft+5, bottoms at soft+9
    F_pk = tau_pk / r_post; x_pad = r_post * dtheta * 1e3         # N, mm
    E_pad = 0.5 * F_pk * x_pad * 1e-3                             # J if linear; progressive pad stores less, still >> KE
    # TPU 95A pad, compressive design stress 6 MPa at ~35 % strain (hand value: Shore 95A TPUs, verify by test)
    sigma, strain = 6.0, 0.35
    t_pad = x_pad / strain; A_pad = F_pk / sigma
    capsule = dict(I_P_kgm2=round(I_P, 4), KE_at_tip_cap_J=round(KE, 4), F_peak_on_post_N=round(F_pk), pad_travel_mm=round(x_pad, 2),
                   stored_energy_linear_J=round(E_pad, 3), pad_thickness_mm=round(t_pad, 1), pad_area_mm2=round(A_pad),
                   suggested_pad="TPU 95A, domed, %.0f mm thick, ~%.0f x %.0f mm footprint, bonded to the rigid stop face" % (np.ceil(t_pad), np.ceil(np.sqrt(A_pad * 1.3)), np.ceil(np.sqrt(A_pad / 1.3))))

    out = dict(
        current=dict(axis=a0.round(3).tolist(), p5w=wpct(ic0, Wt, 5), min=float(ic0.min()), mean=float(ic0.mean()),
                     frac_below_0p2=float((ic0 < 0.2).mean()), motor_min_x=motor_min_x(a0)),
        recommended=best, candidates=table,
        homologous_speed_blowup_current=blow,
        capsule_pads=capsule, n_dirs=len(D))
    # park-pose analysis (adopted fix): dexterity at the park pose and over the workspace outside a cap around 'up'
    import rom_limits as rl
    park_ic = inv_cond_dir(fk((rl.PARK["S1"], rl.PARK["S2"], 0)) @ Z, Z)
    out["park"] = dict(pose=rl.PARK, inv_cond_at_park=round(float(park_ic), 3),
                       note="from park a lateral intent tracks immediately; only poses within ~10 deg of vertical are slow (DLS)")
    os.makedirs(os.path.join(ROOT, "stl", "control"), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, "stl", "control", "biomimetic_shoulder.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))

    # ----------------------------------------------------- plot
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(14, 4.8))
    def dirs_plot(ax, ic, a, title):
        el = np.degrees(np.arcsin(np.clip(D[:, 2], -1, 1))); az = np.degrees(np.arctan2(D[:, 1], -D[:, 0]))
        sc = ax.scatter(az, el, c=ic, cmap="viridis", vmin=0, vmax=1, s=9)
        for sgn, lab in ((1, "+a"), (-1, "-a")):
            v = sgn * a; ax.plot(np.degrees(np.arctan2(v[1], -v[0])), np.degrees(np.arcsin(v[2])), "rx", ms=12, mew=3)
            ax.annotate("singular " + lab, (np.degrees(np.arctan2(v[1], -v[0])), np.degrees(np.arcsin(v[2]))), color="r", xytext=(5, 5), textcoords="offset points")
        ax.set_xlim(-180, 180); ax.set_ylim(-90, 90); ax.set_xlabel("azimuth (0 = forward over shoulder, +90 = lateral)"); ax.set_ylabel("elevation (deg)")
        ax.set_title(title, fontsize=10); ax.grid(alpha=.3); return sc
    ax1 = fig.add_subplot(1, 3, 1); dirs_plot(ax1, ic0, a0, "Current S1 = z: 1/κ over the soft-limit workspace\nweighted 5th pct %.2f, min %.2f (rest pose is singular)" % (wpct(ic0, Wt, 5), ic0.min()))
    ax2 = fig.add_subplot(1, 3, 2); sc = dirs_plot(ax2, icb, ab, "Tilted mount β=%.1f°, γ=%.0f°\n5th pct %.2f, min %.2f, P shift %.0f mm" % (best["beta"], best["gamma"], best["p5"], best["mn"], best["p_shift"]))
    plt.colorbar(sc, ax=ax2, label="1/κ (1 = isotropic, 0 = singular)")
    ax3 = fig.add_subplot(1, 3, 3)
    b = np.array([r["beta"] for r in res]); p5 = np.array([r["p5"] for r in res]); ps = np.array([r["p_shift"] for r in res])
    ax3.plot(b, p5, "b-", lw=2, label="5th-pct 1/κ"); ax3.set_xlabel("mount tilt β (deg, S1 axis toward +x)"); ax3.set_ylabel("weighted 5th-pct 1/κ", color="b")
    ax3b = ax3.twinx(); ax3b.plot(b, ps, "r--", lw=2); ax3b.set_ylabel("P must move back (mm)", color="r")
    ax3.axvline(best["beta"], color="k", alpha=.4); ax3.set_title("Dexterity vs packaging (γ=%.0f°)" % best["gamma"], fontsize=10); ax3.grid(alpha=.3)
    plt.tight_layout(); plt.savefig(os.path.join(ROOT, "renders", "shoulder_singularity.png"), dpi=120)
