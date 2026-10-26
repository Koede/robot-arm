"""Elbow four-bar study (right arm; mm, deg). The XM540 stays fixed in the upper arm and drives the forearm through a planar four-bar:
crank (on the XM540 horn, ground pivot O2) -> coupler -> forearm lever (pivot = elbow axis O4 = (0, 230), rotates with E).
Plane: x-z of the upper arm, O4 = origin below; polar angle psi measured from +z toward +x; the forearm points along psi = -E (R = Ry(-E)).
Lever tip = O4 + c*u(beta - E), u(psi) = (sin psi, cos psi); crank tip = O2 + a*u(theta); |lever tip - crank tip| = b.
Torque ratio (ideal, frictionless) MA = tau_elbow / tau_motor = |d theta / d E|.  Direct drive: MA = 1.
Gates (task): E 0..145 (soft) covered by a crank rotation <= 300 deg; transmission angle mu (coupler/lever) 30..150 over the soft range;
no toggle (mu and the crank-side angle nu away from 0/180, theta(E) monotonic) over the hard range -5..150.
Required elbow torque: same method as verify_distal.py (static, link horizontal, masses summed from the meshes, 0.5 kg at Z_TIP 575);
verify_distal.json is not in the repo (needs verify_claw.json), so the sum is redone here with the same densities and bought masses.
Writes stl/elbow_fourbar/fourbar.json (new file).  Run: python3 elbow_fourbar.py   (1 core, a few minutes)."""
import os, sys, json, numpy as np
from scipy.optimize import differential_evolution
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "..", "stl", "elbow_fourbar"); os.makedirs(OUT, exist_ok=True)
import rom_limits as rl

XM_X, XM_Z, XM_D = 58.5, 33.5, 44.0
TAU_STALL = 10.6                    # XM540-W270 @ 12 V
TAU_CONT = 0.5 * TAU_STALL          # task: 0.5 x stall continuous
E_SOFT = np.arange(rl.SOFT["E"][0], rl.SOFT["E"][1] + 1e-9, 1.0)
E_HARD = np.arange(rl.HARD["E"][0], rl.HARD["E"][1] + 1e-9, 1.0)
SOFTM = (E_HARD >= rl.SOFT["E"][0]) & (E_HARD <= rl.SOFT["E"][1])
MU_LO, MU_HI = 30.0, 150.0          # gate over the soft range
TOG = 12.0                          # min distance of mu / nu from 0 and 180 over the hard range (no toggle)
TRAVEL_MAX = 300.0
BOSS_CLR = 9.0                      # elbow-axis boss + bearing radius: the O4 shaft must stay this far outside the motor rectangle

# ------------------------------------------------------------------------------------------------ required elbow torque
def required_torque():
    """verify_distal.py section 'torques' reproduced from the meshes: everything distal of the elbow axis (ME, MW1, MW2, MW3) + 0.5 kg at Z_TIP."""
    import arm_kin as ak
    G = ak.groups(); LAY = ak.LAY; ZE = ak.ZE; Z_TIP = LAY["Z_TIP"]
    CLAW = dict(petg_cf=1.30, pa_cf=1.20, petg=1.27, tpu=1.21)
    MAT = {"hand_tray": "petg_cf", "gear_cover": "petg_cf", "servo_tab": "petg_cf", "claw_pinion": "pa_cf", "claw_idler": "pa_cf", "claw_sector_A": "pa_cf", "claw_sector_B": "pa_cf",
           "tpu_stop_sleeve": "tpu", "spacer_A_mid": "petg", "spacer_A_top": "petg", "spacer_B_low": "petg", "spacer_B_top": "petg",
           "prox_": "pa_cf", "dist_": "pa_cf", "link_drive_": "pa_cf", "link_couple_": "pa_cf", "claw_rotor": "pa_cf", "claw_ring": "pa_cf", "stop_cap": "petg_cf"}
    def dens(k):
        if k.startswith("tpu"): return 1.21
        b = k.rstrip("0123456789"); return CLAW[MAT[b]] if b in MAT else 1.27
    PROXY = ("sts", "xm_", "ak_", "shaft", "pin_", "stop_pin", "bearing", "scr_")
    CH = ["ME", "MW1", "MW2", "MW3"]
    items = {g: [] for g in CH}
    for g in CH:
        for k, m in G[g].items():
            if k.startswith(PROXY): continue
            vol = abs(m.volume) / 1e3
            if vol <= 0: continue
            c = m.center_mass if m.is_watertight else m.bounds.mean(0)
            items[g].append((k, vol * dens(k) / 1e3, float(c[2])))
    pz = lambda g, k: float(G[g][k].bounds.mean(0)[2])
    SERVO = 0.055
    HW_CLAW = 8.4e-3                    # claw pin / screw hardware (verify_claw.json missing: 8.4 g quoted in verify_distal.py's comment)
    bought = {"ME": [("sts_w1", SERVO, pz("ME", "sts_w1")), ("wiring ME", 0.025, 330.0)],
              "MW1": [("sts_w2", SERVO, pz("MW1", "sts_w2")), ("6808-2RS", 0.022, float(LAY["BRG_Z0"]) + 3.5), ("wiring MW1", 0.030, 380.0)],
              "MW2": [("sts_w3", SERVO, pz("MW2", "sts_w3"))],
              "MW3": [("sts_g", SERVO, pz("MW3", "sts_g")), ("claw hardware", HW_CLAW, 520.0)]}
    for g, l in bought.items(): items[g] += l
    m = sum(i[1] for g in CH for i in items[g]); M = sum(i[1] * (i[2] - ZE) / 1e3 for g in CH for i in items[g])     # kg, kg.m
    pay = 0.5; g0 = 9.81
    tau = (abs(M) + pay * (Z_TIP - ZE) / 1e3) * g0
    forks = sum(i[1] for i in items["ME"] if i[0].startswith("fork_"))
    return dict(tau_Nm=round(tau, 3), m_distal_g=round(m * 1e3, 1), cg_above_E_mm=round(M / m * 1e3, 1), payload_kg=pay, z_tip_mm=Z_TIP, fork_mass_g=round(forks * 1e3, 1),
                gravity_term_Nm=round(abs(M) * g0, 3), payload_term_Nm=round(pay * (Z_TIP - ZE) / 1e3 * g0, 3),
                source="verify_distal.py torque section (static, link horizontal): sum over ME+MW1+MW2+MW3 mesh volumes x density + bought masses (STS3215 55 g, wiring) about the elbow axis z=230, + 0.5 kg at Z_TIP 575")

# ------------------------------------------------------------------------------------------------ four-bar kinematics
def u(psi): p = np.radians(psi); return np.stack([np.sin(p), np.cos(p)], -1)
def solve(par, E, s):
    """par = (a, b, c, g, gamma, beta). Returns dict of arrays over E, or None if the loop cannot close."""
    a, b, c, g, gam, beta = par
    O2 = g * u(gam); L = c * u(beta - E)
    d = L - O2; D = np.hypot(d[:, 0], d[:, 1])
    if np.any(D > a + b - 0.5) or np.any(D < abs(a - b) + 0.5): return None
    ca = np.clip((a * a + D * D - b * b) / (2 * a * D), -1, 1)
    th = np.degrees(np.arctan2(d[:, 0], d[:, 1])) + s * np.degrees(np.arccos(ca))
    th = np.degrees(np.unwrap(np.radians(th)))
    C = O2 + a * u(th)
    cp = C - L; lv = -L                                           # at the lever tip: toward crank tip, toward O4
    mu = np.degrees(np.arccos(np.clip((cp * lv).sum(1) / (np.hypot(*cp.T) * np.hypot(*lv.T)), -1, 1)))
    c1 = O2 - C; c2 = L - C
    nu = np.degrees(np.arccos(np.clip((c1 * c2).sum(1) / (np.hypot(*c1.T) * np.hypot(*c2.T)), -1, 1)))
    dth = np.gradient(th, E)
    return dict(E=E, theta=th, mu=mu, nu=nu, dth=dth, O2=O2, C=C, L=L)

def motor_ok(par, orient):
    """O4 must stay >= BOSS_CLR outside the motor rectangle centred on O2 (orient 0: 58.5 along x, 33.5 along z; 90: swapped). >= 0 is ok."""
    a, b, c, g, gam, beta = par; O2 = g * u(gam)
    hx, hz = (XM_X / 2, XM_Z / 2) if orient == 0 else (XM_Z / 2, XM_X / 2)
    return max(abs(O2[0]) - hx, abs(O2[1]) - hz) - BOSS_CLR

def evaluate(par, s, orient=0):
    """-> (violation, info). violation = 0 when every gate is met."""
    r = solve(par, E_HARD, s)
    if r is None: return 1e3, None
    mu, nu, dth = r["mu"], r["nu"], r["dth"]
    v = 0.0
    v += np.sum(np.maximum(MU_LO + 2 - mu[SOFTM], 0) + np.maximum(mu[SOFTM] - (MU_HI - 2), 0)) / 10
    v += np.sum(np.maximum(TOG - mu, 0) + np.maximum(mu - (180 - TOG), 0)) / 10
    v += np.sum(np.maximum(TOG - nu, 0) + np.maximum(nu - (180 - TOG), 0)) / 10
    mono = dth * np.sign(np.median(dth))
    v += np.sum(np.maximum(0.05 - mono, 0)) * 10                    # theta(E) strictly monotonic, MA > 0.05
    trav = abs(r["theta"][SOFTM][-1] - r["theta"][SOFTM][0])
    v += max(trav - (TRAVEL_MAX - 3), 0) / 5
    v += max(-motor_ok(par, orient), 0) / 3
    ma = np.abs(dth)
    return float(v), dict(res=r, travel=float(trav), ma_min=float(ma[SOFTM].min()), ma_mean=float(ma[SOFTM].mean()), ma_max=float(ma[SOFTM].max()))

def search(bounds, s, orient, seed, maxiter=200):
    def f(x):
        v, info = evaluate(x, s, orient)
        if info is None: return 1e3
        return -min(info["ma_min"], 4.0) + 5 * v
    r = differential_evolution(f, bounds, seed=seed, maxiter=maxiter, popsize=20, tol=1e-9, polish=False, workers=1, updating="immediate")
    v, info = evaluate(r.x, s, orient)
    return r.x, v, info

def summarize(par, s, orient, tau_req):
    v, info = evaluate(par, s, orient); r = info["res"]
    ma = np.abs(r["dth"])
    tbl = [dict(E=float(e), theta=round(float(t - r["theta"][SOFTM][0]), 1), mu=round(float(m), 1), nu=round(float(n), 1), MA=round(float(k), 3),
                tau_stall_Nm=round(float(k) * TAU_STALL, 2), tau_cont_Nm=round(float(k) * TAU_CONT, 2))
           for e, t, m, n, k in zip(E_HARD, r["theta"], r["mu"], r["nu"], ma) if abs(e % 5) < 1e-9]
    return dict(params=dict(a=round(float(par[0]), 2), b=round(float(par[1]), 2), c=round(float(par[2]), 2), ground_g=round(float(par[3]), 2),
                            ground_dir_deg=round(float(par[4]) % 360, 2), lever_beta_deg=round(float(par[5]), 2), branch=int(s), motor_orient=int(orient)),
                O2_xz=[round(float(x), 2) for x in par[3] * u(par[4])], violation=round(v, 4), feasible=bool(v < 1e-6),
                crank_travel_soft_deg=round(info["travel"], 1), crank_travel_hard_deg=round(float(abs(r["theta"][-1] - r["theta"][0])), 1),
                mu_soft=[round(float(r["mu"][SOFTM].min()), 1), round(float(r["mu"][SOFTM].max()), 1)], mu_hard=[round(float(r["mu"].min()), 1), round(float(r["mu"].max()), 1)],
                nu_hard=[round(float(r["nu"].min()), 1), round(float(r["nu"].max()), 1)],
                MA_soft=dict(min=round(info["ma_min"], 3), mean=round(info["ma_mean"], 3), max=round(info["ma_max"], 3)),
                E_at_MA_min=float(E_HARD[SOFTM][np.argmin(ma[SOFTM])]),
                MA_at=dict(E0=round(float(ma[E_HARD == 0][0]), 3), E145=round(float(ma[E_HARD == 145][0]), 3)),
                torque_available_Nm=dict(stall_min=round(info["ma_min"] * TAU_STALL, 2), cont_min=round(info["ma_min"] * TAU_CONT, 2),
                                         stall_E0=round(float(ma[E_HARD == 0][0]) * TAU_STALL, 2), stall_E145=round(float(ma[E_HARD == 145][0]) * TAU_STALL, 2)),
                SF_vs_required=dict(stall_min=round(info["ma_min"] * TAU_STALL / tau_req, 2), cont_min=round(info["ma_min"] * TAU_CONT / tau_req, 2)),
                table=tbl)

if __name__ == "__main__":
    req = required_torque(); tau_req = req["tau_Nm"]
    print("required elbow torque", req)
    direct = dict(MA=1.0, stall_Nm=TAU_STALL, cont_Nm=TAU_CONT, SF_stall=round(TAU_STALL / tau_req, 2), SF_cont=round(TAU_CONT / tau_req, 2),
                  note="direct drive: MA = 1 for every E (horn on the elbow axis); crank travel = elbow travel (145 deg soft)")
    out = dict(required=req, direct=direct, gates=dict(travel_max=TRAVEL_MAX, mu=[MU_LO, MU_HI], toggle_margin=TOG, E_soft=list(rl.SOFT["E"]), E_hard=list(rl.HARD["E"]),
                                                      boss_clearance_mm=BOSS_CLR, tau_stall=TAU_STALL, tau_cont=TAU_CONT))
    # A: free ground pivot anywhere around the elbow axis; B: motor kept on the upper-arm centreline, proximal (O2 below the elbow axis, |x| <= ~4)
    fams = {"A_free": [(10, 60), (20, 150), (15, 90), (25, 90), (0, 360), (-180, 180)],
            "B_inline_proximal": [(10, 60), (20, 150), (15, 90), (25, 52), (176, 184), (-180, 180)],
            # C: compact: every link tip inside ~ the ball + 14 mm (lever <= 56, crank <= 34), motor in line and proximal
            "C_compact_inline": [(12, 34), (20, 90), (20, 56), (25, 52), (176, 184), (-180, 180)]}
    for name, bnd in fams.items():
        best = None
        for s in (+1, -1):
            for orient in (0, 90):
                for seed in (1, 2):
                    x, v, info = search(bnd, s, orient, seed)
                    if info is None: continue
                    key = (v > 1e-6, round(-info["ma_min"], 2) if v < 1e-6 else v, orient)      # ties: prefer orient 0 (long side across the arm, fewer shell changes)
                    print(name, "s", s, "orient", orient, "seed", seed, "viol %.3f" % v, "MAmin %.3f travel %.1f" % (info["ma_min"], info["travel"]), flush=True)
                    if best is None or key < best[0]: best = (key, x, s, orient)
        if best is None: out[name] = dict(feasible=False); continue
        _, x, s, orient = best
        out[name] = summarize(x, s, orient, tau_req)
        print(name, json.dumps({k: v for k, v in out[name].items() if k != "table"}, indent=1))
    json.dump(out, open(os.path.join(OUT, "fourbar.json"), "w"), indent=1)
    print("wrote", os.path.join(OUT, "fourbar.json"))
