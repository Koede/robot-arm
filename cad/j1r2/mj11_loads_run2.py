"""MJ11 harness / body loads (2026-11-03, SHOULDER / HARNESS).  harness_loads.py (2-D sagittal statics of the J1 rev 0 rig: chassis plate, wing tongue, S2 balancer) and body_loads.py need the retired rig and sensor_pods.py /
electronics_fit.py outputs; they are NOT run.  Their load cases are PORTED onto the MJ11 3-D frame model (mj_frame.py harness + mj_concepts3.mj11_frame) and onto the same 3-contact-group statics (top F1+F4, front F2+F3, rail) with the MJ11 mass lumps.
Robot frame, mm / N / kPa / MPa.  Loads: gravity of arm + payload + stage + mount (mj_frame.arm_masses) + rig extras (rail strip, clasp, pods, bridge); dynamic factor 1.3 as harness_loads.py (arm accelerations at the 0.5 m/s tip cap, hand estimate); x2 for bolts (task).
usage: J1R2_ROOT=<root> python3 mj11_loads.py  -> stl/j1r2_mounts/MJ11/loads.json"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import j1r2_params as P, mj_frame as M, mj_concepts as C, mj_concepts3 as C3
from mj_concepts import link_bolts, rail_node_name, clasp_bracket_frame, pad_nodes
G_ = 9.81; X0, XB, Y1, Z1 = C3.X0, C3.XB, C3.Y1, C3.Z1
MOUNT_G = 186.7                                         # MJ11 mount mass (mj_eval metrics)
LIM = dict(sustained=14.7, transient=44.0, axilla=4.3, strap_line=4.3)          # SPEC_safety / SPEC_body_loads
INSERT_PULL = 1000.0                                   # N, typical M4 heat-set insert in PA-CF (UNVERIFIED, task statement)
BASE_CFG = dict(name="MJ11 as adopted (2 pairs z -160/-220, 8 mm apart in y)", bolt_z=(-160.0, -220.0), dy=4.0, tab=False)
def rig_extras():
    """masses (g) the J1 rev 0 harness_loads carried that still exist on MJ11, placed at ASSUMED positions where the CAD is not in the repo (pods)."""
    w = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")); import clib as L
    strip = L.I(w, L.box(-300, 100, -150, -100.0, -500, 100)); m_strip = abs(strip.volume) * 1.15e-3
    cl = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "clasp_frame_R.stl")); m_clasp = abs(cl.volume) * 1.27e-3
    return dict(rail_strip_g=round(m_strip, 1), clasp_g=round(m_clasp, 1), bridge_half_g=14.0, pods_g=78.0 + 212.0 + 390.0, strip_cg=strip.center_mass.tolist(), clasp_cg=cl.center_mass.tolist())
EX = None
def frame_variant(fr, side, cfg):
    """C3.mj11_frame with a configurable bolt layout.  Bolt pairs sit at rail nodes z (multiples of 20); each pair = 2 bolts +-dy in y about y = -109; cfg['tab'] = pad tab (bolts beyond the R66 disc)."""
    mp = (lambda p: np.asarray(p, float)) if side == "R" else M.mirror; pad = C.pad_nodes(fr, side); Y, Z, skin, inner, plane = C3.seat_surfaces()
    ymean = float(np.mean(C3.BOLT_Y))
    for z in cfg["bolt_z"]:
        nb = f"{side}_pb{int(z)}"; fr.node(nb, mp(np.array([X0, ymean, z]))); fr.rigid(pad, nb, tag="pad"); link_bolts(fr, nb, rail_node_name(side, z), [(-cfg["dy"], 0), (cfg["dy"], 0)], tag="iface")
    pts = []; step = 16.0
    for y in np.arange(Y1 - C3.PAD_R + 8, Y1 + C3.PAD_R - 7, step):
        for z in np.arange(Z1 - C3.PAD_R + 8, Z1 + C3.PAD_R - 7, step):
            if math.hypot(y - Y1, z - Z1) <= C3.PAD_R - 4 and y >= C3.RAIL_EDGE_Y + 2: pts.append((y, z))
    k_a = M.E_FOAM / C3.LINER_T; A_i = step * step
    for i, (y, z) in enumerate(pts):
        xs = float(M.back_x(y, z)); n = f"{side}_ln{i}"; fr.node(n, mp(np.array([xs + C3.LINER_T / 2, y, z]))); fr.rigid(pad, n, tag="pad"); fr.spring(n, [1.0, 0, 0], k_a * A_i, area=A_i, label=f"{side}_liner{i}", kind="liner")
    an = f"{side}_clasp_anchor"; fr.node(an, mp(C.pad_pt(C3.CLASP_PSI, C3.PAD_R - 8, x=XB - 3.0))); fr.rigid(pad, an, tag="pad"); clasp_bracket_frame(fr, side, an); return pad
def build(cfg=BASE_CFG):
    fr = M.Frame(); M.harness(fr, "R"); M.harness(fr, "L"); M.spine_bridge(fr); pads = {s: frame_variant(fr, s, cfg) for s in ("R", "L")}; return fr, pads
def add_rig(fr, side="both"):
    global EX
    if EX is None: EX = rig_extras()
    for s in (("R", "L") if side == "both" else (side,)):
        rn = [n for n in fr.idx if n.startswith(f"{s}_rail")]; zs = sorted(rn, key=lambda n: fr.nodes[fr.idx[n]][2])
        for n in zs: fr.force(n, [0, 0, -EX["rail_strip_g"] * 1e-3 * G_ / len(zs)])
        fr.force(f"{s}_clasp_hub", [0, 0, -EX["clasp_g"] * 1e-3 * G_]); fr.force(f"{s}_rail-180", [0, 0, -EX["bridge_half_g"] * 1e-3 * G_]); fr.force(f"{s}_rail-240", [0, 0, -EX["pods_g"] * 1e-3 * G_])
def solve_case(cfg, th_r, ph_r, pay_r, th_l, ph_l, pay_l, rig=True, mount_g=MOUNT_G, torque_extra=None):
    fr, pads = build(cfg)
    for s, (th, ph, pay) in (("R", (th_r, ph_r, pay_r)), ("L", (th_l, ph_l, pay_l))): M.apply_arm(fr, pads[s], s, th, ph, pay, extra_g=mount_g)
    if rig: add_rig(fr)
    if torque_extra: fr.force(pads["R"], [0, 0, 0], [torque_extra, 0, 0])
    u = fr.solve(); return fr, pads, u
def report(fr, pads, u, dyn=1.0):
    """pressures per kind (right side), full rail profile, bolt-pair forces (all six end components) and their per-bolt tension / shear."""
    sp = fr.spring_forces(); out = dict(contact={}, rail_profile=[], bolts=[])
    for kind in ("rail", "top", "front", "axilla", "liner", "shear"):
        ps = [s_ for s_ in sp if s_["kind"] == kind and s_["label"].startswith("R_") and s_["p_kPa"] is not None] if kind != "shear" else []
        if ps: out["contact"][kind] = dict(max_kPa=round(dyn * max(s_["p_kPa"] for s_ in ps), 2), min_kPa=round(dyn * min(s_["p_kPa"] for s_ in ps), 2), sum_N=round(dyn * sum(s_["F"] for s_ in ps), 1), area_mm2=round(sum(s_["F"] / s_["p_kPa"] * 1e3 for s_ in ps if abs(s_["p_kPa"]) > 1e-9)))
    rails = sorted([s_ for s_ in sp if s_["kind"] == "rail" and s_["label"].startswith("R_")], key=lambda s_: int(s_["label"].replace("R_rail", ""))); out["rail_profile"] = [(int(s_["label"].replace("R_rail", "")), round(dyn * s_["p_kPa"], 2)) for s_ in rails]
    # shear (in-plane) forces on the right rail: sum of the shear springs
    shear = np.zeros(3)
    for sp_ in fr.springs:
        if sp_["kind"] == "shear" and fr.nodes[sp_["n"]][1] > M.SPINE_Y and sp_["label"] == "":
            pass
    for b in fr.beams:
        if b["tag"] != "iface" or fr.nodes[b["a"]][1] < M.SPINE_Y: continue
        R_, Lb, kg = b["_RL"]; ue = np.r_[u[b["a"]], u[b["b"]]]; f = np.kron(np.eye(4), R_) @ (kg @ ue); N, Vy, Vz, Tt, My, Mz = f[:6]; bo = np.asarray(b["bolts"], float); n_b = len(bo)
        su = float((bo[:, 0] ** 2).sum()) or 1e9; sv = float((bo[:, 1] ** 2).sum()) or 1e9; Fi = N / n_b + My * bo[:, 1] / sv - Mz * bo[:, 0] / su
        out["bolts"].append(dict(z=round(float(fr.nodes[b["a"]][2])), n=n_b, tension_max_N=round(dyn * float(np.abs(Fi).max()), 1), shear_per_bolt_N=round(dyn * float(math.hypot(Vy, Vz)) / n_b, 1), N=round(dyn * N, 1), Vy=round(dyn * Vy, 1), Vz=round(dyn * Vz, 1), T_Nmm=round(dyn * Tt), My_Nmm=round(dyn * My), Mz_Nmm=round(dyn * Mz)))
    return out

# ------------------------------------------------------------------ Coulomb contact on the tangential (foam-shear) springs
def solve_friction(fr, mu=0.4, p0=0.0, dyn=1.0, mu_pad=None, iters=60, verbose=False):
    """Linear solve with Coulomb friction at every node that has a normal spring AND tangential springs (rail nodes, clasp pads): normal force N = p0*A + compression increment; if the tangential foam-shear force exceeds mu*N the node slips
    (tangential spring removed, constant force mu*N along the trial direction).  Loads scaled by dyn.  Returns u, per-node table, slip count.  p0 = uniform strap/foam pre-pressure (kPa) on rail, liner, top and front pads."""
    mu_pad = mu if mu_pad is None else mu_pad
    loads0 = {i: f.copy() for i, f in fr.loads.items()}
    for i in fr.loads: fr.loads[i] = fr.loads[i] * dyn
    nodes = {}
    for si, sp in enumerate(fr.springs):
        if sp["kind"] in ("rail", "top", "front"): nodes.setdefault(sp["n"], {})["normal"] = si
        elif sp["kind"] == "shear": nodes.setdefault(sp["n"], {}).setdefault("tang", []).append(si)
    cn = {n: d for n, d in nodes.items() if "normal" in d and "tang" in d}
    k0 = {si: fr.springs[si]["k"] for d in cn.values() for si in d["tang"]}; slip = {}                       # node -> unit trial direction
    extra = {}
    def run():
        saved = {i: f.copy() for i, f in fr.loads.items()}
        for n, (dirv, mag) in extra.items():
            f = fr.loads.setdefault(n, np.zeros(6)); f[:3] += dirv * mag
        u = fr.solve(); fr.loads = saved; return u
    for it in range(iters):
        for n, d in cn.items():
            for si in d["tang"]: fr.springs[si]["k"] = (1e-4 * k0[si]) if n in slip else k0[si]
        extra = {n: (slip[n][0], slip[n][1]) for n in slip}
        u = run(); changed = False; rows = {}
        for n, d in cn.items():
            nsp = fr.springs[d["normal"]]; A = nsp["area"]; Fn = -nsp["k"] * float(u[n, :3] @ nsp["d"]) + p0 * 1e-3 * A                  # N, compression positive (+ preload)
            ft = sum((-k0[si] * float(u[n, :3] @ fr.springs[si]["d"])) * fr.springs[si]["d"] for si in d["tang"])                    # trial tangential force on the rig (stick)
            m_ = mu_pad if nsp["kind"] != "rail" else mu; cap = max(0.0, m_ * Fn); mag = float(np.linalg.norm(ft)); rows[n] = dict(Fn=Fn, ft=mag, cap=cap, kind=nsp["kind"], area=A)
            if n in slip:
                # sliding node: the trial force from the stick springs at the CURRENT u is not defined (spring removed): test by the applied force direction against displacement
                tr = sum((-k0[si] * float(u[n, :3] @ fr.springs[si]["d"])) * fr.springs[si]["d"] for si in d["tang"]); trm = float(np.linalg.norm(tr))
                if trm < cap * 0.98:   # would stick again
                    del slip[n]; changed = True
                else:
                    nd = tr / trm if trm > 1e-9 else slip[n][0]
                    if abs(cap - slip[n][1]) > 1e-3 or np.linalg.norm(nd - slip[n][0]) > 1e-3: slip[n] = (nd, cap); changed = True
            elif mag > cap + 1e-9 and cap >= 0:
                slip[n] = (ft / mag, cap); changed = True
        if not changed: break
    for d in cn.values():
        for si in d["tang"]: fr.springs[si]["k"] = k0[si]
    fr.loads = loads0; fr.u = u
    return u, rows, slip
def pressures(fr, rows, p0, side="R"):
    out = {}
    for n, r in rows.items():
        if not fr.nodes[n][1] > M.SPINE_Y and side == "R": continue
        nm = [k for k, v in fr.idx.items() if v == n][0]
        if not nm.startswith(side + "_"): continue
        out[nm] = round((r["Fn"] / r["area"]) * 1e3, 2)
    return out

def hand_pos(th, ph): return M.hand_tip(th, ph)
def tang_table(fr, u, p0=0.0):
    """stick solution -> per-node tangential foam-shear force (N) and normal force (N, incl. uniform pre-pressure p0 kPa) for the right-side contact nodes."""
    nodes = {}
    for si, sp in enumerate(fr.springs):
        if sp["kind"] in ("rail", "top", "front"): nodes.setdefault(sp["n"], {})["normal"] = si
        elif sp["kind"] == "shear": nodes.setdefault(sp["n"], {}).setdefault("tang", []).append(si)
    rows = []
    for n, d in nodes.items():
        if "normal" not in d or "tang" not in d or fr.nodes[n][1] < M.SPINE_Y: continue
        nsp = fr.springs[d["normal"]]; nm = [k for k, v in fr.idx.items() if v == n][0]
        dN = -nsp["k"] * float(u[n, :3] @ nsp["d"]); ft = sum((-fr.springs[si]["k"] * float(u[n, :3] @ fr.springs[si]["d"])) * fr.springs[si]["d"] for si in d["tang"])
        rows.append(dict(node=nm, kind=nsp["kind"], A=nsp["area"], dN=dN, p_kPa=dN / nsp["area"] * 1e3, ft=float(np.linalg.norm(ft)), ftv=ft.tolist()))
    return rows
def with_shear_scale(fr, f, dyn=1.0):
    for sp in fr.springs:
        if sp["kind"] == "shear": sp["k"] = sp.setdefault("k0", sp["k"]) * f
    saved = {i: v.copy() for i, v in fr.loads.items()}
    for i in fr.loads: fr.loads[i] = fr.loads[i] * dyn
    u = fr.solve(); fr.loads = saved; return u
LCS = {"park (R 0.5 kg)": ((P.PARK["S1p"], P.PARK["S2"], 0.5), (P.PARK["S1p"], P.PARK["S2"], 0.0)),
       "reach th1 -90 (R 0.5 kg, L park)": ((-90.0, 0.0, 0.5), (P.PARK["S1p"], P.PARK["S2"], 0.0)),
       "reach th1 -55 (R 0.5 kg, L park)": ((-55.0, 0.0, 0.5), (P.PARK["S1p"], P.PARK["S2"], 0.0)),
       "both reach th1 -90 (0.5 kg each)": ((-90.0, 0.0, 0.5), (-90.0, 0.0, 0.5)),
       "arm up th1 0 (R 0.5 kg)": ((0.0, 0.0, 0.5), (P.PARK["S1p"], P.PARK["S2"], 0.0))}
def loaded(cfg, lc, rig=True, mount_g=MOUNT_G):
    fr, pads = build(cfg); (r, l) = LCS[lc]
    for sd, (a, b, c) in (("R", r), ("L", l)): M.apply_arm(fr, pads[sd], sd, a, b, c, extra_g=mount_g)
    if rig: add_rig(fr)
    return fr, pads
def cap_solve(fr, mu, p0, dyn=1.3):
    """friction-capacity limited shear: all tangential (foam-shear) springs are scaled by one factor s so that the right-side tangential demand sum|ft| equals the capacity sum mu*(p0*A + dN) (s = 1 if stick holds).  Uniform-slip-fraction approximation of Coulomb contact."""
    def ev(s):
        u = with_shear_scale(fr, s, dyn); tb = tang_table(fr, u); D = sum(r["ft"] for r in tb); C = sum(mu * max(0.0, p0 * 1e-3 * r["A"] + r["dN"]) for r in tb); return u, tb, D, C
    u, tb, D, C = ev(1.0)
    if D <= C: return 1.0, u, tb, D, C
    lo, hi = 0.003, 1.0
    for _ in range(16):
        mid = (lo * hi) ** 0.5; u2, tb2, D2, C2 = ev(mid)
        if D2 > C2: hi = mid
        else: lo = mid
    u, tb, D, C = ev(lo); return lo, u, tb, D, C
def summarise(fr, pads, u, tb, p0, dyn=1.3):
    sp = fr.spring_forces(); g = lambda kind: [s_["p_kPa"] + p0 for s_ in sp if s_["kind"] == kind and s_["label"].startswith("R_")]       # p0 = uniform pre-pressure on rail, liner, top, front (NOT the axilla cradle: strap-only)
    top = {s_["label"]: round(s_["p_kPa"] + p0, 1) for s_ in sp if s_["kind"] == "top" and s_["label"].startswith("R_")}
    fr_ = {s_["label"]: round(s_["p_kPa"] + p0, 1) for s_ in sp if s_["kind"] == "front" and s_["label"].startswith("R_")}
    ax = [s_["p_kPa"] for s_ in sp if s_["kind"] == "axilla" and s_["label"].startswith("R_")]
    rb = report(fr, pads, u, 1.0)["bolts"]
    return dict(top=top, front=fr_, rail_min=round(min(g("rail")), 1), rail_max=round(max(g("rail")), 1), liner_min=round(min(g("liner")), 1), liner_max=round(max(g("liner")), 1), axilla=round(ax[0], 1),
                bolt_tension_max_N=max(b["tension_max_N"] for b in rb), bolt_shear_per_bolt_N=max(b["shear_per_bolt_N"] for b in rb), rail_shear_N=round(sum(r["ft"] for r in tb if r["kind"] == "rail"), 1),
                lift_off=bool(min(g("rail")) < 0 or min(g("liner")) < 0 or min(fr_.values()) < 0))

VARIANTS = [dict(name="V0 as adopted: 2 pairs z -160/-220, 8 mm apart", bolt_z=(-160.0, -220.0), dy=4.0, tab=False),
            dict(name="V1 wider pairs: 2 pairs, 12 mm apart (y -115/-103)", bolt_z=(-160.0, -220.0), dy=6.0, tab=True),
            dict(name="V2 wider spacing in z: 2 pairs at -140/-240 (100 mm)", bolt_z=(-140.0, -240.0), dy=4.0, tab=True),
            dict(name="V3 third pair: 3 pairs at -160/-200/-240", bolt_z=(-160.0, -200.0, -240.0), dy=4.0, tab=True),
            dict(name="V4 third pair, wide: 3 pairs at -140/-200/-260 (120 mm)", bolt_z=(-140.0, -200.0, -260.0), dy=4.0, tab=True),
            dict(name="V5 third pair + 12 mm pairs: -160/-200/-240, dy 6", bolt_z=(-160.0, -200.0, -240.0), dy=6.0, tab=True),
            dict(name="V6 wide z + 12 mm pairs: -140/-240, dy 6", bolt_z=(-140.0, -240.0), dy=6.0, tab=True)]
def statics_port():
    """harness_loads.py 3-contact-group sagittal statics with the MJ11 mass lumps (top F1+F4, front F2+F3, rail triangular)."""
    global EX
    if EX is None: EX = rig_extras()
    sites = M.SITES; area = lambda n: float(sites[n]["area_mm2"])
    def contact(n):
        s_ = sites[n]; p = np.array(s_["p"]); c = s_.get("contour")
        if c and "u" in s_: nn = np.array(s_["n"]); u = np.array(s_["u"]); v = np.cross(nn, u); cu, cv = c["centre_uv"]; p = p + cu * u + cv * v
        return p
    top = ["F1_upper_trap", "F4_acromion"]; front = ["F2_ant_delt", "F3_pec"]
    cen = lambda gr: (sum(area(n) * contact(n) for n in gr) / sum(area(n) for n in gr), sum(area(n) for n in gr)); (pt, At), (pf, Af) = cen(top), cen(front)
    z0, z1 = -80.0, -440.0; Lr = z0 - z1; pr = np.array([-112.0, -135.0, z0 - 2.0 / 3.0 * Lr]); Ar = 38.0 * Lr / 2.0
    out = {}
    # old pose (S1, S2) -> arm direction -> (th1, phi):  d = (-sin S2 cos S1, sin S2 sin S1, cos S2);  MJ11 dirJ1 = (-sin phi, -sin th cos phi, cos th cos phi)
    OLD = {"park (S1 45, S2 25)": (45, 25), "arm up (S2 0)": (45, 0), "forward horizontal 0.5 kg (S1 15, S2 88)": (15, 88), "lateral horizontal 0.5 kg (S1 90, S2 88)": (90, 88), "back-lateral (S1 140, S2 60)": (140, 60), "back lean (S1 45, S2 -30)": (45, -30)}
    box = dict(th=(-95.0, 5.0), ph=(-15.0, 17.4))
    for nm, (s1, s2) in OLD.items():
        a, b = math.radians(s1), math.radians(s2); d = np.array([-math.sin(b) * math.cos(a), math.sin(b) * math.sin(a), math.cos(b)]); ph = math.degrees(math.asin(-d[0])); th = math.degrees(math.atan2(-d[1], d[2]))
        out[nm] = dict(th1=round(th, 1), phi=round(ph, 1), in_MJ11_box=bool(box["th"][0] <= th <= box["th"][1] and box["ph"][0] <= ph <= box["ph"][1]))
    for nm, (th, ph) in {"MJ11 park": (P.PARK["S1p"], P.PARK["S2"]), "MJ11 reach th1 -90": (-90.0, 0.0), "MJ11 reach th1 -55": (-55.0, 0.0), "MJ11 arm up th1 0": (0.0, 0.0)}.items(): out[nm] = dict(th1=th, phi=ph, in_MJ11_box=True)
    for nm, v in out.items():
        pay = 0.5 if "0.5" in nm or "MJ11" in nm else 0.0
        ms = [(m, p) for m, p in M.arm_masses(v["th1"], v["phi"], pay)] + [(MOUNT_G * 1e-3, np.array([XB, Y1, Z1])), (EX["rail_strip_g"] * 1e-3, np.array(EX["strip_cg"])), (EX["clasp_g"] * 1e-3, np.array(EX["clasp_cg"])),
                                                                          (EX["bridge_half_g"] * 1e-3, np.array([-100.0, -150.0, -180.0])), (EX["pods_g"] * 1e-3, np.array([-100.0, -140.0, -250.0]))]
        Mt = sum(m for m, _ in ms); W = Mt * G_ * 1.3; xc = sum(m * p[0] for m, p in ms) / Mt
        A = np.array([[1, 0, 0], [0, -1, 1], [-pt[0], -pf[2], pr[2]]], float); b = np.array([W, 0.0, -(xc * W)]); Nt, Nf, Nr = np.linalg.solve(A, b)
        v.update(mass_kg=round(Mt, 2), W_dyn_N=round(W, 1), cg_x_mm=round(xc, 1), N_top=round(Nt, 1), N_front=round(Nf, 1), N_rail=round(Nr, 1), p_top_kPa=round(Nt / At * 1e3, 1), p_front_kPa=round(Nf / Af * 1e3, 1), p_rail_peak_kPa=round(Nr / Ar * 1e3, 1))
    return dict(poses=out, areas_mm2=dict(top=round(At), front=round(Af), rail_equiv=round(Ar)), contact_pts=dict(top=pt.round(1).tolist(), front=pf.round(1).tolist(), rail=pr.tolist()), rig_extras=EX)
def closure_force(p0):
    """x-force the strap must supply per side for a uniform pre-pressure p0 (kPa) on rail, liner, top and front pads: sum(back pads, +x) - sum(front pads, x-component of the pad normals)."""
    sites = M.SITES; back = (14440.0 + 10240.0) * p0 * 1e-3; fx = 0.0
    for k in ("F1_upper_trap", "F4_acromion", "F2_ant_delt", "F3_pec"): fx += abs(sites[k]["n"][0]) * sites[k]["area_mm2"] * p0 * 1e-3
    return back - fx
def main():
    import time; t0 = time.time(); R = dict(date="2026-11-03", pad=[Y1, Z1, 12.0], mount_g=MOUNT_G, dyn=1.3, bolt_dyn=2.0, insert_pullout_N=INSERT_PULL, limits=LIM); cfg = BASE_CFG
    # --- 1. baseline reproduction (mj_eval run_frame, dyn 1) + rig extras + dyn 1.3
    R["baseline"] = {}
    for lc in ("park (R 0.5 kg)", "reach th1 -90 (R 0.5 kg, L park)"):
        for rig in (False, True):
            fr, pads = loaded(cfg, lc, rig=rig); u = fr.solve(); R["baseline"][f"{lc} | rig_extras={rig}"] = report(fr, pads, u, 1.0)
    # --- 2. friction-capacity sweep -> minimum uniform pre-pressure p0 per load case and mu
    R["friction_sweep"] = {}; R["p0_required"] = {}
    for lc in LCS:
        fr, pads = loaded(cfg, lc); rows = []; req = {}
        u = fr.solve(); stick = report(fr, pads, u, 1.3); R["friction_sweep"].setdefault(lc, {})["stick_dyn1.3"] = stick["contact"]
        for mu in (0.2, 0.4, 0.6):
            for p0 in (0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0):
                s, uu, tb, D, Cap = cap_solve(fr, mu, p0); r = summarise(fr, pads, uu, tb, p0); r.update(mu=mu, p0=p0, shear_scale=round(s, 3), demand_N=round(D, 1), capacity_N=round(Cap, 1)); rows.append(r)
            ok = [r for r in rows if r["mu"] == mu and not r["lift_off"] and r["shear_scale"] >= 0.999 and max(r["top"].values()) <= LIM["sustained"] and r["rail_max"] <= LIM["sustained"]]
            req[mu] = ok[0]["p0"] if ok else None
            ok2 = [r for r in rows if r["mu"] == mu and not r["lift_off"] and max(r["top"].values()) <= LIM["sustained"] and r["rail_max"] <= LIM["sustained"]]
            req[f"{mu}_slip_allowed"] = ok2[0]["p0"] if ok2 else None
        R["friction_sweep"][lc]["rows"] = rows; R["p0_required"][lc] = req; print(lc, req, round(time.time() - t0), flush=True)
    # lift-off-only minimum (stick solution, dyn 1.3): p0 >= -min delta p on rail, liner, front
    liftoff = {}
    for lc in LCS:
        fr, pads = loaded(cfg, lc); u = fr.solve(); c = report(fr, pads, u, 1.3)["contact"]; liftoff[lc] = dict(rail=-c["rail"]["min_kPa"], liner=-c["liner"]["min_kPa"], front=-c["front"]["min_kPa"])
    R["liftoff_only_p0_kPa"] = liftoff
    worst_p0 = max(max(v.values()) for v in liftoff.values()); R["closure_force_N_per_side"] = {f"p0={p}": round(closure_force(p), 1) for p in (3.0, 4.0, 5.0, 6.0, round(worst_p0, 1))}
    # --- 3. bolts: variants, dyn 2, normal and stall torque
    R["bolt_variants"] = {}
    for v in VARIANTS:
        row = {}
        for lc in ("park (R 0.5 kg)", "reach th1 -90 (R 0.5 kg, L park)", "both reach th1 -90 (0.5 kg each)"):
            fr, pads = loaded(v, lc); u = fr.solve(); rep = report(fr, pads, u, 2.0); b = rep["bolts"]; row[lc] = dict(tension_x2_N=max(x["tension_max_N"] for x in b), shear_per_bolt_x2_N=max(x["shear_per_bolt_N"] for x in b), per_pair=[(x["z"], x["tension_max_N"]) for x in b], rail_unload_kPa_x1=report(fr, pads, u, 1.0)["contact"]["rail"]["min_kPa"])
            row[lc]["insert_SF"] = round(INSERT_PULL / max(row[lc]["tension_x2_N"], 1e-9), 2)
        # stall: arm blocked, motor at cap torque about x on the pad on top of the reach-LC gravity (worst sign)
        for lab, tq in (("stall S1' cap 8.3", 8.3), ("stall S1' cap 8.0", 8.0), ("stall S1' AK70 peak 24.8", 24.8)):
            worst = 0.0
            for sg in (1, -1):
                fr, pads = loaded(v, "reach th1 -90 (R 0.5 kg, L park)"); fr.force(pads["R"], [0, 0, 0], [sg * tq * 1e3, 0, 0]); u = fr.solve(); b = report(fr, pads, u, 1.0)["bolts"]; worst = max(worst, max(x["tension_max_N"] for x in b))
            row[lab] = dict(tension_N=worst, insert_SF=round(INSERT_PULL / worst, 2))
        row["name"] = v["name"]; row["needs_pad_tab"] = v["tab"]; R["bolt_variants"][v["name"]] = row; print(v["name"], {k: (x["tension_x2_N"] if "tension_x2_N" in x else x.get("tension_N")) for k, x in row.items() if isinstance(x, dict)}, round(time.time() - t0), flush=True)
    # --- 4. harness_loads statics port
    R["statics_port"] = statics_port()
    # --- 5. PA-CF closed forms at the bolt zone and dish (reach LC, worst of the V0 bolt result x2)
    F = R["bolt_variants"][VARIANTS[0]["name"]]["reach th1 -90 (R 0.5 kg, L park)"]["tension_x2_N"]; Vs = R["bolt_variants"][VARIANTS[0]["name"]]["reach th1 -90 (R 0.5 kg, L park)"]["shear_per_bolt_x2_N"]
    t_pad, d_hole, d_wash = 6.0, 4.5, 12.0
    chord = 2 * math.sqrt(66.0 ** 2 - 40.0 ** 2); a_ear = 17.0            # ear section at the dish edge (y -100): chord 105 mm, lever to the outer bolt 17 mm
    sig_ear = 6 * F * a_ear / (chord * t_pad ** 2); sig_bear = Vs / (d_hole * t_pad); sig_head = F / (math.pi / 4 * (7.0 ** 2 - d_hole ** 2)); sig_wash = F / (math.pi / 4 * (d_wash ** 2 - d_hole ** 2))
    R["pacf_stress"] = dict(inputs=dict(F_bolt_x2_N=F, shear_per_bolt_x2_N=Vs, pad_t_mm=t_pad, ear_chord_mm=round(chord, 1), ear_lever_mm=a_ear), ear_bending_MPa=round(sig_ear, 1), hole_bearing_MPa=round(sig_bear, 1), head_bearing_no_washer_MPa=round(sig_head, 1), head_bearing_with_M4_washer_OD12_MPa=round(sig_wash, 1),
                            allowable_note="PA-CF printed flat: in-plane tensile ~70-90 MPa dry (vendor-class, UNVERIFIED), use 0.5 for humidity/fatigue = 35-45 MPa; layers lie in the pad plane so the ear bends in the strong direction; compressive crush under a bolt head ~100 MPa (UNVERIFIED), preload 1.5 kN max",
                            dish=dict(liner_peak_kPa_dyn1p3_plus_p0_6=round(1.3 * 2.21 + 6.0, 1), dish_plate_stress_MPa="<0.1 (foam pressure <= 9 kPa over a 16 mm cell lattice on a 3.1 mm+ PA-CF shell: p (a/t)^2 /2 ~ 0.009 * 26 / 2 = 0.12 MPa)", dish_note="the dish is not the weak path; the pad ear + insert are"))
    json.dump(R, open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "loads.json"), "w"), indent=1, default=float); print("done", round(time.time() - t0)); return R
if __name__ == "__main__": main()
