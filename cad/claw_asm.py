"""Pose the C3 claw hand for a gripper opening G (%) or a proximal angle theta1 (deg). Session 2026-10-13g.
G = 100 -> open (theta1 = TH1_O -8), 0 -> closed (theta1 = TH1_C +15.5); hard ends theta1 -13 / +20.5 via pose_th1().  Returns (meshes, info) in the hand (= W3) frame.
Servo angle psi (rad, about +z, CCW) = (ring_z - ring_z_hard_open) / K_HELIX ; ring_z = nut_z(theta1).  Rotor turns by psi; ring translates; fingers curl."""
import os, sys, json, math, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as rot, translation_matrix as tr
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import claw_synth as C
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "distal", "claw")
LAY = json.load(open(os.path.join(OUT, "claw.json")))["_layout"]
D = LAY["D"]; RB, ZB, L1 = LAY["RB"], LAY["ZB"], LAY["L1"]; DP, NP, LD, RL = LAY["DP"], LAY["NP"], LAY["LD"], LAY["RL"]; QA, QB, LC, RY, RN = LAY["QA"], LAY["QB"], LAY["LC"], LAY["RY"], LAY["RN"]
TH_O, TH_C = LAY["TH1_O"], LAY["TH1_C"]; TH_HARD = LAY["TH1_HARD"]; AZ = LAY["AZ"]; K = LAY["K_HELIX"]; ZN = LAY["ZN"]; TAB_PTS = LAY["TAB_PTS"]
SY = json.load(open(os.path.join(HERE, "claw_synth.json"))); PFB = (SY["qa"], SY["qb"], SY["lc"], SY["ry"], SY["rn"])
def fm(a, b): return (RB - a, ZB + b)
Bx, Bz = fm(0, 0); Jx, Jz = fm(0, L1); Px, Pz = fm(-NP, DP); Rx, Rz = fm(RN, L1 + RY); Qx, Qz = fm(QA, QB)
_cache = {}
def L(name, sub=""):
    key = (name, sub)
    if key not in _cache: _cache[key] = trimesh.load(os.path.join(OUT, sub, name + ".stl"))
    return _cache[key]
def G_to_th1(G): return TH_O + (TH_C - TH_O) * (1.0 - G / 100.0)
def th1_to_G(th): return 100.0 * (1.0 - (th - TH_O) / (TH_C - TH_O))
_t2 = {}
def th2_of(th1_deg):
    """distal angle (deg) from the four-bar, marched from the hard-open end for branch continuity."""
    key = round(th1_deg, 4)
    if key in _t2: return _t2[key]
    ths = np.radians(np.linspace(TH_HARD[0], th1_deg, max(2, int(abs(th1_deg - TH_HARD[0]) / 0.5) + 2)))
    p = math.radians(C.TH2_O + (C.TH2_C - C.TH2_O) * (TH_HARD[0] - TH_O) / (TH_C - TH_O))
    for t in ths:
        v = C.pick(t, PFB, p); p = v if v is not None else p
    _t2[key] = math.degrees(p); return _t2[key]
def Ry(alpha_deg, pt_xz):                       # rotation about +y through (x, D, z)
    pt = np.array([pt_xz[0], D, pt_xz[1]]); return tr(pt) @ rot(math.radians(alpha_deg), [0, 1, 0]) @ tr(-pt)
def RotAz(az_deg):
    pt = np.array([0.0, D, 0.0]); return tr(pt) @ rot(math.radians(az_deg), [0, 0, 1]) @ tr(-pt)
def T(mesh, M): m = mesh.copy(); m.apply_transform(M); return m
def link_M(e1, e2):
    """transform taking a canonical bar (pin centres (0,0) and (len,0) in x-z, y absolute) onto the segment e1->e2 (x,z)."""
    d = np.array(e2) - np.array(e1); beta = math.degrees(math.atan2(d[1], d[0]))
    return tr([e1[0], 0.0, e1[1]]) @ rot(math.radians(-beta), [0, 1, 0])
def apply_pt(M, x, z): return (M @ np.array([x, D, z, 1.0]))[[0, 2]]
def pose_th1(th1_deg, with_static=True):
    t1 = th1_deg; t2 = th2_of(t1); zn = C.nut_z(t1)[0]; psi = (zn - ZN["hard_open"]) / K; out = {}; info = {}
    if with_static:
        out["hand_tray"] = L("hand_tray")
        for k in ("sts_g", "horn_disc"): out[k] = L(k, "proxies")                       # sts_g includes the 2 +y mounting ears (PLACEHOLDER pattern, rev 2026-10-15)
        for f in sorted(os.listdir(os.path.join(OUT, "proxies"))):                         # fastener proxies (already in the hand frame): scr_tab*, scr_ear*, scr_cap*
            if f.startswith("scr_"): out[f[:-4]] = L(f[:-4], "proxies")
        for i, az in enumerate(LAY["EAR_AZ"]): out[f"stop_cap{i}"] = T(L("stop_cap"), RotAz(az))
        tab = L("servo_tab")
        for i, (tx, ty) in enumerate(TAB_PTS):
            M = tr([tx, ty + D if False else ty, 0.0]) @ (np.diag([-1.0, 1, 1, 1]) if tx < 0 else np.eye(4))
            m = tab.copy(); m.apply_transform(M)
            if m.volume < 0: m.invert()
            out[f"servo_tab{i}"] = m
    out["claw_rotor"] = T(L("claw_rotor"), tr([0, D, 0]) @ rot(psi, [0, 0, 1]) @ tr([0, -D, 0]))
    Mr = tr([0, 0, zn - ZN["open"]]); out["claw_ring"] = T(L("claw_ring"), Mr)
    for k in range(3): out[f"pin_ring{k}"] = T(L(f"pin_ring{k}", "proxies"), Mr)
    for k, az in enumerate(AZ):
        Mz = RotAz(az); Mp = Ry(-t1, (Bx, Bz)); Md = Mp @ Ry(-t2, (Jx, Jz))
        out[f"prox_{k}"] = T(L("prox_F"), Mz @ Mp); out[f"dist_{k}"] = T(L("dist_F"), Mz @ Md); out[f"tpu_pad_{k}"] = T(L("tpu_pad_F"), Mz @ Md)
        Pg = apply_pt(Mp, Px, Pz); lug = np.array([RL, zn]); Rg = apply_pt(Md, Rx, Rz); Qg = np.array([Qx, Qz])
        out[f"link_drive_{k}"] = T(L("link_drive_F"), Mz @ link_M(Pg, lug)); out[f"link_couple_{k}"] = T(L("link_couple_F"), Mz @ link_M(Qg, Rg))
        out[f"pin_pivot_{k}"] = T(L("pin_pivot_F", "proxies"), Mz); out[f"pin_Q_{k}"] = T(L("pin_Q_F", "proxies"), Mz)
        out[f"pin_J_{k}"] = T(L("pin_J_F", "proxies"), Mz @ Mp); out[f"pin_R_{k}"] = T(L("pin_R_F", "proxies"), Mz @ Md)
        out[f"pin_P_{k}"] = T(L("pin_P_F", "proxies"), Mz @ Mp)
        out[f"pin_lug_{k}"] = T(L("pin_lug_F", "proxies"), Mz @ tr([0, 0, zn - ZN["open"]]))
        tipref = np.array([Jx, D, Jz + 0.0])
        info[k] = dict(th1=t1, th2=t2, zn=zn, psi_deg=math.degrees(psi), len_drive=float(np.linalg.norm(np.array(Pg) - lug)), len_couple=float(np.linalg.norm(np.array(Rg) - Qg)),
                       tip=(Mz @ Md @ np.array([Jx - 0.0, D, Jz + 14.5, 1.0]))[:3].tolist())
    return out, info
def pose(G, with_static=True): return pose_th1(G_to_th1(G), with_static)
if __name__ == "__main__":
    for G in (100, 50, 0):
        m, info = pose(G); i0 = info[0]
        print(G, "th1 %.1f th2 %.1f ring z %.2f psi %.1f deg | link lens drive %.3f (want %.3f) couple %.3f (want %.3f)" % (i0["th1"], i0["th2"], i0["zn"], i0["psi_deg"], i0["len_drive"], LD, i0["len_couple"], LC))
