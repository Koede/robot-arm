"""MJ11 harness / body load checks (2026-11-03).  cad/harness_loads.py, body_loads.py, sensor_pods.py, electronics_fit.py are NOT in the repo, so their load cases are ported onto the MJ11 frame model
(mj_frame.py + mj_concepts3.mj11_frame): linear 3-D frame, rail + spine bridge + clasp on unilateral-about-preload foam springs, liner springs normal to the back, bolt pairs pad -> rail.
Reports (N, mm, kPa, N.m):
  A  frame load cases at the baseline bolt layout (park, reach, pose scan over the free region)
  B  stall cases (S1' / S2 / S3 at the 8.3 / 8.3 / 4.8 N.m caps and the 8.0 proposal): pad pressures, bolt loads
  C  strap preload: unit strap pair (cradle -> F3) -> pressure per N on each contact
  E  bolt-layout options -> bolt tension (frame), shear (rigid group), vs the 1 kN UNVERIFIED insert pull-out
usage: J1R2_ROOT=<work root> python3 mj11_loads.py   -> stl/j1r2_mounts/MJ11/loads.json"""
import os, sys, json, math, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import clib as L, j1r2_params as P, mj_frame as M, mj_concepts as C, mj_concepts3 as C3
G_ = 9.81
# ------------------------------------------------------------------ frame with a parametrised bolt layout
def frame_v(fr, side, bolt_z, bolt_y, half_sy):
    """mj11_frame with the pad->rail bolt layout as parameters: bolt_z rail nodes, bolt_y mean y of the pair, half_sy half spacing of the 2 bolts of a pair in y."""
    mp = (lambda p: np.asarray(p, float)) if side == "R" else M.mirror; pad = C.pad_nodes(fr, side)
    for z in bolt_z:
        nb = f"{side}_pb{int(z)}"; fr.node(nb, mp(np.array([C3.X0, bolt_y, z]))); fr.rigid(pad, nb, tag="pad")
        C.link_bolts(fr, nb, C.rail_node_name(side, z), [(-half_sy, 0), (half_sy, 0)], tag="iface")
    pts = []; step = 16.0
    for y in np.arange(C3.Y1 - C3.PAD_R + 8, C3.Y1 + C3.PAD_R - 7, step):
        for z in np.arange(C3.Z1 - C3.PAD_R + 8, C3.Z1 + C3.PAD_R - 7, step):
            if math.hypot(y - C3.Y1, z - C3.Z1) <= C3.PAD_R - 4 and y >= C3.RAIL_EDGE_Y + 2: pts.append((y, z))
    k_a = M.E_FOAM / C3.LINER_T; A_i = step * step
    for i, (y, z) in enumerate(pts):
        xs = float(M.back_x(y, z)); n = f"{side}_ln{i}"; fr.node(n, mp(np.array([xs + C3.LINER_T / 2, y, z]))); fr.rigid(pad, n, tag="pad"); fr.spring(n, [1.0, 0, 0], k_a * A_i, area=A_i, label=f"{side}_liner{i}", kind="liner")
    an = f"{side}_clasp_anchor"; fr.node(an, mp(C.pad_pt(C3.CLASP_PSI, C.PAD_R - 8, x=M.XB - 3.0))); fr.rigid(pad, an, tag="pad"); C.clasp_bracket_frame(fr, side, an); return pad
LAYOUTS = {   # rail nodes every 10 mm in this study (layouts need z off the 20 mm grid); y spread > 4 needs a local rail ear (rail edge y -100, pad edge y -121)
    "L0 baseline: 2 pairs z -160/-220, y -113/-105 (4 bolts)":                  dict(bz=(-160.0, -220.0), by=-109.0, sy=4.0),
    "L1 z spread 80 mm: 2 pairs z -150/-230, y -113/-105 (4 bolts)":            dict(bz=(-150.0, -230.0), by=-109.0, sy=4.0),
    "L2 3rd pair: z -160/-190/-220, y -113/-105 (6 bolts)":                     dict(bz=(-160.0, -190.0, -220.0), by=-109.0, sy=4.0),
    "L3 3 pairs z -150/-190/-230, y -113/-105 (6 bolts)":                       dict(bz=(-150.0, -190.0, -230.0), by=-109.0, sy=4.0),
    "L4 y spread 12 mm (y -114.5/-102.5) + rail ear, z -160/-220 (4 bolts)":    dict(bz=(-160.0, -220.0), by=-108.5, sy=6.0),
    "L5 y 12 mm + rail ear, z -150/-230 (4 bolts)":                             dict(bz=(-150.0, -230.0), by=-108.5, sy=6.0),
    "L6 3 pairs z -150/-190/-230, y 12 mm + rail ear (6 bolts)":                dict(bz=(-150.0, -190.0, -230.0), by=-108.5, sy=6.0),
}
BASE = list(LAYOUTS)[0]
def solve_lc(layout, th, phi, pay, dyn_arm=1.0, left=(None, None, 0.0), stall=None, strap_N=0.0, hub_pull_N=0.0, dz=20.0):
    """one load case.  right arm th, phi, pay.  stall: dict(S1, S2, S3) N.m reaction moments on the pad (on top of gravity).  strap_N: strap pair cradle<->F3 on both sides."""
    lay = LAYOUTS[layout] if isinstance(layout, str) else layout
    M.RAIL["dz"] = dz; fr = M.Frame(); M.harness(fr, "R"); M.harness(fr, "L"); M.spine_bridge(fr)
    pads = {s: frame_v(fr, s, lay["bz"], lay["by"], lay["sy"]) for s in ("R", "L")}
    lth, lph, lpay = left; lth = P.PARK["S1p"] if lth is None else lth; lph = P.PARK["S2"] if lph is None else lph
    for s in ("R", "L"):
        a = (th, phi, pay) if s == "R" else (lth, lph, lpay)
        M.apply_arm(fr, pads[s], s, a[0], a[1], a[2])
        if dyn_arm != 1.0:
            pn = fr.nodes[fr.idx[pads[s]]]
            for m, r in M.arm_masses(a[0], a[1], a[2])[:2]:
                r = r if s == "R" else M.mirror(r); f = np.array([0, 0, -m * G_ * (dyn_arm - 1.0)]); fr.force(pads[s], f, np.cross(r - pn, f))
    if stall:
        F0, FA, s_, a_, d_ = P.shoulder_frames(M.DER, th, phi)
        Mv = (stall.get("S1", 0.0) * np.array([1.0, 0, 0]) + stall.get("S2", 0.0) * s_ + stall.get("S3", 0.0) * d_) * 1e3
        fr.force("R_pad", np.zeros(3), Mv)
    if strap_N:
        for s in ("R", "L"):
            a_, b_ = fr.nodes[fr.idx[f"{s}_axilla"]], fr.nodes[fr.idx[f"{s}_F3"]]; u_ = (b_ - a_) / np.linalg.norm(b_ - a_)
            fr.force(f"{s}_axilla", strap_N * u_); fr.force(f"{s}_F3", -strap_N * u_)
    if hub_pull_N:
        for s_ in ("R", "L"): fr.force(f"{s_}_clasp_hub", np.array([-hub_pull_N, 0, 0]))
    u = fr.solve(); sp = fr.spring_forces(); res = {}; tip = M.hand_tip(th, phi); i = fr.idx["R_pad"]; pn = fr.nodes[i]
    res["tip_mm"] = float(np.linalg.norm(u[i, :3] + np.cross(u[i, 3:], tip - pn)))
    for kind in ("rail", "top", "front", "axilla", "liner"):
        ps = [s_ for s_ in sp if s_["kind"] == kind and s_["label"].startswith("R_")]
        if ps: res[kind] = dict(max=float(max(s_["p_kPa"] for s_ in ps)), min=float(min(s_["p_kPa"] for s_ in ps)), sumN=float(sum(s_["F"] for s_ in ps)))
    # vertical load carried by the right rail foam shear (frame) -> the rest goes to the clasp top pads
    rail_idx = {fr.idx[k_] for k_ in fr.idx if k_.startswith("R_rail")}; Fz_rail = 0.0
    for spg in fr.springs:
        if spg["kind"] == "shear" and spg["n"] in rail_idx and abs(spg["d"][2]) > 0.99: Fz_rail += -spg["k"] * float(u[spg["n"], 2])
    res["rail_shear_Fz_N"] = Fz_rail
    Fsum = np.zeros(3); Msum = np.zeros(3); cen = np.array([C3.X0, lay["by"], np.mean(lay["bz"])])
    for b in fr.beams:
        if b["tag"] != "iface" or fr.nodes[b["a"]][1] < M.SPINE_Y: continue
        R_, Lb, kg = b["_RL"]; ue = np.r_[u[b["a"]], u[b["b"]]]; fg = kg @ ue; Fa, Ma = fg[0:3], fg[3:6]; ra = fr.nodes[b["a"]]; Fsum += Fa; Msum += Ma + np.cross(ra - cen, Fa)
    pos = np.array([[lay["by"] + sg * lay["sy"], z] for z in lay["bz"] for sg in (-1, 1)]); n = len(pos); c = pos.mean(0); d_ = pos - c
    Sy, Sz = float((d_[:, 0] ** 2).sum()) or 1e9, float((d_[:, 1] ** 2).sum()) or 1e9; r2 = float((d_ ** 2).sum())
    ten = np.abs(Fsum[0]) / n + np.abs(Msum[1]) * np.abs(d_[:, 1]) / Sz + np.abs(Msum[2]) * np.abs(d_[:, 0]) / Sy
    sh = []
    for dd in d_:
        nn = np.linalg.norm(dd) or 1.0; t = np.array([-dd[1], dd[0]]) / nn; fv = Fsum[1:3] / n + Msum[0] * t * nn / r2; sh.append(float(np.linalg.norm(fv)))
    res["bolts"] = dict(n=n, tension_max_N=float(ten.max()), shear_max_N=float(max(sh)), Fx_N=float(Fsum[0]), Fy_N=float(Fsum[1]), Fz_N=float(Fsum[2]), Mx_Nmm=float(Msum[0]), My_Nmm=float(Msum[1]), Mz_Nmm=float(Msum[2]))
    pp = []                                          # per-pair method of mj_eval.run_frame (reproduces the 231 / 312 N of the MJ11 spec): N/n + My v/sum v2 - Mz u/sum u2 at the bolt-group beam end a
    for b in fr.beams:
        if b["tag"] != "iface" or fr.nodes[b["a"]][1] < M.SPINE_Y: continue
        R_, Lb, kg = b["_RL"]; ue = np.r_[u[b["a"]], u[b["b"]]]; f = np.kron(np.eye(4), R_) @ (kg @ ue); N_, My_, Mz_ = f[0], f[4], f[5]; bo = np.asarray(b["bolts"], float); nb_ = len(bo)
        su = float((bo[:, 0] ** 2).sum()) or 1e9; sv = float((bo[:, 1] ** 2).sum()) or 1e9; Fi = N_ / nb_ + My_ * bo[:, 1] / sv - Mz_ * bo[:, 0] / su; pp.append(float(np.abs(Fi).max()))
    res["bolt_pair_tension_N"] = max(pp) if pp else 0.0; res["bolt_pair_each_N"] = pp
    stress = {}
    for b in fr.beams:
        if b["tag"] in ("pad", "rigid", "iface") or b["E"] >= M.MAT["STEEL"][0] or fr.nodes[b["a"]][1] < M.SPINE_Y - 1: continue
        R_, Lb, kg = b["_RL"]; ue = np.r_[u[b["a"]], u[b["b"]]]; f = np.kron(np.eye(4), R_) @ (kg @ ue)
        for e0 in (0, 6):
            N, My, Mz = f[e0], f[e0 + 4], f[e0 + 5]; sig = abs(N) / b["A"] + abs(My) * b.get("cz", 5) / b["Iy"] + abs(Mz) * b.get("cy", 5) / b["Iz"]; stress[b["tag"]] = max(stress.get(b["tag"], 0), sig)
    res["member_stress_MPa"] = {k: round(v, 2) for k, v in stress.items()}
    return res
def rnd(o, n=2):
    if isinstance(o, dict): return {k: rnd(v, n) for k, v in o.items()}
    if isinstance(o, (list, tuple)): return [rnd(v, n) for v in o]
    return round(o, n) if isinstance(o, float) else o
if __name__ == "__main__":
    out = {}; park = (P.PARK["S1p"], P.PARK["S2"], 0.5); reach = (-90.0, 0.0, 0.5); D = os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11")
    A = {"park": solve_lc(BASE, *park), "reach": solve_lc(BASE, *reach)}; print("A pair tension", A["park"]["bolt_pair_tension_N"], A["reach"]["bolt_pair_tension_N"], A["park"]["bolts"], A["reach"]["bolts"], A["park"]["rail"], A["reach"]["rail"], flush=True)
    scan = []
    for th in (-95, -80, -65, -50, -35, -20, -5, 5):
        for phi in (-15, 0, 17.4, 35):
            r = solve_lc(BASE, th, phi, 0.5); scan.append(dict(th=th, phi=phi, rail_min=r["rail"]["min"], rail_max=r["rail"]["max"], liner_min=r["liner"]["min"], liner_max=r["liner"]["max"], front_min=r["front"]["min"], top_max=r["top"]["max"], top_sumN=r["top"]["sumN"], bolt=r["bolt_pair_tension_N"], shear=r["bolts"]["shear_max_N"], Fz_rail=r["rail_shear_Fz_N"]))
    A["scan"] = scan; wb = max(scan, key=lambda s: s["bolt"])
    A["scan_worst"] = dict(rail_min=min(s["rail_min"] for s in scan), rail_max=max(s["rail_max"] for s in scan), liner_min=min(s["liner_min"] for s in scan), liner_max=max(s["liner_max"] for s in scan), front_min=min(s["front_min"] for s in scan),
                           top_max=max(s["top_max"] for s in scan), bolt_tension=wb["bolt"], bolt_shear=max(s["shear"] for s in scan), at_bolt=[wb["th"], wb["phi"]])
    out["A_frame"] = A; print("A scan worst", A["scan_worst"], flush=True)
    Bd = {}
    for lab, caps in (("cap 8.3/8.3/4.8", dict(S1=8.3, S2=8.3, S3=4.8)), ("cap 8.0/8.0/4.8 (proposal)", dict(S1=8.0, S2=8.0, S3=4.8))):
        w = dict(p_rail=0, p_liner=0, p_top=0, p_front=0, p_axilla=0, bolt_t=0, bolt_s=0, rail_min=0, liner_min=0)
        for pose in (park, reach, (-50.0, 17.4, 0.5), (-20.0, 35.0, 0.5)):
            for sg in itertools.product((-1, 1), repeat=3):
                r = solve_lc(BASE, *pose, stall=dict(S1=sg[0] * caps["S1"], S2=sg[1] * caps["S2"], S3=sg[2] * caps["S3"]))
                for k_, kk in (("p_rail", "rail"), ("p_liner", "liner"), ("p_top", "top"), ("p_front", "front"), ("p_axilla", "axilla")):
                    if kk in r and r[kk]["max"] > w[k_]: w[k_] = r[kk]["max"]
                w["rail_min"] = min(w["rail_min"], r["rail"]["min"]); w["liner_min"] = min(w["liner_min"], r["liner"]["min"])
                if r["bolt_pair_tension_N"] > w["bolt_t"]: w["bolt_t"] = r["bolt_pair_tension_N"]; w["case"] = [list(pose), list(sg)]
                w["bolt_s"] = max(w["bolt_s"], r["bolts"]["shear_max_N"])
        Bd[lab] = w; print("B", lab, w, flush=True)
    out["B_stall"] = Bd
    z0 = solve_lc(BASE, park[0], park[1], 0.0); s10 = solve_lc(BASE, park[0], park[1], 0.0, strap_N=10.0); h10 = solve_lc(BASE, park[0], park[1], 0.0, hub_pull_N=10.0); Cs = {}
    for lab_, rr in (("front strap pair cradle<->F3, per N", s10), ("hub pull -x on both clasp hubs, per N per side", h10)):
        Cs[lab_] = {kk: dict(max_per_N=(rr[kk]["max"] - z0[kk]["max"]) / 10.0, min_per_N=(rr[kk]["min"] - z0[kk]["min"]) / 10.0) for kk in ("rail", "top", "front", "axilla", "liner") if kk in rr}
    out["C_strap_per_N_kPa"] = Cs; print("C", Cs, flush=True)
    json.dump(rnd(out, 3), open(os.path.join(D, "loads_partA.json"), "w"), indent=1)
    Ed = {}
    for lab in LAYOUTS:
        row = {}
        for nm, pose in (("park", park), ("reach", reach)):
            r = solve_lc(lab, *pose, dz=10.0); row[nm] = dict(tension=r["bolt_pair_tension_N"], tension_group=r["bolts"]["tension_max_N"], shear=r["bolts"]["shear_max_N"], rail_min=r["rail"]["min"], liner_min=r["liner"]["min"], tip=r["tip_mm"], rail_stress=r["member_stress_MPa"].get("rail"))
        sc = [solve_lc(lab, th, phi, 0.5, dz=10.0) for th in (-95, -70, -45, -20, 5) for phi in (-15, 17.4, 35)]
        row["scan_tension_max"] = max(s["bolt_pair_tension_N"] for s in sc); row["scan_shear_max"] = max(s["bolts"]["shear_max_N"] for s in sc)
        rs = solve_lc(lab, *reach, stall=dict(S1=8.3, S2=8.3, S3=4.8), dz=10.0); row["stall_reach_tension"] = rs["bolt_pair_tension_N"]; row["stall_reach_shear"] = rs["bolts"]["shear_max_N"]
        Ed[lab] = row; print("E", lab, row, flush=True)
    out["E_layouts"] = Ed
    json.dump(rnd(out, 3), open(os.path.join(D, "loads.json"), "w"), indent=1)
