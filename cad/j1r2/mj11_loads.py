"""MJ11 loads re-run (2026-11-03, HARNESS / SHOULDER). harness_loads.py / body_loads.py / sensor_pods.py / electronics_fit.py are NOT in the repo
(DESIGN_STATE 12.7), so their load cases are ported onto the MJ11 frame model (mj_frame.py + mj_concepts3.mj11_frame, linear about the strap preload).
What this adds to mj_eval.run_frame:
  * pose scan over the MJ11 passive box (worst rail unload / liner unload / bolt load), not only the two load cases park / reach;
  * x1 and x2 dynamic (linear model: increments scale, the preload does not), stall (AK70 cap 8.3 N.m about S1' and about S2) and S1' stop-plateau (20 N.m) reactions on the pad;
  * strap preload needed: max of lift-off and skin friction (vertical load carried by rail foam shear / (mu * normal));
  * per-bolt loads: (a) beam-group model (includes prying across the spacers), (b) rigid-plate statics with dowels carrying the shear (axial only);
  * bolt layouts: baseline 2 rows, wider pair spacing, 3rd row, 4 rows, dowel variant; pad edge distances;
  * PA-CF hand checks at the pad bolt zone and the dish (formulas in the SPEC).
usage: J1R2_ROOT=/home/claude/ts python3 mj11_loads.py   -> stl/j1r2_mounts/MJ11/loads.json"""
import os, sys, json, math, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import clib as L, j1r2_params as P, mj_frame as M, mj_concepts as C, mj_concepts3 as C3
OUT = os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11"); os.makedirs(OUT, exist_ok=True)
X0, XB, Y1, Z1 = M.X0, M.XB, M.Y1, M.Z1
MOUNT_G = 187.0                       # MJ11 mount mass (mj_eval) lumped at the pad
MU = 0.5                              # foam-on-base-layer friction, ASSUMPTION (coupon needed)
TOP_LIMIT_KPA = 14.7                  # harness top-pad limit used since rev f2 (SPEC_flex_electrode_leads)
INSERT_PULLOUT_N = 1000.0             # M4 heat-set in PA-CF, typical, UNVERIFIED
TAU_STALL = 8.3e3                     # AK70-10 cap, N.mm (DESIGN_STATE 12.5: 8.3 -> 8.0 pending)
TAU_STOP = 20.0e3                     # S1' bumper plateau, N.mm (SPEC_j1r2_balancer)
RAIL_NODES = {int(z) for z in np.arange(-440, -79, 20)}

def rows_nodes(rows): return [int(z) for z in rows]

def frame_var(fr, side, rows, ys):
    """C3.mj11_frame with a free bolt layout: rows = pad-plane bolt-row heights z (must be rail node heights), ys = bolt y's inside a row."""
    mp = (lambda p: np.asarray(p, float)) if side == "R" else M.mirror; pad = C.pad_nodes(fr, side); ym = float(np.mean(ys))
    for z in rows:
        nb = f"{side}_pb{int(z)}"; fr.node(nb, mp(np.array([X0, ym, z]))); fr.rigid(pad, nb, tag="pad")
        C.link_bolts(fr, nb, C.rail_node_name(side, z), [(y - ym, 0.0) for y in ys], tag="iface")
    pts = []; step = 16.0
    for y in np.arange(Y1 - C.PAD_R + 8, Y1 + C.PAD_R - 7, step):
        for z in np.arange(Z1 - C.PAD_R + 8, Z1 + C.PAD_R - 7, step):
            if math.hypot(y - Y1, z - Z1) <= C.PAD_R - 4 and y >= C3.RAIL_EDGE_Y + 2: pts.append((y, z))
    k_a = M.E_FOAM / C3.LINER_T; A_i = step * step
    for i, (y, z) in enumerate(pts):
        xs = float(M.back_x(y, z)); n = f"{side}_ln{i}"; fr.node(n, mp(np.array([xs + C3.LINER_T / 2, y, z]))); fr.rigid(pad, n, tag="pad")
        fr.spring(n, [1.0, 0, 0], k_a * A_i, area=A_i, label=f"{side}_liner{i}", kind="liner")
    an = f"{side}_clasp_anchor"; fr.node(an, mp(C.pad_pt(C3.CLASP_PSI, C.PAD_R - 8, x=XB - 3.0))); fr.rigid(pad, an, tag="pad"); C.clasp_bracket_frame(fr, side, an); return pad

def build(rows, ys):
    fr = M.Frame(); M.harness(fr, "R"); M.harness(fr, "L"); M.spine_bridge(fr); pads = {s: frame_var(fr, s, rows, ys) for s in ("R", "L")}; return fr, pads

def beam_end_global(fr, u, b):
    """global force/moment at end a of beam b (on the member by the node): returns (F[3], M[3], local f)."""
    R_, Lb, kg = b["_RL"]; ue = np.r_[u[b["a"]], u[b["b"]]]; fg = kg @ ue; return fg[:3], fg[3:6], np.kron(np.eye(4), R_) @ fg, Lb

def solve_lc(fr, pads, th, ph, pay, left=(P.PARK["S1p"], P.PARK["S2"], 0.0), dyn=1.0, pad_moment=None, extra_g=MOUNT_G):
    """pad_moment = (Mx, Ms) extra reaction moments on the right pad (N.mm) about the S1' axis (x) and the S2 axis s(th1)."""
    fr.loads = {}
    for s_ in ("R", "L"):
        a = (th, ph, pay) if s_ == "R" else left; M.apply_arm(fr, pads[s_], s_, a[0], a[1], a[2], extra_g=extra_g)
    if dyn != 1.0:
        for i in fr.loads: fr.loads[i] = fr.loads[i] * dyn
    if pad_moment is not None:
        Mx, Ms = pad_moment; s = P.s_of(th); fr.force("R_pad", (0, 0, 0), M=np.array([Mx, 0.0, 0.0]) + Ms * s)
    return fr.solve()

def contact_summary(fr, u):
    sp = fr.spring_forces(); out = {}
    for kind in ("rail", "top", "front", "axilla", "liner"):
        ps = [s_ for s_ in sp if s_["kind"] == kind and s_["label"].startswith("R_")]
        if ps: out[kind] = dict(max=max(s_["p_kPa"] for s_ in ps), min=min(s_["p_kPa"] for s_ in ps), sum_N=sum(s_["F"] for s_ in ps), neg_sum_N=sum(min(s_["F"], 0.0) for s_ in ps))
    # vertical support carried by shear springs of the right rail (skin friction demand): -k (u.d) for the z-shear springs
    sh = 0.0
    for s_ in fr.springs:
        if s_["kind"] == "shear" and fr.nodes[s_["n"]][1] > M.SPINE_Y and abs(s_["d"][2]) > 0.99: sh += -s_["k"] * float(u[s_["n"], :3] @ s_["d"])
    out["rail_vertical_shear_N"] = sh
    return out

def bolt_loads(fr, u, rows, ys, side="R"):
    """per-bolt demand: (a) beam-group (prying) model, (b) rigid-plate statics with dowels (axial only). returns dict."""
    ym = float(np.mean(ys)); bo = np.array([[y - ym, 0.0] for y in ys])      # local (u = y offset)
    tens_beam = []; shear_row = []; Fw = np.zeros(3); Mw = np.zeros(3); c = np.array([X0, ym, float(np.mean(rows))])
    for b in fr.beams:
        if b["tag"] != "iface" or fr.nodes[b["a"]][1] < M.SPINE_Y: continue
        Fg, Mg, f, Lb = beam_end_global(fr, u, b); N, Vy, Vz, T, My, Mz = f[0], f[1], f[2], f[3], f[4], f[5]
        n_b = len(b["bolts"]); d_ = np.asarray(b["bolts"], float); su = float((d_[:, 0] ** 2).sum()) or 1e9
        Fi = N / n_b - Mz * d_[:, 0] / su + My * d_[:, 1] / (float((d_[:, 1] ** 2).sum()) or 1e9)
        tens_beam.append(float(np.abs(Fi).max())); shear_row.append(float(math.hypot(Vy, Vz)))
        pa = fr.nodes[b["a"]]; Fw += Fg; Mw += Mg + np.cross(pa - c, Fg)                               # wrench of the pad on the joints, about the pattern centroid
    # (b) rigid pad on axial-only bolts, dowels carry all shear: tension T_i = Fx/n - (Mz*(y_i-ym) ...) with the real pattern
    pts = np.array([[y, z] for z in rows for y in ys]); n = len(pts); yc, zc = pts.mean(0); dy, dz = pts[:, 0] - yc, pts[:, 1] - zc
    Syy, Szz = float((dy ** 2).sum()) or 1e9, float((dz ** 2).sum()) or 1e9
    Fx = Fw[0]; Ti = Fx / n + Mw[2] * dy / Syy * -1.0 + Mw[1] * dz / Szz        # about-z moment -> y lever, about-y moment -> z lever (right-hand sign; worst case taken as abs max)
    Tmax_static = float(np.abs(Fx / n) + abs(Mw[2]) * np.abs(dy).max() / Syy + abs(Mw[1]) * np.abs(dz).max() / Szz)
    # in-plane shear per dowel: Fy, Fz, torsion Mx about the pattern centroid
    rr2 = float((dy ** 2 + dz ** 2).sum()) or 1e9; sh = []
    for i in range(n):
        v = np.array([Fw[1] / n, Fw[2] / n]) + Mw[0] * np.array([-dz[i], dy[i]]) / rr2; sh.append(float(np.linalg.norm(v)))
    return dict(beam_tension_max_N=max(tens_beam) if tens_beam else 0.0, beam_row_shear_N=shear_row, dowel_tension_max_N=Tmax_static, dowel_shear_max_N=max(sh), wrench_Fx_Fy_Fz=Fw.round(1).tolist(), wrench_Mx_My_Mz_Nmm=Mw.round(0).tolist())

def edge_report(rows, ys, d_hole=4.5):
    """pad edge distance (centre to R66 rim, to the trimmed flat y -121) and the net ligament."""
    out = []
    for z in rows:
        for y in ys:
            r = math.hypot(y - Y1, z - Z1); e_rim = C.PAD_R - r; e_flat = y - C3.PAD_TRIM_Y; e = min(e_rim, e_flat)
            out.append(dict(y=y, z=z, r_mm=round(r, 1), e_rim=round(e_rim, 1), e_flat=round(e_flat, 1), e_over_d=round(e / 4.0, 2), ligament_mm=round(e - d_hole / 2, 2)))
    return out

LAYOUTS = {
    "L0 baseline (2 rows 60 apart, pair 8 mm)": dict(rows=(-160, -220), ys=(-113.0, -105.0)),
    "L1 edge-corrected pair (y -111/-103)": dict(rows=(-160, -220), ys=(-111.0, -103.0)),
    "L2 +3rd row (z -160/-200/-220), y -111/-103": dict(rows=(-160, -200, -220), ys=(-111.0, -103.0)),
    "L3 4 rows (z -160/-180/-200/-220), y -114/-103": dict(rows=(-160, -180, -200, -220), ys=(-112.0, -103.0)),
    "L4 3 rows near-centre (z -180/-200/-220), y -115/-103": dict(rows=(-180, -200, -220), ys=(-115.0, -103.0)),
}

def lc_table(layout, dyn=1.0):
    rows, ys = layout["rows"], layout["ys"]; fr, pads = build(rows, ys); res = {}
    cases = {"park": (P.PARK["S1p"], P.PARK["S2"], 0.5, None), "reach": (-90.0, 0.0, 0.5, None),
             "reach+stallS1(+)": (-90.0, 0.0, 0.5, (TAU_STALL, 0.0)), "reach+stallS1(-)": (-90.0, 0.0, 0.5, (-TAU_STALL, 0.0)),
             "reach+stallS2(+)": (-90.0, 0.0, 0.5, (0.0, TAU_STALL)), "reach+stallS2(-)": (-90.0, 0.0, 0.5, (0.0, -TAU_STALL)),
             "reach+S1stop(+)": (-90.0, 0.0, 0.5, (TAU_STOP, 0.0)), "reach+S1stop(-)": (-90.0, 0.0, 0.5, (-TAU_STOP, 0.0))}
    for k, (th, ph, pay, pm) in cases.items():
        u = solve_lc(fr, pads, th, ph, pay, dyn=dyn, pad_moment=pm)
        cs = contact_summary(fr, u); bl = bolt_loads(fr, u, rows, ys); i = fr.idx["R_pad"]; pn = fr.nodes[i]; tip = M.hand_tip(th, ph)
        res[k] = dict(contact=cs, bolts=bl, tip_mm=float(np.linalg.norm(u[i, :3] + np.cross(u[i, 3:], tip - pn))))
    return res

def pose_scan(layout, dyn=1.0):
    rows, ys = layout["rows"], layout["ys"]; fr, pads = build(rows, ys); best = dict(neg_rail_N=(0, None), neg_liner_N=(0, None), rail_min=(0, None), liner_min=(0, None), bolt=(0, None), shear=(0, None), liner_max=(0, None), top_max=(0, None), top_min=(1e9, None))
    for th in np.arange(-95.0, 5.1, 10.0):
        for ph in (-15.0, 0.0, 17.4):
            u = solve_lc(fr, pads, th, ph, 0.5, dyn=dyn); cs = contact_summary(fr, u); bl = bolt_loads(fr, u, rows, ys); pose = (float(th), float(ph))
            for key, v in (("neg_rail_N", cs["rail"]["neg_sum_N"]), ("neg_liner_N", cs["liner"]["neg_sum_N"])):
                if v < best[key][0]: best[key] = (v, pose)
            for key, v in (("rail_min", cs["rail"]["min"]), ("liner_min", cs["liner"]["min"])):
                if v < best[key][0]: best[key] = (v, pose)
            if cs["liner"]["max"] > best["liner_max"][0]: best["liner_max"] = (cs["liner"]["max"], pose)
            if cs["top"]["max"] > best["top_max"][0]: best["top_max"] = (cs["top"]["max"], pose)
            if cs["top"]["min"] < best["top_min"][0]: best["top_min"] = (cs["top"]["min"], pose)
            if bl["beam_tension_max_N"] > best["bolt"][0]: best["bolt"] = (bl["beam_tension_max_N"], pose)
            if cs["rail_vertical_shear_N"] > best["shear"][0]: best["shear"] = (cs["rail_vertical_shear_N"], pose)
    return {k: dict(value=round(v[0], 2), pose_th1_phi=v[1]) for k, v in best.items()}

def rail_area_mm2(): return RAIL_A
RAIL_A = M.RAIL["b"] * (M.RAIL["z"][1] - M.RAIL["z"][0] + M.RAIL["dz"])          # 38 x 380 mm of foam-backed rail (right)
def liner_area_mm2():
    pts = 0
    for y in np.arange(Y1 - C.PAD_R + 8, Y1 + C.PAD_R - 7, 16.0):
        for z in np.arange(Z1 - C.PAD_R + 8, Z1 + C.PAD_R - 7, 16.0):
            if math.hypot(y - Y1, z - Z1) <= C.PAD_R - 4 and y >= C3.RAIL_EDGE_Y + 2: pts += 1
    return pts * 256.0

# ------------------------------------------------------------------ PA-CF hand checks (formulas in SPEC_j1r2_mj11_loads_balancer.md)
PACF = dict(UTS_xy=70.0, UTS_z=35.0, creep_derate=0.5, tau_allow=15.0, bearing_allow=17.5)   # MPa, PLACEHOLDER typical PA12-CF values, coupons needed (derate 0.5 for creep / print variation)
def pacf_checks(T_bolt, V_bolt, e_edge, t_pad=6.0, d=4.0, dw=9.0, d_h=4.5, M_pad_Nmm=0.0, lever_mm=57.0, r_boss=44.5, b_eff=78.0):
    A_w = math.pi / 4 * (dw ** 2 - d_h ** 2); sig_pull = T_bolt / A_w                                    # washer bearing, through thickness
    tau_so = V_bolt / (2 * t_pad * max(e_edge - d_h / 2, 0.1))                                            # shear-out, two planes
    sig_br = V_bolt / (d * t_pad)                                                                         # in-plane hole bearing
    V_tot = M_pad_Nmm / lever_mm; Mb = V_tot * max(lever_mm - r_boss, 1.0); Zs = b_eff * t_pad ** 2 / 6.0; sig_pl = Mb / Zs     # pad strip between motor boss and the bolt line
    A_t = 8.78; sig_m4 = math.sqrt((T_bolt / A_t) ** 2 + 3 * (V_bolt / A_t) ** 2)                         # M4 8.8 steel (tensile area 8.78 mm2) von Mises, no preload
    return dict(washer_bearing_MPa=round(sig_pull, 1), shear_out_MPa=round(tau_so, 1), hole_bearing_MPa=round(sig_br, 1), pad_strip_bending_MPa=round(sig_pl, 1), M4_vonMises_MPa=round(sig_m4, 0),
                allow=dict(bearing=PACF["bearing_allow"] , tau=PACF["tau_allow"], bending=PACF["UTS_xy"] * PACF["creep_derate"], M4_yield_8p8=640.0))

def dish_checks(p_kPa, span=30.0, t_wall=2.4):
    """dish shell: simply supported strip between ribs under foam pressure: sigma = 0.75 p (span/t)^2 (plate, nu .3, long strip 6 p L^2/(8 t^2) = 0.75 p (L/t)^2)."""
    p = p_kPa * 1e-3; return dict(p_kPa=p_kPa, span_mm=span, t_mm=t_wall, sigma_MPa=round(0.75 * p * (span / t_wall) ** 2, 2), allow_MPa=PACF["UTS_xy"] * PACF["creep_derate"])

if __name__ == "__main__":
    out = dict(assumptions=dict(MU=MU, top_limit_kPa=TOP_LIMIT_KPA, insert_pullout_N=INSERT_PULLOUT_N, tau_stall_Nm=TAU_STALL / 1e3, tau_stop_Nm=TAU_STOP / 1e3, mount_g=MOUNT_G, PACF=PACF,
                           note="harness_loads/body_loads not in repo: ported to the MJ11 frame model; linear about strap preload; pressures are increments"))
    out["rail_area_mm2"] = RAIL_A; out["liner_area_mm2"] = liner_area_mm2(); out["layouts"] = {}
    for name, lay in LAYOUTS.items():
        d1 = lc_table(lay, 1.0); d2 = lc_table(lay, 2.0); ps = pose_scan(lay); ps2 = pose_scan(lay, 2.0)
        ent = dict(rows=lay["rows"], ys=lay["ys"], edge=edge_report(lay["rows"], lay["ys"]), x1=d1, x2=d2, pose_scan=ps, pose_scan_x2=ps2); out["layouts"][name] = ent
        r = d1["reach"]; print(name, "| reach beam %.0f N / dowel %.0f N | x2 %.0f N | stallS1 %.0f stop %.0f | rail min %.2f liner min %.2f" % (r["bolts"]["beam_tension_max_N"], r["bolts"]["dowel_tension_max_N"], d2["reach"]["bolts"]["beam_tension_max_N"],
              max(d1[k]["bolts"]["beam_tension_max_N"] for k in d1 if "stallS" in k), max(d1[k]["bolts"]["beam_tension_max_N"] for k in d1 if "stop" in k), r["contact"]["rail"]["min"], r["contact"]["liner"]["min"]), flush=True)
    # ---- derived: strap preload, top-pad window, PA-CF checks (baseline L0 and recommended L3)
    L0, L3 = out["layouts"]["L0 baseline (2 rows 60 apart, pair 8 mm)"], out["layouts"]["L3 4 rows (z -160/-180/-200/-220), y -114/-103"]
    A_r, A_l = RAIL_A, out["liner_area_mm2"]; der = {}
    for lab, ps in (("x1", L0["pose_scan"]), ("x2", L0["pose_scan_x2"])):
        sc = 1.0 if lab == "x1" else 2.0; rail_unload = -ps["rail_min"]["value"]; liner_unload = -ps["liner_min"]["value"]; V = ps["shear"]["value"]
        p_fric = V / MU / A_r * 1e3
        der[lab] = dict(rail_unload_kPa=round(rail_unload, 2), liner_unload_kPa=round(liner_unload, 2), rail_vertical_shear_N=round(V, 1), rail_p_for_friction_kPa=round(p_fric, 2),
                        rail_p0_required_kPa=round(max(rail_unload, p_fric), 2), liner_p0_required_kPa=round(liner_unload, 2),
                        F_pre_uniform_N=round(max(rail_unload, p_fric) * A_r * 1e-3 + liner_unload * A_l * 1e-3, 0), F_pre_min_N=round(-(ps["neg_rail_N"]["value"] + ps["neg_liner_N"]["value"]), 1))
        der[lab]["front_pad_p_kPa_for_uniform"] = round(der[lab]["F_pre_uniform_N"] / 5531.0 * 1e3, 1); der[lab]["front_pad_p_kPa_for_min"] = round(der[lab]["F_pre_min_N"] / 5531.0 * 1e3, 1)
        der[lab]["top_increment_kPa"] = ps["top_max"]["value"]; der[lab]["top_preload_budget_kPa_to_14p7"] = round(TOP_LIMIT_KPA - ps["top_max"]["value"], 1); der[lab]["top_preload_budget_kPa_with_2pct_margin"] = round(TOP_LIMIT_KPA * 0.98 - ps["top_max"]["value"], 1)
    out["derived_preload"] = der; out["derived_pacf"] = {}
    for nm, lay_ in (("L0", L0), ("L3", L3)):
        e_min = min(min(e["e_rim"], e["e_flat"]) for e in lay_["edge"]); r2 = lay_["x2"]["reach"]["bolts"]; st = lay_["x1"]["reach+S1stop(-)"]["bolts"]; sS1 = lay_["x1"]["reach+stallS1(-)"]["bolts"]
        out["derived_pacf"][nm] = dict(e_edge_mm=round(e_min, 1), x2_reach=pacf_checks(r2["beam_tension_max_N"], r2["dowel_shear_max_N"], e_min, M_pad_Nmm=abs(r2["wrench_Mx_My_Mz_Nmm"][0])),
                                       stallS1=pacf_checks(sS1["beam_tension_max_N"], sS1["dowel_shear_max_N"], e_min, M_pad_Nmm=TAU_STALL + abs(lay_["x1"]["reach"]["bolts"]["wrench_Mx_My_Mz_Nmm"][0])),
                                       S1stop=pacf_checks(st["beam_tension_max_N"], st["dowel_shear_max_N"], e_min, M_pad_Nmm=TAU_STOP + abs(lay_["x1"]["reach"]["bolts"]["wrench_Mx_My_Mz_Nmm"][0])),
                                       insert_SF=dict(static=round(INSERT_PULLOUT_N / lay_["x1"]["reach"]["bolts"]["beam_tension_max_N"], 2), x2=round(INSERT_PULLOUT_N / r2["beam_tension_max_N"], 2),
                                                      stallS1=round(INSERT_PULLOUT_N / sS1["beam_tension_max_N"], 2), S1stop=round(INSERT_PULLOUT_N / st["beam_tension_max_N"], 2)))
    out["derived_dish"] = dish_checks(der["x2"]["rail_p0_required_kPa"] + L0["pose_scan_x2"]["liner_max"]["value"])
    json.dump(out, open(os.path.join(OUT, "loads.json"), "w"), indent=1, default=float)
    print(json.dumps(dict(preload=der, pacf=out["derived_pacf"], dish=out["derived_dish"]), indent=1, default=float))
