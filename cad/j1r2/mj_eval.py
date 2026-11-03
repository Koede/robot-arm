"""Evaluate every J1 mount concept (mj_concepts.CONCEPTS): frame load cases, mass, skin gaps by zone, clearance to the moving shoulder stage
over the HARD ranges, wearer range of motion, depth, balancer home; exports per-concept STLs + metrics and a render scene.
usage: J1R2_ROOT=/home/claude/ts python3 mj_eval.py [CID,CID..]   -> stl/j1r2_mounts/<CID>/..., stl/j1r2_mounts/summary.json"""
import os, sys, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import clib as L, j1r2_params as P, j1r2_geom as G, mj_frame as M, mj_concepts as C, mj_concepts2, mj_concepts3   # noqa: F401  (registers MJ7..MJ10 no balancer; MJ11 blade-seated pad)
OUT = os.path.join(M.REPO, "stl", "j1r2_mounts"); os.makedirs(OUT, exist_ok=True)
DENS = dict(L.DENS); DENS.update(CF=1.55, AL=2.70, TPU=1.21, STEEL=7.85)
t0 = time.time(); log = lambda *a: print("[%5.0fs]" % (time.time() - t0), *a, flush=True)
BASE, INFO = G.build(y1=M.Y1, z1=M.Z1, standoff=M.DER["standoff"], full=True); DER = INFO["derived"]
REPLACED = ("pad_riser", "standoffs", "fairing_lap")
LIM = dict(S1p=(-95.0, 35.0), S2=(-30.0, 80.0))     # HARD ranges of the J1 rev 2 recommended soft limits (S1' -90..30, S2 -25..75; SPEC_j1r2_integration section 5)
M1N = ("S1_yoke", "S1_stop_collar", "S2_AK70", "S2_clamp", "S2_sleeve", "S2_stop_posts", "tpu_S2_bumpers", "cable_M1")
M2N = ("S2_adapter", "S3_bracket", "S3_XM540", "S2_stop_lug", "arm_tube", "arm_elbow_ball", "cable_M2")
def samp(m, sp=1.2):
    p, _ = trimesh.sample.sample_surface_even(m, int(m.area / sp ** 2) + 300); return np.vstack([p, m.vertices]) if len(p) else m.vertices
MOV1 = np.vstack([samp(BASE[k]["mesh"], 2.0) for k in M1N]); MOV2 = np.vstack([samp(BASE[k]["mesh"], 2.0) for k in M2N])
def mass_of(p):
    if p.get("mass_g") is not None: return float(p["mass_g"])
    m = p["mesh"]; return float(abs(m.volume) / 1000.0 * DENS[p["mat"]] * p.get("fill", 1.0)) if m.is_watertight else float("nan")
# ------------------------------------------------------------------ frame load cases
def run_frame(cid, mount_g=0.0, stiff=False):
    Cc = C.CONCEPTS[cid]; res = {}
    for lc, sides in M.LCS.items():
        fr = M.Frame(); M.harness(fr, "R"); M.harness(fr, "L"); M.spine_bridge(fr)
        pads = {s: Cc["frame"](fr, s) for s in ("R", "L")}
        if stiff:                                    # mount members x1000 stiffer: what is left is the harness on the body (foam + tissue)
            for b in fr.beams:
                if b["tag"] not in ("rail", "spine_bridge", "clasp_arm", "clasp_foot_link", "pad"): b["E"] *= 1000.0; b["G"] *= 1000.0
        for s in ("R", "L"):
            th, ph, pay = sides["right" if s == "R" else "left"]; M.apply_arm(fr, pads[s], s, th, ph, pay, extra_g=mount_g)
        u = fr.solve(); i = fr.idx["R_pad"]; pn = fr.nodes[i]; th, ph, pay = sides["right"]
        tip = M.hand_tip(th, ph); dtip = u[i, :3] + np.cross(u[i, 3:], tip - pn)
        sp = fr.spring_forces(); rows = {}
        for kind in ("rail", "top", "front", "axilla", "liner"):
            ps = [s_ for s_ in sp if s_["kind"] == kind and s_["label"].startswith("R_")]
            if ps: rows[kind] = dict(max_kPa=round(max(s_["p_kPa"] for s_ in ps), 2), min_kPa=round(min(s_["p_kPa"] for s_ in ps), 2), sum_N=round(sum(s_["F"] for s_ in ps), 1))
        bolts = []
        for b in fr.beams:
            if b["tag"] not in ("iface",) or fr.nodes[b["a"]][1] < M.SPINE_Y: continue
            R_, Lb, kg = b["_RL"]; ue = np.r_[u[b["a"]], u[b["b"]]]; f = np.kron(np.eye(4), R_) @ (kg @ ue); N, My, Mz = f[0], f[4], f[5]
            if "bolts" in b:
                bo = np.asarray(b["bolts"], float); n_b = len(bo); su = float((bo[:, 0] ** 2).sum()) or 1e9; sv = float((bo[:, 1] ** 2).sum()) or 1e9
                Fi = N / n_b + My * bo[:, 1] / sv - Mz * bo[:, 0] / su
                bolts.append(dict(n=n_b, F_bolt_max_N=round(float(np.abs(Fi).max()), 1), N=round(N, 1), My_Nmm=round(My, 0), Mz_Nmm=round(Mz, 0)))
            else: bolts.append(dict(n=0, contact=True, N=round(N, 1), My_Nmm=round(My, 0), Mz_Nmm=round(Mz, 0)))
        stress = {}
        for b in fr.beams:
            if b["tag"] in ("pad", "rigid", "iface", "rec_bolts", "clasp_foot_link") or b["E"] >= M.MAT["STEEL"][0] or fr.nodes[b["a"]][1] < M.SPINE_Y - 1: continue
            R_, Lb, kg = b["_RL"]; ue = np.r_[u[b["a"]], u[b["b"]]]; f = np.kron(np.eye(4), R_) @ (kg @ ue)
            for e0 in (0, 6):
                N, My, Mz = f[e0], f[e0 + 4], f[e0 + 5]; cz, cy = b.get("cz", 5), b.get("cy", 5)
                sig = abs(N) / b["A"] + abs(My) * cz / b["Iy"] + abs(Mz) * cy / b["Iz"]; stress[b["tag"]] = max(stress.get(b["tag"], 0), sig)
        res[lc] = dict(tip_mm=round(float(np.linalg.norm(dtip)), 2), tip_vec=np.round(dtip, 2).tolist(), pad_rot_deg=round(float(np.degrees(np.linalg.norm(u[i, 3:]))), 3),
                       pad_trans_mm=round(float(np.linalg.norm(u[i, :3])), 2), contact=rows, iface=bolts, member_stress_MPa={k: round(v, 2) for k, v in stress.items()})
    return res
# ------------------------------------------------------------------ geometry checks
TK, TP, TN = L._tree()
def skin_zones(parts):
    seats = np.array([np.asarray(M.SITES[k]["p"], float) for k in ("F1_upper_trap", "F4_acromion")]); out = {}
    for k, p in parts.items():
        if p["kind"] == "cord": continue
        s_ = samp(p["mesh"], 1.5); d, i = TK.query(s_); sd = np.sign(np.einsum("ij,ij->i", s_ - TP[i], TN[i])) * d
        near_seat = np.min(np.linalg.norm(s_[:, None, :] - seats[None], axis=2), axis=1) < 30.0
        zone = np.where(near_seat, "seat", np.where(s_[:, 1] <= -100.0, "rail", np.where(s_[:, 0] < -150.0, "top", "blade")))
        r = {}
        for z in ("rail", "blade", "top", "seat"):
            m_ = zone == z
            if m_.any(): r[z] = round(float(sd[m_].min()), 1)
        out[k] = r
    return out
RULE = dict(rail=5.0, blade=20.0, top=10.0)
def stage_clearance(parts):
    pts = np.vstack([samp(p["mesh"], 1.2) for k, p in parts.items() if p["kind"] != "cord" and not k.startswith("bal_cassette")])
    r = np.hypot(pts[:, 1] - DER["y1"], pts[:, 2] - DER["z1"]); keep = ~((r < 68.5) & (pts[:, 0] > DER["x0"] - 1.0)); pts = pts[keep]     # the pad disc / stop-post zone was verified in J1 rev 2 (designed proximity)
    T = cKDTree(pts); best1, best2 = (1e9, None), (1e9, None)
    for th in np.arange(LIM["S1p"][0], LIM["S1p"][1] + 0.1, 5.0):
        Mx = G.pose_matrix("M1", DER, th, 0.0); q = MOV1 @ Mx[:3, :3].T + Mx[:3, 3]; d = float(T.query(q, distance_upper_bound=40)[0].min())
        if d < best1[0]: best1 = (d, th)
        for ph in np.arange(LIM["S2"][0], LIM["S2"][1] + 0.1, 10.0):
            if th <= -85.0 and ph > 50.0: continue                  # J1 rev 2 recommendation: S2 soft cap 45 (hard 50) for S1' <= -85 (arm vs skin frontier)
            Mm = G.pose_matrix("M2", DER, th, ph); q = MOV2 @ Mm[:3, :3].T + Mm[:3, 3]; d = float(T.query(q, distance_upper_bound=40)[0].min())
            if d < best2[0]: best2 = (d, (th, ph))
    return dict(M1_min_mm=round(min(best1[0], 40.0), 1), M1_at_S1p=best1[1], M2_arm_min_mm=round(min(best2[0], 40.0), 1), M2_at=best2[1])
def _wing_rail():
    w = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")); return L.I(w, L.box(-300, 100, -150, -100.0, -500, 100))   # rail strip only: the bridge + tongue are retired with the chassis plate
HARN = {"wing_rail": cKDTree(samp(_wing_rail(), 1.2)), "spine_bridge": cKDTree(samp(trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "spine_bridge.stl")), 1.2))}
def harness_clear(parts):
    """min distance (mm) of each new part to the verified wing (rail + bridge + tongue) and spine bridge meshes; < 0.5 = touching (designed for rail-bolted parts)."""
    return {k: {h: round(float(T.query(samp(p["mesh"], 1.5), distance_upper_bound=30)[0].min()), 1) for h, T in HARN.items()} for k, p in parts.items() if p["kind"] != "cord"}
# wearer ROM (same model as verify_armor.py: rigid GH rotation to 110 deg, cervical-blended head rotation)
BODY = L.body(); V = BODY.vertices; GH = np.array(L.LM["gh_joint_rel_P"], float); NECK = np.array(L.LM["neck_joint_rel_P"], float)
_ax = np.array([0.0, np.sin(np.radians(10)), -np.cos(np.radians(10))]); _t = (V - GH) @ _ax; _rad = V - GH - _t[:, None] * _ax
ARMV = V[(_t > 25) & (_t < 320) & (np.linalg.norm(_rad, axis=1) < 62) & (_rad[:, 1] > -40)]
_hv = (V[:, 2] > NECK[2]) & (np.abs(V[:, 1] - NECK[1]) < 115) & (V[:, 0] < -60); HEAD = V[_hv]; HW = np.clip((HEAD[:, 2] - NECK[2]) / 50.0, 0, 1)
def _rot(axis, ang, piv, P_, w=None):
    axis = np.asarray(axis, float) / np.linalg.norm(axis); a = np.radians(ang) * (w if w is not None else 1.0); Q = P_ - piv
    c, s = np.cos(a), np.sin(a)
    if np.ndim(c): c, s = c[:, None], s[:, None]
    return Q * c + np.cross(axis[None, :], Q) * s + axis[None, :] * (Q @ axis)[:, None] * (1 - c) + piv
def wearer_rom(parts):
    pts = np.vstack([samp(p["mesh"], 1.5) for p in parts.values() if p["kind"] != "cord" and not p.get("skin_contact")]); T = cKDTree(pts); out = {}
    for nm, tgt in (("arm abduction", [0, 1.0, 0]), ("arm flexion", [-1.0, 0, 0]), ("arm scaption 30", [-0.5, 0.866, 0])):
        ax = np.cross([0, 0, -1.0], tgt); out[nm] = None
        for a in np.arange(0, 110.1, 2.5):
            if T.query(_rot(ax, a, GH, ARMV), distance_upper_bound=3)[0].min() < 2.0: out[nm] = float(a); break
    for nm, ax, rng in (("neck lat flex R", [-1.0, 0, 0], 60), ("neck rotation R", [0, 0, -1.0], 90), ("neck flexion", [0, -1.0, 0], 70), ("neck extension", [0, 1.0, 0], 70)):
        out[nm] = None
        for a in np.arange(0, rng + 0.1, 2.5):
            if T.query(_rot(ax, a, NECK, HEAD, HW), distance_upper_bound=3)[0].min() < 2.0: out[nm] = float(a); break
    return out
def head_clear(meshes):
    """min distance (mm) of fixed parts to the head/neck (z above the neck joint) at the neutral head pose (60 mm gate applies to MOVING parts; reported)."""
    pts = np.vstack([samp(m, 2.0) for m in meshes]); return round(float(cKDTree(HEAD).query(pts)[0].min()), 1)
# ------------------------------------------------------------------ balancer homes
def radial_tube():
    r = np.array([0.0, C.A_SHEAVE[1] - DER["y1"], C.A_SHEAVE[2] - DER["z1"]]); r /= np.linalg.norm(r); a = C.A_SHEAVE + r * 6.0
    return C.tubebeam(a, a + r * 236.0, 44.5, 2.5)
def balancer_check(cid, parts):
    kind = C.CONCEPTS[cid]["balancer"]
    if kind == "none": return dict(home="none (no balancer)", verdict="no balancer: shoulder motors carry the full gravity torque (SF 1.14 vs rated at the worst pose, gate waiver needed)")
    if kind == "radial":
        t = radial_tube(); sk = L.skin_gap(t, 3000)[0]; hc = head_clear([t]); top = float(t.bounds[1][2])
        T = cKDTree(samp(t, 2.0)); best = 1e9
        for th in np.arange(LIM["S1p"][0], LIM["S1p"][1] + 0.1, 10.0):
            for ph in np.arange(LIM["S2"][0], LIM["S2"][1] + 0.1, 10.0):
                Mm = G.pose_matrix("M2", DER, th, ph); q = MOV2 @ Mm[:3, :3].T + Mm[:3, 3]; best = min(best, float(T.query(q, distance_upper_bound=60)[0].min()))
        return dict(home="radial tube on the pad (J1 rev 2 unresolved option)", tube_top_z=round(top, 0), skin_mm=round(sk, 1), head_neutral_mm=hc, arm_sweep_min_mm=round(min(best, 60), 1),
                    verdict="tube rises to z %.0f beside the head (%.0f mm at neutral) and the arm sweep reaches it (%.1f mm)" % (top, hc, min(best, 60)), mesh=t)
    if kind == "spar":
        cord = [p for k, p in parts.items() if k == "bal_cord_fixed"][0]["mesh"]; T = cKDTree(samp(cord, 1.0)); b1 = b2 = 1e9
        for th in np.arange(LIM["S1p"][0], LIM["S1p"][1] + 0.1, 5.0):
            Mx = G.pose_matrix("M1", DER, th, 0.0); b1 = min(b1, float(T.query(MOV1 @ Mx[:3, :3].T + Mx[:3, 3], distance_upper_bound=60)[0].min()))
            for ph in np.arange(LIM["S2"][0], LIM["S2"][1] + 0.1, 10.0):
                Mm = G.pose_matrix("M2", DER, th, ph); b2 = min(b2, float(T.query(MOV2 @ Mm[:3, :3].T + Mm[:3, 3], distance_upper_bound=60)[0].min()))
        cl = float(np.linalg.norm(np.diff(np.array([cord.bounds[0], cord.bounds[1]]), axis=0)))
        fair = BASE["fairing_fixed"]["mesh"]; fc = float(cKDTree(samp(fair, 1.2)).query(samp(cord, 1.0))[0].min())
        return dict(home="spring + piston inside the spar bore (z -66..-298), cord up over a spar-head sheave to the pad sheave A", cord_vs_M1_min_mm=round(min(b1, 60), 1), cord_vs_arm_min_mm=round(min(b2, 60), 1),
                    cord_vs_fixed_fairing_mm=round(fc, 1), verdict="spring off the shoulder entirely; fixed cord run %.1f mm from the S1' group, %.1f mm from the arm sweep, %.1f mm from the fairing (needs a guide channel)" % (min(b1, 60), min(b2, 60), fc))
    if kind == "cassette":
        cs = parts["bal_cassette"]["mesh"]; return dict(home="flat spiral spring + cam drum inside the shell (NEW balancer type, not designed)", skin_mm=round(L.skin_gap(cs, 2000)[0], 1),
                    verdict="fits the shell depth, but a cam-drum spiral balancer that reproduces the zero-length cord law over both axes is not designed (PLACEHOLDER)")
def evaluate(cid):
    Cc = C.CONCEPTS[cid]; parts = Cc["geo"](BASE); log(cid, "geometry", len(parts), "parts")
    rows = {k: dict(mat=p["mat"], kind=p["kind"], mass_g=round(mass_of(p), 1), watertight=bool(p["mesh"].is_watertight), bbox=np.round(p["mesh"].extents, 1).tolist(),
                    fits_256=bool(np.all(p["mesh"].bounding_box_oriented.extents <= 256)), note=p.get("note", "")) for k, p in parts.items()}
    if cid == "MJ3": rows["yoke_tube"]["mass_g"] = round(rows["yoke_tube"]["mass_g"] / 2, 1); rows["yoke_tube"]["note"] += " (mass: half per side)"
    bal_fixed = dict(radial=307.4, spar=108.4 + 28.0, cassette=0.0, none=0.0)[Cc["balancer"]]          # REF radial: tube 96 + cap 31 + post 34 + spring 108 + piston 28 + sheave 9 + cord 1; spar: spring + piston (liner/sheaves counted as parts)
    mount_g = sum(r["mass_g"] for k, r in rows.items() if r["kind"] in ("printed", "bought", "cord"))
    fr_res = run_frame(cid, mount_g); fr_stiff = run_frame(cid, mount_g, stiff=True); fr_res["park"]["tip_mm_body_only"] = fr_stiff["park"]["tip_mm"]; fr_res["reach"]["tip_mm_body_only"] = fr_stiff["reach"]["tip_mm"]; sk = skin_zones(parts); viol = []
    rule = dict(RULE, **Cc.get("rule", {}))
    for k, z in sk.items():
        if parts[k].get("skin_contact"): continue                       # designed skin interface (foam liner): reported in skin_gap_by_zone_mm, not a rule violation
        for zn, v in z.items():
            if zn in rule and v < rule[zn]: viol.append(f"{k}: {zn} {v} mm < {rule[zn]}")
    cl = stage_clearance(parts); log(cid, "stage clearance", cl); rom = wearer_rom(parts); log(cid, "rom", rom)
    cl["by_part"] = {k: stage_clearance({k: p}) for k, p in parts.items() if p["kind"] != "cord" and not k.startswith("bal_cassette")}
    rom_by = {k: {a: b for a, b in wearer_rom({k: p}).items() if b is not None} for k, p in parts.items() if p["kind"] != "cord" and not p.get("skin_contact")}
    fixed = [p["mesh"] for p in parts.values() if p["kind"] != "cord"]; depth = round(max(m.bounds[1][0] for m in fixed) + 108.0, 1)
    bal = balancer_check(cid, parts); bal_mesh = bal.pop("mesh", None)
    hc = harness_clear(parts)
    met = dict(id=cid, harness_clearance_mm=hc, title=Cc["title"], clasp=Cc["clasp"], parts=rows, mount_mass_g=round(mount_g, 1), balancer_fixed_g=bal_fixed, total_fixed_g=round(mount_g + bal_fixed, 1),
               frame=fr_res, skin_gap_by_zone_mm=sk, skin_rule_violations=viol, stage_clearance=cl, wearer_rom_first_contact_deg=rom, wearer_rom_by_part={k: v for k, v in rom_by.items() if v}, fixed_depth_behind_back_mm=depth,
               head_neutral_mm=head_clear(fixed), balancer=bal)
    if "fairing" in parts:
        import mj11_fairing_check as FK; met["fairing_check"] = FK.run(parts=parts); log(cid, "fairing check", {k: v for k, v in met["fairing_check"].items() if k != "moving_min_mm"})
    d = os.path.join(OUT, cid); os.makedirs(d, exist_ok=True)
    for k, p in parts.items():
        p["mesh"].export(os.path.join(d, f"{cid}_{k}_R.stl")); G.mirror_L(p["mesh"]).export(os.path.join(d, f"{cid}_{k}_L.stl"))
    if bal_mesh is not None: bal_mesh.export(os.path.join(d, f"{cid}_bal_tube_radial_R.stl"))
    json.dump(dict(met, colours={k: list(p["colour"]) for k, p in parts.items()}), open(os.path.join(d, "metrics.json"), "w"), indent=1, default=float)
    log(cid, "done: mount %.0f g + balancer %.0f g, tip park %.2f / reach %.2f mm" % (mount_g, bal_fixed, fr_res["park"]["tip_mm"], fr_res["reach"]["tip_mm"])); return met
if __name__ == "__main__":
    ids = sys.argv[1].split(",") if len(sys.argv) > 1 else list(C.CONCEPTS)
    allm = {}
    for cid in ids: allm[cid] = evaluate(cid)
    sp = os.path.join(OUT, "summary.json"); old = json.load(open(sp)) if os.path.exists(sp) else {}; old.update(allm); json.dump(old, open(sp, "w"), indent=1, default=float)
