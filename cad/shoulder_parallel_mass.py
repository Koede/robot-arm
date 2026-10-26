"""Mass / inertia comparison, serial S1-S2-S3 shoulder vs 3-RRR parallel shoulder (point-mass parts, about P). New file 2026-10-26.
Usage: python3 shoulder_parallel_mass.py [tag]   (needs stl/shoulder_parallel/link_*_<tag>_*.stl and platform_<tag>_park.stl from shoulder_parallel_pack.py)"""
import os, sys, json, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak
S = os.path.join(HERE, "..", "stl"); MM = json.load(open(os.path.join(S, "balancer", "s2_massmodel.json")))
q = dict(S1=45.0, S2=25.0, S3=0.0, E=45.0, W1=0, W2=0, W3=0, G=50.0); T = ak.frames(q)
G = ak.groups(g_pct=50.0)
def parts_M1():
    out = []
    for k, m in G["M1"].items():
        if k == "ak_s2": out.append((k, 621.0, m.bounds.mean(0))); continue
        out.append((k, abs(m.volume) / 1e3 * 1.27, m.center_mass if m.is_watertight else m.bounds.mean(0)))
    return out
def world_parts_chain():
    out = []
    for g in ("M2", "M3", "ME", "MW1", "MW2", "MW3"):
        for k, p in MM[g]["parts"].items(): out.append((g, k, p["g"], (T[g] @ np.r_[p["c"], 1])[:3]))
    return out
def inertia(parts, axis):
    axis = np.asarray(axis, float) / np.linalg.norm(axis); I = 0.0
    for m, c in parts:
        c = np.asarray(c); r = c - (c @ axis) * axis; I += m * 1e-3 * (r @ r) * 1e-6
    return I
chain = world_parts_chain()
m1 = [(m, (T["M1"] @ np.r_[c, 1])[:3]) for k, m, c in parts_M1()]
tot = lambda L: sum(m for m, c in L)
arm_all = [(p[2], p[3]) for p in chain]
m2 = [(p[2], p[3]) for p in chain if p[0] == "M2"]; rest = [(p[2], p[3]) for p in chain if p[0] != "M2"]
axis1 = [0, 0, 1.0]; axis2 = T["M1"][:3, :3] @ [0, 1.0, 0]
res = {}
res["serial"] = dict(M1_g=round(tot(m1)), M2_g=round(tot(m2)), beyond_S3_g=round(tot(rest)),
                     note="balancer adds ~0.23 kg (anchor 100, collar 42, tubecap 21, ...) not included; M1 includes AK70 S2 (621 g), M2 includes XM540 S3 (165 g)",
                     I_S1_kgm2=round(inertia(m1 + arm_all, axis1), 4), I_S2_kgm2=round(inertia(m2 + rest, axis2), 4), I_S3_kgm2=round(inertia(rest, T["M3"][:3, :3] @ [0, 0, 1.0]), 4),
                     moving_mass_S1_g=round(tot(m1) + tot(arm_all)), actuators_on_arm_g=621.0 + 165.0, actuators_on_harness_g=621.0)
tag = sys.argv[1] if len(sys.argv) > 1 else "s4"; R = T["M3"][:3, :3]
links = []
for i in (1, 2, 3):
    for nm in ("prox", "dist"):
        m = trimesh.load(os.path.join(S, "shoulder_parallel", f"link_{nm}_{tag}_{i}.stl")); links.append((abs(m.volume) / 1e3 * 1.2 + 15.0, m.bounds.mean(0)))
plat = trimesh.load(os.path.join(S, "shoulder_parallel", f"platform_{tag}_park.stl")); pm = (abs(plat.volume) / 1e3 * 1.2 * 0.35 + 30.0, plat.bounds.mean(0))
par_moving = [pm] + rest
links_dyn = [(m * 0.5, c * 0.6) for m, c in links]
res["parallel"] = dict(links_g=round(sum(m for m, c in links)), platform_g=round(pm[0]), beyond_platform_g=round(tot(rest)), motors_on_harness_g=3 * 621.0,
                       I_axis_z_kgm2=round(inertia(par_moving, axis1) + inertia(links_dyn, axis1), 4), I_axis_y_kgm2=round(inertia(par_moving, axis2) + inertia(links_dyn, axis2), 4),
                       I_roll_kgm2=round(inertia(rest, R @ [0, 0, 1.0]), 4), moving_mass_g=round(pm[0] + tot(rest) + sum(m for m, c in links_dyn)),
                       note="links lumped at 0.5 mass / 0.6 radius (they move at about half the platform rate); motor-side reflected inertia scales with 1/sigma^2 of G, i.e. 2x to 25x at weak poses")
print(json.dumps(res, indent=1)); json.dump(res, open(os.path.join(S, "shoulder_parallel", "mass_inertia.json"), "w"), indent=1)
