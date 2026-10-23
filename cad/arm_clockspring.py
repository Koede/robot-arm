"""Clock-spring (spiral ribbon) sizing for a joint bay found by arm_bay.py.  Flat ribbon cable(s) wound as an Archimedean spiral between the rotating hub (inner end) and the fixed housing wall (outer end).
Tight state: the spiral hugs the hub (n_t turns, outer radius r_i + n_t * p).  Loose state: the cable has unwound against the outer wall (n_l turns at ~R_out).
Requirement: n_t - n_l >= dphi/360 + margin, and tight outer radius <= R_out - t/2, bend radius >= Rmin (ribbon thin axis) everywhere.
Formulas: L = 2 pi r_i n + pi p n^2 (tight) ; L = 2 pi (R_out - t/2 - n p/2) n (loose).  Inputs and results are written to stl/electronics/clockspring_<joint>.json.
Usage: python3 arm_clockspring.py S1"""
import sys, os, json, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wire_spec as S, rom_limits as RL
OUT = os.path.join(HERE, "..", "stl", "electronics")

def ribbon(joint, ribbons=1, gap=0.5, kind="ultra"):
    w = S.CROSS[joint]["wires"]; wid = sum(n * S.OD[g][kind] for _, g, n in w) / ribbons + 0.4; t = max(S.OD[g][kind] for _, g, n in w)
    return dict(width=round(wid, 2), t=t, pitch=ribbons * t + (ribbons + 1) * gap, rmin=8.0 * t, ribbons=ribbons)

def solve(joint, R_in, R_out, height, ribbons=1, margin_turns=0.1, clearance_h=1.0):
    rb = ribbon(joint, ribbons); p, t = rb["pitch"], rb["t"] * ribbons
    lo, hi = RL.HARD[joint]; dphi = hi - lo; need = dphi / 360.0 + margin_turns
    r_i = R_in + p / 2.0; r_o = R_out - p / 2.0; res = None
    if rb["width"] + clearance_h > height: return dict(ok=False, why="ribbon width %.1f + %.1f > bay height %.1f" % (rb["width"], clearance_h, height), ribbon=rb)
    for L in np.arange(200, 4000, 5.0):
        # tight: n from L = 2 pi r_i n + pi p n^2
        nt = (-2 * math.pi * r_i + math.sqrt((2 * math.pi * r_i) ** 2 + 4 * math.pi * p * L)) / (2 * math.pi * p)
        if r_i + nt * p > r_o + 1e-6: break                        # tight state no longer fits radially
        # loose: cable pressed on the outer wall, n turns of radius r_o - n p / 2
        nl = L / (2 * math.pi * r_o)
        for _ in range(30): nl = L / (2 * math.pi * (r_o - nl * p / 2.0))
        if nt - nl >= need: res = dict(ok=True, L_mm=float(L), n_tight=round(nt, 2), n_loose=round(nl, 2), dn=round(nt - nl, 2), need=round(need, 2), outer_r_tight=round(r_i + nt * p, 1))
    if res is None: return dict(ok=False, why="no length satisfies the unwind requirement within the bay", ribbon=rb, need=round(need, 2))
    res.update(ribbon=rb, R_in=R_in, R_out=R_out, height=height, dphi_deg=dphi, min_bend_radius_mm=round(r_i, 1), rmin_allowed=rb["rmin"]); return res

if __name__ == "__main__":
    j = sys.argv[1]; b = json.load(open(os.path.join(OUT, "bay_%s.json" % j))); R0, R1 = b["R"]; h0, h1 = b["h"]
    best = None
    for nrib in (1, 2):
        r = solve(j, R0 + 0.5, R1 - 0.5, h1 - h0, ribbons=nrib); print(j, nrib, "ribbon(s):", {k: v for k, v in r.items() if k != "ribbon"}, r.get("ribbon"))
        if r.get("ok") and best is None: best = r
    json.dump(best or {}, open(os.path.join(OUT, "clockspring_%s.json" % j), "w"), indent=1)
