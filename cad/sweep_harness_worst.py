"""Which poses and parts give the worst arm-vs-harness distance in sweep_full.py?

sweep_full.py reports only min_harness_mm. This reuses its grid, kinematics and meshes (imported, not edited) and records the
worst poses with the arm group and harness part involved, in the same two modes (full and --restricted).
Output: stl/sweep_harness_worst[_restricted].json (top N poses, plus the worst distance per harness part).
Usage: python3 sweep_harness_worst.py [--restricted] [N]"""
import os, sys, json, itertools, time, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sweep_full as sf, rom_limits as rl

RESTRICTED = "--restricted" in sys.argv
N = next((int(a) for a in sys.argv[1:] if a.isdigit()), 15)
(a0, a1), (b0, b1) = rl.SOFT["S1"], rl.SOFT["S2"]
S1v = [float(v) for v in np.linspace(a0, a1, 6)]; S2v = [float(v) for v in np.linspace(b0, b1, 8)]
S3v = [-70.0, 0.0, 90.0]; Ev = [0.0, 30.0, 60.0, 90.0, 120.0, 145.0]
wrists = [dict(W1=0, W2=0, W3=0), dict(W1=80, W2=80, W3=30), dict(W1=-80, W2=-70, W3=-20)]
if RESTRICTED:
    S3v = [-30.0, 0.0, 30.0]; wrists = [dict(W1=0, W2=0, W3=0), dict(W1=80, W2=30, W3=15), dict(W1=-80, W2=-30, W3=-15)]

rows = []; per_part = {}
t0 = time.time()
for s1, s2, s3, e, wi in itertools.product(S1v, S2v, S3v, Ev, range(len(wrists))):
    q = dict(S1=s1, S2=s2, S3=s3, E=e, G=0, **wrists[wi]); sf.setq(q)
    best = (1e9, None, None, None)
    for g in sf.ARM:
        d, names = sf.MG[g].min_distance_other(sf.mH, return_names=True)
        part = next((n for n in names if n in sf.HARN), "?")           # the harness-side name (plate, clasp, wing, spine, pauldron)
        arm_part = next((n for n in names if n not in sf.HARN), "?")
        if d < best[0]: best = (float(d), g, part, arm_part)
        per_part[part] = min(per_part.get(part, 1e9), float(d))
    rows.append(dict(d_mm=round(best[0], 1), arm_group=best[1], arm_part=best[3], harness_part=best[2], q=q))
rows.sort(key=lambda r: r["d_mm"])
out = dict(restricted=RESTRICTED, poses=len(rows), seconds=round(time.time() - t0), worst=rows[:N],
           worst_per_harness_part_mm={k: round(v, 1) for k, v in sorted(per_part.items(), key=lambda kv: kv[1])},
           n_below_0=sum(1 for r in rows if r["d_mm"] < 0), n_below_15=sum(1 for r in rows if r["d_mm"] < 15))
sfx = "_restricted" if RESTRICTED else ""
json.dump(out, open(os.path.join(HERE, "..", "stl", f"sweep_harness_worst{sfx}.json"), "w"), indent=1)
print(json.dumps({k: v for k, v in out.items() if k != "worst"}, indent=1))
for r in rows[:N]: print(r["d_mm"], r["arm_group"], r["arm_part"], "vs", r["harness_part"], {k: v for k, v in r["q"].items() if k in ("S1", "S2", "S3", "E", "W1", "W2", "W3")})
