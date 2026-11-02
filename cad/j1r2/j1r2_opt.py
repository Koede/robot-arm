"""Local-geometry optimisation of J1 rev 2: pad position (y1, z1) and standoff against the collision tables. Score = baseline-arm-direction coverage of the best
passive-stop box (S1' lo..hi, S2 lo..hi containing park) whose +-5 deg expansion (the hard stops) is free of every gate (head/neck 60, skin 2, parts 3)."""
import sys, os, json, time, itertools; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, j1r2_params as P, j1r2_sweep as S, j1r2_integrate as I
def allowed_map(out):
    pairs, tabs, vals, designed = (out["_internal"][k] for k in ("pairs", "tabs", "vals", "designed")); J = list(vals); shape = tuple(len(vals[j]) for j in J)
    ok = np.ones(shape, bool); worst = {}
    for pi, (p, d) in enumerate(zip(pairs, designed)):
        t = tabs[pi]
        if d: continue
        t = np.asarray(t); full = S._bcast(t, p.dims, J) if (p.dims and t.ndim == len(p.dims)) else (t.reshape([1] * len(J)) if t.size == 1 else (print('SHAPE?', p.key, p.dims, t.shape) or t.reshape([1] * len(J)))); bad = full < p.MM; ok &= ~bad; worst[p.key] = float(np.min(full))
    return ok, vals, worst
_B = None
def _preimage():
    global _B
    if _B is None:
        B = L.baseline_dir_set(2.0); ph = np.degrees(np.arcsin(np.clip(-B[:, 0], -1, 1))); th = np.degrees(np.arctan2(-B[:, 1], B[:, 2])); _B = (th, ph)
    return _B
def best_box(ok, vals, margin_cells=1, tol=3.0):
    """maximise the arm-direction coverage (baseline set, preimage in (th1,phi) within tol) of a rectangle [S1p lo..hi] x [S2 lo..hi] containing park whose
    expansion by margin_cells (= the hard stops, 5 deg) is entirely free."""
    ok2 = ok.reshape(ok.shape[0], ok.shape[1], -1).all(axis=2); n1, n2 = ok2.shape; S1, S2 = vals["S1p"], vals["S2"]
    ip = int(np.argmin(abs(S1 - P.PARK["S1p"]))); jp = int(np.argmin(abs(S2 - P.PARK["S2"])))
    if not ok2[ip, jp]: return None
    C = np.zeros((n1 + 1, n2 + 1)); C[1:, 1:] = np.cumsum(np.cumsum(~ok2, 0), 1)
    bad = lambda a, b, c, d: C[b + 1, d + 1] - C[a, d + 1] - C[b + 1, c] + C[a, c]
    th_b, ph_b = _preimage(); N = len(th_b); best = (-1, None)
    # cumulative histogram of preimages on the (S1p, S2) grid for O(1) coverage of a rectangle
    ti = np.clip(np.searchsorted(S1, th_b), 0, n1 - 1); pj = np.clip(np.searchsorted(S2, ph_b), 0, n2 - 1)
    H = np.zeros((n1, n2)); np.add.at(H, (ti, pj), 1.0); CH = np.zeros((n1 + 1, n2 + 1)); CH[1:, 1:] = np.cumsum(np.cumsum(H, 0), 1)
    cnt = lambda a, b, c, d: CH[b + 1, d + 1] - CH[a, d + 1] - CH[b + 1, c] + CH[a, c]
    for i0 in range(0, ip + 1):
        for i1 in range(ip, n1):
            for j0 in range(0, jp + 1):
                for j1 in range(jp, n2):
                    a, b, c, d = max(0, i0 - margin_cells), min(n1 - 1, i1 + margin_cells), max(0, j0 - margin_cells), min(n2 - 1, j1 + margin_cells)
                    if bad(a, b, c, d) != 0: break
                    cv = cnt(i0, i1, j0, j1) / N
                    if cv > best[0]: best = (cv, (i0, i1, j0, j1))
    if best[1] is None: return None
    i0, i1, j0, j1 = best[1]; return dict(ranges=dict(S1p=(float(S1[i0]), float(S1[i1])), S2=(float(S2[j0]), float(S2[j1]))), cov=float(best[0]), area=(i1 - i0 + 1) * (j1 - j0 + 1))
def evaluate(y1, z1, so, steps=None, gkw=None, label="opt"):
    t0 = time.time(); sc, parts, info = I.make_scene(y1, z1, so, "S3", gkw)
    steps = steps or {"S1p": 5.0, "S2": 5.0, "S3": [0.0]}
    out = S.run_sweep(sc, dict(steps=steps, procs=2, label=label)); ok, vals, worst = allowed_map(out); box = best_box(ok, vals)
    reach = out['_internal']['reach'].reshape(out['_internal']['reach'].shape[0], out['_internal']['reach'].shape[1], -1).any(axis=2); th_b, ph_b = _preimage(); S1, S2 = vals['S1p'], vals['S2']
    ti = np.clip(np.round(np.interp(th_b, S1, np.arange(len(S1)))).astype(int), 0, len(S1) - 1); pj = np.clip(np.round(np.interp(ph_b, S2, np.arange(len(S2)))).astype(int), 0, len(S2) - 1)
    inr = (th_b >= S1[0] - 2.5) & (th_b <= S1[-1] + 2.5) & (ph_b >= S2[0] - 2.5) & (ph_b <= S2[-1] + 2.5); reach_cov = float(np.mean(inr & reach[ti, pj]))
    return dict(y1=y1, z1=z1, standoff=so, reach_cov=reach_cov, box=box, park_ok=bool(box), worst=worst, depth=info.get("depth_behind_back_mm"), t=round(time.time() - t0, 1)), out
if __name__ == "__main__":
    r, _ = evaluate(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])); print(json.dumps({k: v for k, v in r.items() if k != "worst"}, indent=1)); print(sorted(r["worst"].items(), key=lambda kv: kv[1])[:8])
