"""Re-search of the spherical five-bar with the extra cost terms that wrist_parallel_kin.py lacks (does NOT edit it; imports its functions).
Extra terms (per serial target, using the best well-conditioned configuration within 1.5 deg of the target axis):
  cone:  the claw hand (tray radius 42 mm, 112 mm tall about the claw axis) fills a cone around +h, so no ground axis (+-J0, +-J4, where the servos sit)
         may come within CONE_SERVO deg of h, and no link point (crank 1, floater 1, crank 2 arcs) within CONE_LINK deg of h;
  roll:  the twist of the carrier frame (J2, h x J2, h) vs the serial frame Ry(-W2) Rx(W3) at the same axis; its span over the targets should be <= 20 deg.
Usage: python3 wrist_parallel_search.py [--iters N] [--seed S] [--out kin_v2.json]   (1 core, a few minutes)
"""
import os, sys, json, time, numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import differential_evolution
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import wrist_parallel as wp, wrist_parallel_kin as wk, rom_limits as rl
CONE_SERVO, CONE_LINK, ROLL_OK, TOL = 58.0, 48.0, 20.0, 1.5
CONE_FOREARM = 36.0          # links keep this many deg away from the forearm axis (-z): distal tube r 29 + 8 mm margin seen from the outer layer
CONE_FOREARM_ON = True

def serial_rolls_frames(h):
    """serial claw frame (N,3,3) with z = h: Ry(-W2) Rx(W3), W2 = -atan2(hx, hz), W3 = -asin(hy)."""
    w2 = -np.arctan2(h[:, 0], h[:, 2]); w3 = -np.arcsin(np.clip(h[:, 1], -1, 1)); th = -w2; ph = w3
    c, s, cp, sp = np.cos(th), np.sin(th), np.cos(ph), np.sin(ph); N = len(h); R = np.zeros((N, 3, 3))
    R[:, 0, 0] = c; R[:, 0, 1] = s * sp; R[:, 0, 2] = s * cp
    R[:, 1, 1] = cp; R[:, 1, 2] = -sp
    R[:, 2, 0] = -s; R[:, 2, 1] = c * sp; R[:, 2, 2] = c * cp
    return R

def arc_pts(a, b, n=5):
    om = np.arccos(np.clip(np.einsum("ij,ij->i", a, b), -1, 1))[:, None]; t = np.linspace(0, 1, n)[None, :, None]
    so = np.where(om < 1e-6, 1.0, np.sin(om))[..., None]
    return (np.sin((1 - t) * om[..., None]) * a[:, None, :] + np.sin(t * om[..., None]) * b[:, None, :]) / so

def evaluate(params, TGT, grid=61, detail=False):
    t = np.linspace(0, 360, grid, endpoint=False); T1, T2 = np.meshgrid(t, t, indexing="ij"); T1 = T1.ravel(); T2 = T2.ravel()
    P = wp.points(params, T1, T2); J0, J4 = P["J0"], P["J4"]
    sig, H, ok = wk.gain_min(params, T1, T2)
    good = ok[:, None] & (sig >= wk.S_MIN); frac_ok = float(good.mean())
    idx = np.argwhere(good)                                     # (M, 2): grid index, branch
    if len(idx) < 10: return 1e3, None
    ci, cb = idx[:, 0], idx[:, 1]
    h = H[ci, cb]; J1 = P["J1"][ci, cb]; J2 = P["J2"][ci, cb]; J3 = P["J3"][ci, cb]
    ang = lambda u, v: np.degrees(np.arccos(np.clip(u @ v, -1, 1)))
    a_ground = np.min([ang(h, J0), ang(h, -J0), ang(h, J4), ang(h, -J4)], axis=0)
    pts = np.concatenate([arc_pts(J0[None].repeat(len(h), 0), J1, 6), arc_pts(J1, J2, 8), arc_pts(J3, J4[None].repeat(len(h), 0), 6)], 1)   # (M, n, 3)
    a_link = np.degrees(np.arccos(np.clip(np.einsum("mnj,mj->mn", pts, h), -1, 1))).min(1)
    viol = np.clip(CONE_SERVO - a_ground, 0, None) + np.clip(CONE_LINK - a_link, 0, None)
    if CONE_FOREARM_ON:
        allp = np.concatenate([pts, arc_pts(J2, J3, 10)], 1)
        a_fore = np.degrees(np.arccos(np.clip(-allp[:, :, 2], -1, 1))).min(1); viol = viol + np.clip(CONE_FOREARM - a_fore, 0, None)
    Rs = serial_rolls_frames(h); Rc = np.stack([J2, np.cross(h, J2), h], 2); M = np.einsum("nji,njk->nik", Rs, Rc)
    roll = np.degrees(np.arctan2(M[:, 1, 0], M[:, 0, 0]))
    tree = cKDTree(h); near = tree.query_ball_point(TGT, 2 * np.sin(np.radians(TOL) / 2))
    worst_cov = 0.0; sel = []; cov_pen = []
    d0, _ = tree.query(TGT); angmin = np.degrees(2 * np.arcsin(np.clip(d0 / 2, 0, 1)))
    cov_pen = np.clip(angmin - 1.0, 0, None)
    # pass 1: per target pick the config with the least violation (ties: smallest |roll - r0|), r0 = roll at the target nearest (0,0)
    k0 = int(np.argmin(np.linalg.norm(TGT - np.array([0, 0, 1.0]), axis=1)))
    r0 = None; picks = []
    for it in range(2):
        picks = []
        for i, nb in enumerate(near):
            if not nb: picks.append(None); continue
            nb = np.array(nb); cost_i = viol[nb] * 5.0 + (np.abs(((roll[nb] - r0 + 180) % 360) - 180) * 0.1 if r0 is not None else 0)
            picks.append(int(nb[np.argmin(cost_i)]))
        r0 = roll[picks[k0]] if picks[k0] is not None else 0.0
    ok_t = np.array([p is not None for p in picks]); pk = np.array([p for p in picks if p is not None], dtype=int)
    if len(pk) == 0: return 1e3, None
    v = viol[pk]; rr = ((roll[pk] - r0 + 180) % 360) - 180
    span = float(rr.max() - rr.min()) if len(rr) else 360.0
    cost = float(cov_pen.mean() + 2 * cov_pen.max() + 20 * (1 - frac_ok) + 0.5 * v.mean() + 0.2 * v.max() + 0.3 * max(0.0, span - ROLL_OK) + 10 * (1 - ok_t.mean()))
    if detail: return cost, dict(cov_worst=float(angmin.max()), cone_viol_mean=float(v.mean()), cone_viol_max=float(v.max()), roll_span_deg=span, frac_ok=frac_ok, frac_targets=float(ok_t.mean()))
    return cost, None

if __name__ == "__main__":
    a = sys.argv; iters = int(a[a.index("--iters") + 1]) if "--iters" in a else 60; seed = int(a[a.index("--seed") + 1]) if "--seed" in a else 3
    outn = a[a.index("--out") + 1] if "--out" in a else "kin_v2.json"
    TGT, _ = wk.targets(step=10.0); bounds = [(0, 360), (10, 170), (0, 360), (10, 170), (20, 100), (20, 100), (30, 130), (30, 130)]
    base = wp.load_params(a[a.index("--x0") + 1] if "--x0" in a else None); print("baseline (kin.json):", evaluate(base, TGT, 91, True)[1])
    t0 = time.time()
    res = differential_evolution(lambda p: evaluate(p, TGT)[0], bounds, seed=seed, maxiter=iters, popsize=10, tol=1e-5, polish=False, workers=1, x0=base,
                                 updating="immediate")
    c, det = evaluate(res.x, TGT, 91, True); print("search", round(time.time() - t0), "s  cost", round(c, 3), det)
    TGT5, _ = wk.targets(step=5.0); cov = wk.coverage(res.x, TGT5, grid=181)
    out = dict(params=dict(zip(wp.KEYS, res.x.tolist())), cost=c, detail=det, worst_angle_deg=cov[2], frac_configs_well_conditioned=cov[1], s_min=wk.S_MIN,
               n_targets=len(TGT5), baseline=evaluate(base, TGT, 91, True)[1],
               note="extra cost: cone clearance of ground axes / links around the claw axis, roll span; see wrist_parallel_search.py")
    json.dump(out, open(os.path.join(wp.OUT, outn), "w"), indent=1); print(json.dumps(out, indent=1))
