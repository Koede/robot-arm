"""Rev B pose solver: chuck = bunched-fingertip pinch (index, middle, ring tips + thumb tip around a bit; free bit axis), and the 👉 👍 ✌️ poses.
Writes stl/hand/poses.json.  Clearance model: phalanges as capsules (radius = mean half-width/thickness), fingertips as spheres TIP_R."""
import numpy as np, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as K
from scipy.optimize import differential_evolution
RAD = {nm: sum(K.WID[nm])/2 + 0.3 for nm in K.LEN}; RAD['thumb'] = 7.6        # width-true (fingers sit side by side)
PALM = dict(x=(-13.0, 15.0), u=(-38.0, 30.0), z=(493.0, 546.0))
def inside_palm(p, m=0.0):
    return (PALM['x'][0]-m < p[0] < PALM['x'][1]+m) and (PALM['u'][0]-m < p[1] < PALM['u'][1]+m) and (PALM['z'][0]-m < p[2] < PALM['z'][1]+m)
def digits(v):
    thi, thm, thr, psi, sw, lift, tt, rho = v[:8]
    return {'index': K.chain('index', thi, psi), 'middle': K.chain('middle', thm), 'ring': K.chain('ring', thr, psi), 'thumb': K.thumb(sw, lift, tt, rho)}
def segs(nm, c): return K.segs_thumb(c) if nm == 'thumb' else K.segs_finger(c)
def clearance_pen(C, margin=0.4, skip_tip_pairs=False):
    pen = 0.0; names = list(C); worst = 1e9
    for i in range(len(names)):
        for j in range(i+1, len(names)):
            a, b = names[i], names[j]
            for ia, (p1, q1) in enumerate(segs(a, C[a])):
                for ib, (p2, q2) in enumerate(segs(b, C[b])):
                    d = K.seg_seg(p1, q1, p2, q2) - RAD[a] - RAD[b]; worst = min(worst, d)
                    if d < margin: pen += (margin - d)**2
    return pen, worst
def thumb_palm_pen(c):
    pen = 0.0
    for (p, q) in K.segs_thumb(c)[1:]:
        for t in np.linspace(0, 1, 6):
            x = p + t*(q - p)
            if inside_palm(x, RAD['thumb']*0.6): pen += 4.0
    return pen
USE = ('index', 'middle', 'thumb') if os.environ.get('JAW3') else ('index', 'middle', 'ring', 'thumb')
RING_FIST = 82.0
def chuck_cost(v, rb, info=False):
    C = digits(v)
    if 'ring' not in USE: C['ring'] = K.chain('ring', RING_FIST, v[3])
    Call = C; C = {k: c for k, c in C.items() if k in USE}; px, pu, pz, az, el = v[8:13]
    a = np.array([np.cos(np.radians(el))*np.cos(np.radians(az)), np.cos(np.radians(el))*np.sin(np.radians(az)), np.sin(np.radians(el))]); p0 = np.array([px, pu, pz])
    cost = 0.0; ang = []; s_ = []; padok = []
    e1 = np.cross(a, [0, 0, 1.0]) if abs(a[2]) < 0.9 else np.cross(a, [1.0, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(a, e1)
    for nm, c in C.items():
        w = c['tipc'] - p0; s = w @ a; r = w - s*a; dist = np.linalg.norm(r)
        cost += 4.0*(dist - (rb + K.TIP_R))**2
        nc = -r/(dist + 1e-9)                                   # from tip centre toward the axis = contact normal
        pd = nc @ c['n']; padok.append(pd); cost += (20.0*max(0, 0.2 - pd)**2 if nm == 'thumb' else 6.0*max(0, -0.4 - pd)**2)   # full TPU caps on the fingers; the thumb must use its pad
        ang.append(np.degrees(np.arctan2(r @ e2, r @ e1)) % 360); s_.append(s)
    sa = np.sort(ang); gaps = np.diff(np.r_[sa, sa[0] + 360]); gmin = 62 if len(USE) == 4 else 95
    cost += 0.02*sum(max(0, gmin - g)**2 for g in gaps) + 0.02*max(0, gaps.max() - 150)**2
    s_ = np.array(s_); cost += 0.5*sum(max(0, abs(x - s_.mean()) - 4.0)**2 for x in s_)
    pen, worst = clearance_pen(Call); cost += 3.0*pen + thumb_palm_pen(C['thumb'])
    # bit must not pass through the non-distal phalanges or the palm
    for nm, c in C.items():
        for (p, q) in segs(nm, c)[:2]:
            for t in np.linspace(0, 1, 5):
                x = p + t*(q - p); w = x - p0; dd = np.linalg.norm(w - (w @ a)*a)
                if w @ a > -8 and dd < rb + RAD[nm] + 0.5: cost += 0.5*(rb + RAD[nm] + 0.5 - dd)**2
    if info: return dict(cost=cost, gaps=np.round(gaps, 1).tolist(), pad_dot=np.round(padok, 2).tolist(), s_spread=round(float(np.ptp(s_)), 1), worst_clear=round(float(worst), 2), axis=np.round(a, 3).tolist(), p0=np.round(p0, 1).tolist())
    return cost
MAXIT, POP = 260, 14
def _cc(v, rb): return chuck_cost(v, rb)
B = [(0, 70), (0, 70), (0, 70), (-5, 25), (-10, 110), (-10, 110), (0, 70), (0, 180), (10, 90), (-25, 30), (540, 640), (0, 1e-6), (0, 1e-6)]
def solve_chuck(rb, seed=1, init=None, rho_fixed=None):
    b = list(B); rho_fixed = float(os.environ['RHO_FIX']) if os.environ.get('RHO_FIX') else rho_fixed
    if rho_fixed is not None: b[7] = (rho_fixed, rho_fixed + 1e-6)
    from functools import partial
    r = differential_evolution(partial(_cc, rb=rb), b, seed=seed, tol=1e-12, maxiter=MAXIT, popsize=POP, polish=True, init='latinhypercube' if init is None else init, workers=-1, updating='deferred')
    return r
if __name__ == "__main__":
    import time; t0 = time.time()
    best = None
    res = json.load(open(os.path.join(K.OUT, "chuck_solve.json"))) if os.path.exists(os.path.join(K.OUT, "chuck_solve.json")) else {}
    if os.environ.get('JAW3'):
        for rb in [float(x) for x in os.environ.get('RBS', '3.0,1.5').split(',')]:
            r = solve_chuck(rb, seed=7); res[f"chuck3_d{int(2*rb)}"] = r.x.tolist()
            print('3-jaw dia', 2*rb, round(r.fun, 4), chuck_cost(r.x, rb, True), round(time.time() - t0), 's', flush=True)
        json.dump(res, open(os.path.join(K.OUT, "chuck_solve.json"), "w"), indent=1); sys.exit()
    for rb in [float(x) for x in os.environ.get('RBS', '3.5,5.0,6.5').split(',')]:
        r = solve_chuck(rb, seed=7); res[f"chuck_d{int(2*rb)}"] = r.x.tolist()
        print('bit dia', 2*rb, round(r.fun, 4), chuck_cost(r.x, rb, True), round(time.time() - t0), 's', flush=True)
    json.dump(res, open(os.path.join(K.OUT, "chuck_solve.json"), "w"), indent=1)
