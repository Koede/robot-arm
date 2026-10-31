"""Rev B pose library -> stl/hand/poses.json.  Joint vector per pose: fingers {nm: (theta_MCP, abduction)}, thumb (swing, lift, flex); RHO_T fixed.
Chuck poses come from hand_poses.py (chuck_solve.json).  Fist angle chosen as the largest MCP keeping >= 1 mm to the palm box; thumb tucks solved by DE."""
import numpy as np, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as K, hand_poses as HP
from scipy.optimize import differential_evolution
K.RHO_T = 110.0
CH = json.load(open(os.path.join(K.OUT, "chuck_solve.json")))
def palm_clear(c, r):
    worst = 1e9
    for (p, q) in K.segs_finger(c)[1:]:
        for t in np.linspace(0, 1, 9):
            x = p + t*(q - p); dx = max(HP.PALM['x'][0] - x[0], x[0] - HP.PALM['x'][1], 0); du = max(HP.PALM['u'][0] - x[1], x[1] - HP.PALM['u'][1], 0); dz = max(HP.PALM['z'][0] - x[2], x[2] - HP.PALM['z'][1], 0)
            worst = min(worst, np.sqrt(dx*dx + du*du + dz*dz) - r)
    return worst
FIST = {}
for nm in K.LEN:
    best = 0
    for th in np.arange(40, 101, 1.0):
        if palm_clear(K.chain(nm, th), HP.RAD[nm]) >= 1.0: best = th
    FIST[nm] = float(best)
def thumb_tuck(fingers, target_nm, seed=3):
    C0 = {nm: K.chain(nm, th, ps) for nm, (th, ps) in fingers.items()}
    tc = C0[target_nm]; mid = 0.5*(tc['P1'] + tc['P2']); out = -tc['nfun'](tc['S'][1])   # outward face of the curled middle phalanx (its dorsal side faces forward in a fist)
    tgt = mid + out*(HP.RAD[target_nm] + K.TIP_R + 1.5)
    def cost(v):
        t = K.thumb(*v); c = np.sum((t['tipc'] - tgt)**2) + 3*HP.thumb_palm_pen(t)
        for nm, cc in C0.items():
            for (p1, q1) in K.segs_finger(cc):
                for (p2, q2) in K.segs_thumb(t)[:2]:
                    d = K.seg_seg(p1, q1, p2, q2) - HP.RAD[nm] - HP.RAD['thumb']
                    if d < 0.5: c += 10*(0.5 - d)**2
        return c
    r = differential_evolution(cost, [(-10, 110), (-10, 110), (0, 70)], seed=seed, maxiter=200, popsize=20, tol=1e-10, polish=True)
    return [float(x) for x in r.x], float(r.fun)
P = {}
P['open'] = dict(f={nm: (0.0, 0.0) for nm in K.LEN}, th=[0.0, 0.0, 0.0])
def from_chuck(v, jaw3):
    f = dict(index=(v[0], v[3]), middle=(v[1], 0.0), ring=((FIST['ring'] if jaw3 else v[2]), v[3]), pinky=(FIST['pinky'] - (12.0 if jaw3 else 0.0), 0.0)); return dict(f=f, th=list(v[4:7]))
P['chuck4_d13'] = from_chuck(CH['chuck_d13'], False)
P['chuck3_d6'] = from_chuck(CH['chuck3_d6'], True)
fist = {nm: (FIST[nm], 0.0) for nm in K.LEN}
P['thumbsup'] = dict(f=dict(fist), th=[0.0, -5.0, 0.0])
pt = dict(fist); pt['index'] = (0.0, 0.0); th, e1 = thumb_tuck(pt, 'middle'); P['point'] = dict(f=pt, th=th)
v = dict(fist); v['index'] = (0.0, -15.0); v['middle'] = (0.0, 0.0); th2, e2 = thumb_tuck(v, 'ring'); P['victory'] = dict(f=v, th=th2)
P = {k: dict(f={n: [float(a), float(b)] for n, (a, b) in p['f'].items()}, th=[float(x) for x in p['th']]) for k, p in P.items()}
json.dump(dict(RHO_T=K.RHO_T, FIST=FIST, poses=P, tuck_cost=dict(point=e1, victory=e2)), open(os.path.join(K.OUT, "poses.json"), "w"), indent=1)
print('FIST', FIST, 'tuck costs', round(e1, 2), round(e2, 2))
for k, p in P.items():
    C = {nm: K.chain(nm, *p['f'][nm]) for nm in K.LEN}; C['thumb'] = K.thumb(*p['th'])
    pen, worst = HP.clearance_pen(C); print(k, 'worst digit clearance (capsule)', round(worst, 2), 'thumb-palm', HP.thumb_palm_pen(C['thumb']))
