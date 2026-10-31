"""Rev B pose assembler: places parts from stl/hand/poses.json, checks the posed four-bar bar lengths, real-mesh collisions between digits/palm,
exports posed STLs (stl/hand/poses/) and render scenes (render_three.py)."""
import numpy as np, trimesh, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as K
OUT = K.OUT; D = K.D; FB = json.load(open(OUT + "/fourbars.json")); FBT = json.load(open(OUT + "/fourbar_thumb.json"))
PL = json.load(open(OUT + "/poses.json")); K.RHO_T = PL['RHO_T']; CH = json.load(open(OUT + "/chuck_solve.json"))
_c = {}
def mesh(n):
    if n not in _c: _c[n] = trimesh.load(f"{OUT}/{n}.stl", process=True)
    return _c[n].copy()
def T4(o, a, b):
    a = a/np.linalg.norm(a); b = b - a*(a @ b); b /= np.linalg.norm(b); t = np.cross(b, a); M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = a, t, b, o; return M
SHIFT = np.eye(4); SHIFT[1, 3] = D
MAT = dict(palm=([58, 61, 68], 0.15, 0.55), carrier=([58, 61, 68], 0.15, 0.55), phal=([214, 217, 222], 0.0, 0.45), cap=([236, 112, 36], 0.0, 0.75),
           bar=([70, 120, 205], 0.35, 0.35), bit=([200, 203, 210], 0.9, 0.25), servo=([30, 30, 32], 0.0, 0.6))
def place_all(p):
    out = []; audit = []
    for nm in K.LEN:
        th, ps = p['f'][nm]; c = K.chain(nm, th, ps); S1, S2, S3 = c['S']; d, n = c['dfun'], c['nfun']
        fr = dict(prox=(c['M'], d(S1), n(S1)), mid=(c['P1'], d(S2), n(S2)), dist=(c['P2'], d(S3), n(S3)), carrier=(c['M'], c['ez'], c['ec']))
        for k, f in fr.items(): out.append((f"{nm}_{k}", T4(*f), 'carrier' if k == 'carrier' else 'phal'))
        out.append((f"{nm}_cap", T4(*fr['dist']), 'cap'))
        for key, (fg, Gl, fh, Hl) in dict(pip=(fr['carrier'], FB[nm]['pip']['G'], fr['mid'], FB[nm]['pip']['H']),
                                          dip=((c['P1'], d(S1), n(S1)), FB[nm]['dip']['G'], fr['dist'], FB[nm]['dip']['H'])).items():
            Gw = fg[0] + Gl[0]*fg[1] + Gl[1]*fg[2]; Hw = fh[0] + Hl[0]*fh[1] + Hl[1]*fh[2]; v = Hw - Gw; ln = np.linalg.norm(v)
            audit.append(abs(ln - FB[nm][key]['lb'])); out.append((f"{nm}_bar_{key}", T4(Gw, v, fg[2]), 'bar'))
    t = K.thumb(*p['th']); fr = t['frames']
    for (o, a, b), pn in zip(fr, ('meta', 'prox', 'dist')): out.append((f"thumb_{pn}", T4(o, a, b), 'phal'))
    out.append(("thumb_cap", T4(*fr[2]), 'cap'))
    Gw = fr[1][0] + FBT['G'][0]*fr[0][1] + FBT['G'][1]*fr[0][2]; Hw = fr[2][0] + FBT['H'][0]*fr[2][1] + FBT['H'][1]*fr[2][2]
    audit.append(abs(np.linalg.norm(Hw - Gw) - FBT['lb'])); out.append(("thumb_bar", T4(Gw, Hw - Gw, fr[0][2]), 'bar'))
    out += [("palm_back", np.eye(4), 'palm'), ("palm_cover", np.eye(4), 'palm')]
    return out, max(audit)
def meshes(placed):
    res = {}
    for n, M, mat in placed:
        m = mesh(n); m.apply_transform(SHIFT @ M); res[n] = (m, mat)
    return res
def family(n): return n.split('_')[0]
DESIGNED = lambda a, b: (family(a) == family(b) and 'palm' not in a) or ({'palm_back', 'palm_cover'} == {a, b}) or \
    ('palm' in a + b and ('carrier' in a + b or 'thumb_meta' in a + b)) or ('bit' in (a, b) and ('cap' in a + b))   # bit vs TPU caps = the grip itself
def collisions(ms):
    cm = trimesh.collision.CollisionManager()
    for n, (m, _) in ms.items(): cm.add_object(n, m)
    _, pairs = cm.in_collision_internal(return_names=True); bad = []
    for a, b in pairs:
        if DESIGNED(a, b): continue
        try: v = ms[a][0].intersection(ms[b][0], engine='manifold').volume
        except Exception: v = -1
        bad.append((a, b, round(float(v), 2)))
    # min distance between different digits (pairs not colliding)
    return sorted(bad, key=lambda x: -x[2])
def bit_mesh(key, rb):
    v = CH[key]; p0 = np.array(v[8:11]); m = trimesh.creation.cylinder(radius=rb, segment=[p0 + [-8, 0, 0], p0 + [45, 0, 0]], sections=48); m.apply_translation([0, D, 0]); return m
POSES = PL['poses']; BITS = dict(chuck4_d13=('chuck_d13', 6.5), chuck3_d6=('chuck3_d6', 3.0))
TITLE = dict(open='Open hand', chuck4_d13='Chuck, 4 jaws (index, middle, ring, thumb) on a Ø13 bit', chuck3_d6='Chuck, 3 jaws (thumb, index, middle) on a Ø6 bit',
             point='Pointing 👉', thumbsup='Thumbs up 👍', victory='Victory ✌️')
if __name__ == "__main__":
    os.makedirs(OUT + "/poses", exist_ok=True); rep = {}; scenes = []
    for name, p in POSES.items():
        placed, err = place_all(p); ms = meshes(placed)
        if name in BITS: ms['bit'] = (bit_mesh(*BITS[name]), 'bit')
        bad = collisions(ms); grip = {}
        if 'bit' in ms:
            for n, (m, _) in ms.items():
                if n.endswith('cap'):
                    try:
                        v = ms['bit'][0].intersection(m, engine='manifold').volume
                        if v > 0: grip[n] = round(float(v), 2)
                    except Exception: pass
        rep[name] = dict(max_bar_len_err_mm=round(float(err), 3), interferences=bad, cap_bit_compression_mm3=grip)
        print(name, rep[name], flush=True)
        d = f"{OUT}/poses/{name}"; os.makedirs(d, exist_ok=True); parts = []
        for n, (m, mat) in ms.items():
            f = f"{d}/{n}.stl"; m.export(f); col, me, ro = MAT[mat]; parts.append(dict(stl=os.path.abspath(f), color=col, metal=me, rough=ro))
        trimesh.util.concatenate([m for m, _ in ms.values()]).export(f"{OUT}/poses/hand_{name}_posed.stl")
        scenes.append((name, parts))
    json.dump(rep, open(OUT + "/pose_report.json", "w"), indent=1)
    json.dump(scenes, open(OUT + "/scene_parts.json", "w"))
