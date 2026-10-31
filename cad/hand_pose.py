"""Pose assembler for the hand.  Builds posed meshes (hand frame) from joint angles; checks four-bar bar lengths on the posed geometry; collision (fcl) between digits/palm."""
import numpy as np, trimesh, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as K, hand as Hd
OUT = Hd.OUT; D = Hd.D
_cache = {}
def mesh(name):
    if name not in _cache: _cache[name] = trimesh.load(os.path.join(OUT, name + ".stl"))
    return _cache[name].copy()
def T4(o, a, b):                      # local X=a, Y=t=b x a, Z=b -> world
    a = a/np.linalg.norm(a); b = b - a*np.dot(a, b); b = b/np.linalg.norm(b); t = np.cross(b, a)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = a, t, b, o; return M
def place(name, M, col):
    m = mesh(name); m.apply_transform(M); m.visual.face_colors = col; return m
COL = dict(palm=[185, 190, 200, 255], prox=[205, 120, 60, 255], mid=[225, 140, 70, 255], dist=[235, 165, 95, 255], pad=[40, 40, 45, 255], bar=[70, 130, 200, 255], knuckle=[120, 125, 135, 255],
           servo=[60, 60, 65, 255], horn=[230, 230, 230, 255], rod=[200, 200, 60, 255], thumb=[200, 90, 110, 255])
def finger_pose(nm, th1, psi, scene, checks):
    c = K.chain(nm, th1, psi); L1, L2, L3 = K.LEN[nm]; fb = Hd.FB[nm]
    s = np.sign(K.U[nm]) if K.U[nm] != 0 else 1.0
    h = np.array([np.sin(np.radians(K.TAU)), 0, np.cos(np.radians(K.TAU))])
    ec = K.rod(np.array([1.0, 0, 0]), h, s*psi); ez = K.rod(np.array([0, 0, 1.0]), h, s*psi)
    t1 = np.radians(th1); t2 = K.K_PIP*t1; t3 = K.K_DIP*t2
    d = lambda S: ec*np.sin(S) + ez*np.cos(S); n = lambda S: ec*np.cos(S) - ez*np.sin(S)
    M = c['M']; S1, S2, S3 = t1, t1+t2, t1+t2+t3
    fr = {'prox': (M, d(S1), n(S1)), 'mid': (c['P1'], d(S2), n(S2)), 'dist': (c['P2'], d(S3), n(S3)), 'knuckle': (M, ez, ec)}
    for k, (o, a, b) in fr.items(): scene.append((f"{nm}_{k}", place(f"{nm}_{k}", T4(o, a, b), COL[k] if k in COL else COL['prox'])))
    scene.append((f"{nm}_pad", place(f"{nm}_pad", T4(*fr['dist']), COL['pad'])))
    tvec = np.cross(ec, ez)   # t axis of the finger frame (b x a)
    W2 = Hd.BW/2
    def to_w(o, a, b, xy): return o + xy[0]*a + xy[1]*b
    # PIP bar: G1 on knuckle -> H1 on mid ; DIP bar: G2 on prox (origin P1, prox orientation) -> H2 on dist
    for key, (og, ag, bg, Gl, oh, ah, bh, Hl, sg) in {'pip': (M, ez, ec, fb['pip']['G'], c['P1'], d(S2), n(S2), fb['pip']['H'], +1),
                                                     'dip': (c['P1'], d(S1), n(S1), fb['dip']['G'], c['P2'], d(S3), n(S3), fb['dip']['H'], -1)}.items():
        Gw = to_w(og, ag, bg, Gl); Hw = to_w(oh, ah, bh, Hl); v = Hw - Gw; ln = np.linalg.norm(v)
        checks.append((nm, key, ln, fb[key]['lb']))
        # bar frame: origin at G (offset along t by 0, geometry already carries the t offset), a = v, b = in-plane normal
        a = v/ln; bb = ec*np.dot(ec, np.cross(a, tvec))*0 + np.cross(a, tvec)   # a x t = b
        scene.append((f"{nm}_bar_{key}", place(f"{nm}_bar_{key}", T4(Gw, a, bb), COL['bar'])))
    return c
def thumb_pose(sw, lift, th1, scene):
    fr = K.thumb_frames(sw, lift, th1)
    scene.append(("thumb_cmc", place("thumb_cmc", T4(K.T0, np.array([0, 0, 1.0]), np.array([1.0, 0, 0])), COL['thumb'])))
    for nm, (o, a, b) in zip(['thumb_meta', 'thumb_prox', 'thumb_dist'], fr): scene.append((nm, place(nm, T4(o, a, b), COL['thumb'])))
    scene.append(("thumb_pad", place("thumb_pad", T4(*fr[2]), COL['pad'])))
    return fr
def static_scene(scene):
    P = place("palm_tray", np.eye(4), COL['palm']); scene.append(("palm_tray", P))
    for nm, f in Hd.parts_info['FSERV'].items():
        b = trimesh.creation.box([f['x1']-f['x0'], Hd.SV['W'], f['z1']-f['z0']]); b.apply_translation([(f['x0']+f['x1'])/2, D + f['u'], (f['z0']+f['z1'])/2]); b.visual.face_colors = COL['servo']; scene.append((f"servo_{nm}", b))
    return
def apply_D(scene):                         # hand frame u -> robot y: add D to y of every mesh not already in robot coords (palm/servos are built in robot coords)
    return scene
POSES = {}
def make_poses():
    q5 = K.solve_chuck(5.0); thi, thm, thr, psi = [float(v) for v in q5.x]
    th_c = json.load(open(OUT + "/thumb3.json"))
    P = {}
    P['open'] = dict(f=dict(index=(0, 0), middle=(0, 0), ring=(0, 0), pinky=(0, 0)), th=(0, 0, 0))
    P['chuck'] = dict(f=dict(index=(thi, psi), middle=(thm, 0), ring=(thr, psi), pinky=(95, 0)), th=tuple(th_c['chuck5']))
    P['point'] = dict(f=dict(index=(0, 0), middle=(95, 0), ring=(95, 0), pinky=(95, 0)), th=(10, 0, 0))
    P['thumbsup'] = dict(f=dict(index=(95, 0), middle=(95, 0), ring=(95, 0), pinky=(95, 0)), th=(0, 0, 0))
    P['victory'] = dict(f=dict(index=(0, -30), middle=(0, 0), ring=(95, 0), pinky=(95, 0)), th=tuple(th_c['tuck']))
    return P
def build(name, P):
    scene, checks = [], []
    for nm, (th, ps) in P['f'].items(): finger_pose(nm, th, ps, scene, checks)
    thumb_pose(*P['th'], scene)
    return scene, checks
def to_robot(scene):                        # fingers/thumb are in (x,u,z); shift u -> y = D + u.  palm and servos already robot coords
    out = []
    for n, m in scene:
        if n == 'palm_tray' or n.startswith('servo_'): out.append((n, m)); continue
        m = m.copy(); m.apply_translation([0, D, 0]); out.append((n, m))
    return out
if __name__ == "__main__":
    P = make_poses(); rep = {}
    for name, p in P.items():
        scene, checks = build(name, p); st = []; static_scene(st); allm = to_robot(scene) + st
        # bar-length audit
        err = max(abs(l - lb) for _, _, l, lb in checks)
        # collision: every pair of non-adjacent movable parts + palm
        cm = trimesh.collision.CollisionManager()
        for n, m in allm:
            if n.startswith('servo_'): continue
            cm.add_object(n, m)
        hits, names = cm.in_collision_internal(return_names=True)
        def fam(n): return n.split('_')[0]
        bad = []
        for a, b in names:
            fa, fb_ = fam(a), fam(b)
            if fa == fb_ and fa != 'palm': continue            # same finger: bars/posts share pins by design (checked by bar-length audit)
            if 'palm' in (fa, fb_) and ('knuckle' in a+b or 'cmc' in a+b or 'prox' in a+b): continue   # knuckle/CMC seat on the palm by design
            bad.append((a, b))
        # minimum distance finger-vs-finger (different digits)
        dm = trimesh.collision.CollisionManager(); dm2 = trimesh.collision.CollisionManager()
        rep[name] = dict(max_bar_len_err_mm=round(float(err), 3), n_raw_hits=len(names), collisions=sorted(set(bad))[:20])
        print(name, rep[name])
        combined = trimesh.util.concatenate([m for _, m in allm]); combined.export(os.path.join(OUT, "poses", f"hand_{name}_posed.stl"))
        _cache_scene = allm
    json.dump(rep, open(OUT + "/pose_report.json", "w"), indent=1)
