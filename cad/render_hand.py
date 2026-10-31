import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import hand_pose as HP, hand_kin as K
OUTD = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."); os.makedirs(OUTD, exist_ok=True); D = 57.0
def scene_for(name, P):
    scene, _ = HP.build(name, P); st = []; HP.static_scene(st); allm = HP.to_robot(scene) + st
    ms = []
    if name == 'chuck':
        rod = trimesh.creation.cylinder(radius=5.0, height=140, sections=24); rod.apply_translation([0, D, 640]); rod.visual.face_colors = [225, 225, 235, 255]; allm.append(('bit', rod))
    for n, m in allm:
        base = np.array(m.visual.face_colors[0], dtype=float) if len(m.visual.face_colors) else np.array([180, 180, 180, 255.0])
        m = m.copy()
        if len(m.faces) > 1800:
            try: m = m.simplify_quadric_decimation(face_count=1800)
            except Exception: pass
        ms.append((n, m, np.tile(base, (len(m.faces), 1))))
    return ms
def rotx(ms, deg, pivot):
    a = np.radians(deg); R = np.eye(4); R[1, 1] = np.cos(a); R[1, 2] = -np.sin(a); R[2, 1] = np.sin(a); R[2, 2] = np.cos(a)
    T1 = np.eye(4); T1[:3, 3] = -pivot; T2 = np.eye(4); T2[:3, 3] = pivot; M = T2 @ R @ T1
    out = []
    for n, m, c in ms: m2 = m.copy(); m2.apply_transform(M); out.append((n, m2, c))
    return out
def draw(ax, ms, elev, azim, title, lim):
    ax.set_proj_type('ortho'); ax.view_init(elev=elev, azim=azim)
    e = np.radians(elev); a = np.radians(azim); cam = np.array([np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)]); light = np.array([0.4, -0.3, 0.9]); light /= np.linalg.norm(light)
    polys, cols, depth = [], [], []
    for n, m, c in ms:
        tri = m.triangles; nrm = m.face_normals; vis = nrm @ cam > -0.2
        sh = 0.45 + 0.55*np.clip(np.abs(nrm @ cam)*0.6 + np.clip(nrm @ light, 0, 1)*0.5, 0, 1)
        for i in np.where(vis)[0]:
            polys.append(tri[i]); cc = c[i][:3]/255.0*sh[i]; cols.append((*np.clip(cc, 0, 1), 1.0)); depth.append(tri[i].mean(0) @ cam)
    order = np.argsort(depth); pc = Poly3DCollection([polys[i] for i in order], facecolors=[cols[i] for i in order], edgecolors='none', linewidths=0)
    ax.add_collection3d(pc); c0 = np.array(lim['c']); r = lim['r']
    ax.set_xlim(c0[0]-r, c0[0]+r); ax.set_ylim(c0[1]-r, c0[1]+r); ax.set_zlim(c0[2]-r, c0[2]+r); ax.set_box_aspect((1, 1, 1), zoom=1.55); ax.set_axis_off(); ax.set_title(title, fontsize=10)
LABEL = dict(open='Open hand (human layout)', chuck='CHUCK mode (index + middle + ring + thumb on a Ø10 mm bit)', point='Pointing  (index extended)', thumbsup='Thumbs up', victory='Victory / peace')
if __name__ == "__main__":
    P = HP.make_poses(); files = []
    for name, p in P.items():
        ms = scene_for(name, p)
        if name in ('point', 'thumbsup'): ms = rotx(ms, 90, np.array([-20.0, D, 540.0]))           # thumb up, fingers horizontal for the emoji read
        allv = np.vstack([m.vertices for n_, m, _ in ms if m.vertices[:,2].min() > 440 or True]); allv = allv[allv[:,2] > 482]; c = (allv.max(0) + allv.min(0))/2; r = (allv.max(0) - allv.min(0)).max()/2*0.9
        fig = plt.figure(figsize=(15, 5.6)); fig.patch.set_facecolor('white')
        views = [(8, 0, 'palm side'), (8, 90, 'thumb side'), (22, 40, '3/4 view')]
        if name in ('point', 'thumbsup'): views = [(8, 0, 'palm side'), (8, 180, 'back of hand'), (22, 40, '3/4 view')]
        for k, (el, az, t) in enumerate(views):
            ax = fig.add_subplot(1, 3, k+1, projection='3d'); draw(ax, ms, el, az, t, dict(c=c, r=r))
        fig.suptitle(LABEL[name], fontsize=13, y=0.98); fig.tight_layout(); f = f"{OUTD}/hand_{name}.png"; fig.savefig(f, dpi=110); plt.close(fig); files.append(f); print(f, flush=True)
