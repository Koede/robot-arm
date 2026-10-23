"""Build a wearer keep-out mesh from MakeHuman's CC0 base mesh (makehumancommunity/makehuman,
makehuman/data/3dobjs/base.obj, released CC0 Sept 2020; file header says so).
Mesh units are decimetres, +x = character's left, +y = up, +z = forward, A-pose (arms ~40 deg from vertical).
Output frame = robot frame (right-handed): +x rearward, +y wearer's RIGHT, +z up, origin at the RIGHT shoulder centre P,
units mm. The arms are re-posed to hang at the sides (smooth rotation about each glenohumeral helper joint).
Scale: the mesh is an average adult (height shown in landmarks.json). Pass --height to rescale."""
import numpy as np, trimesh, json, os, sys, argparse

ap = argparse.ArgumentParser()
ap.add_argument("--obj", default="/home/claude/external/makehuman_base/base.obj")
ap.add_argument("--height_mm", type=float, default=None, help="rescale whole body to this standing height")
ap.add_argument("--p_lateral", type=float, default=150.0, help="P distance from the spine plane (mm)")
ap.add_argument("--p_above_acromion", type=float, default=100.0)
ap.add_argument("--arm_hang_deg", type=float, default=10.0, help="abduction of the re-posed hanging arm (deg)")
ap.add_argument("--back_to_p", type=float, default=118.0, help="P distance behind the upper-back surface (mm)")
a = ap.parse_args()

V, F, groups, cur, joints = [], [], {}, None, {}
for line in open(a.obj):
    if line.startswith("v "):
        V.append([float(x) for x in line.split()[1:4]])
    elif line.startswith("g "):
        cur = line.split()[1]
    elif line.startswith("f ") and cur == "body":
        idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
        for k in range(1, len(idx) - 1):
            F.append([idx[0], idx[k], idx[k + 1]])
    elif line.startswith("f ") and cur and cur.startswith("joint-"):
        joints.setdefault(cur, set()).update(int(t.split("/")[0]) - 1 for t in line.split()[1:])
V = np.array(V) * 100.0            # dm -> mm
jc = {k: V[sorted(s)].mean(0) for k, s in joints.items()}
used = np.unique(np.array(F))
remap = -np.ones(len(V), int); remap[used] = np.arange(len(used))
mesh_v = V[used].copy(); mesh_f = remap[np.array(F)]

# --- re-pose arms to hang.  Rev 2026-10-13: the GitHub base.obj is an A-POSE (upper arm ~40 deg from vertical, feet apart),
# not the T-pose the first version assumed. The old code rotated everything lateral of the shoulder joint by 90 deg, which threw
# the feet and lower legs out sideways (left) or into the torso (right) and swung the right arm across the chest.
# Now: measure the arm angle in the frontal plane, rotate only arm vertices (angular weight about the shoulder joint, so legs,
# torso, acromion and neck stay put) down to a relaxed hang of ARM_HANG deg abduction.
def smooth(u):
    u = np.clip(u, 0, 1); return u * u * (3 - 2 * u)
ARM_HANG = a.arm_hang_deg
repose = {}
for side, sgn in (("l", 1), ("r", -1)):
    s = jc[f"joint-{side}-shoulder"]
    d = mesh_v - s
    arm = (sgn * mesh_v[:, 0] > abs(s[0]) + 80) & (mesh_v[:, 1] > s[1] - 450)
    da = d[arm]; theta_a = float(np.degrees(np.arctan2(np.median(sgn * da[:, 0]), np.median(-da[:, 1]))))
    phi = np.degrees(np.arctan2(sgn * d[:, 0], -d[:, 1]))                  # 0 = straight down, 90 = lateral, 180 = up
    b = 0.5 * theta_a
    w = smooth((phi - (b - 8.0)) / 16.0) * (1 - smooth((phi - 95.0) / 40.0)) * smooth((sgn * mesh_v[:, 0] - 110.0) / 40.0)
    ang = -sgn * np.radians(theta_a - ARM_HANG) * w
    c, sn = np.cos(ang), np.sin(ang)
    x = d[:, 0] * c - d[:, 1] * sn
    y = d[:, 0] * sn + d[:, 1] * c
    mesh_v[:, 0] = s[0] + x; mesh_v[:, 1] = s[1] + y
    repose[side] = round(theta_a, 1)

# --- mesh frame -> robot frame:  X = -z, Y = -x (mesh right = -x), Z = y   (proper rotation, det +1)
def to_robot(p):
    p = np.atleast_2d(p); return np.c_[-p[:, 2], -p[:, 0], p[:, 1]]
R = to_robot(mesh_v)
ground = to_robot(jc["joint-ground"])[0]
height = R[:, 2].max() - ground[2]
if a.height_mm:
    k = a.height_mm / height
    R = (R - ground) * k + ground; height *= k
else:
    k = 1.0

# --- landmarks (robot frame, before placing P)
shoulder_zone = (R[:, 1] > 100) & (R[:, 1] < 260) & (R[:, 2] > ground[2] + 0.75 * height)
acromion = R[shoulder_zone][np.argmax(R[shoulder_zone][:, 2])]
trunk = (np.abs(R[:, 1]) < 170) & (R[:, 2] > acromion[2] - 220) & (R[:, 2] < acromion[2] - 20)
back_x = R[trunk][:, 0].max()
P = np.array([back_x + a.back_to_p, a.p_lateral, acromion[2] + a.p_above_acromion])
R = R - P
mesh = trimesh.Trimesh(R, mesh_f, process=False)
mesh.fix_normals()
# orientation check (rev 2026-10-13: the old head-face heuristic picked the wrong sign on the corrected mesh): cast a ray from
# behind onto the back at the spine and require the hit face normal to point rearward (+x); flip otherwise.
_o = np.array([R[:, 0].max() + 50.0, -P[1], -150.0])          # behind the spine (spine plane y = -P_y in the P frame)
_d = np.array([-1.0, 0.0, 0.0]); _T = mesh.triangles; _v0 = _T[:, 0]; _e1 = _T[:, 1] - _v0; _e2 = _T[:, 2] - _v0
_p = np.cross(_d, _e2); _det = np.einsum("ij,ij->i", _e1, _p); _ok = np.abs(_det) > 1e-12; _inv = np.where(_ok, 1 / np.where(_ok, _det, 1), 0)
_s = _o - _v0; _u = np.einsum("ij,ij->i", _s, _p) * _inv; _q = np.cross(_s, _e1); _vv = (_q @ _d) * _inv; _t = np.einsum("ij,ij->i", _e2, _q) * _inv
_h = _ok & (_u >= 0) & (_vv >= 0) & (_u + _vv <= 1) & (_t > 0)
_i = np.where(_h)[0][np.argmin(_t[_h])]
if mesh.face_normals[_i][0] < 0:
    mesh.invert()
assert mesh.face_normals[_i][0] > 0, "wearer mesh must face outward"
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "shoulder", "proxies")
os.makedirs(out, exist_ok=True)
mesh.export(os.path.join(out, "wearer_makehuman.stl"))
lm = dict(height_mm=round(float(height), 0), scale=round(float(k), 3),
          acromion_top_rel_P=(acromion - P).round(0).tolist(),
          gh_joint_rel_P=(to_robot(jc["joint-r-shoulder"])[0] * k + (1 - k) * ground - P).round(0).tolist(),
          neck_joint_rel_P=(to_robot(jc["joint-neck"])[0] * k + (1 - k) * ground - P).round(0).tolist(),
          head_joint_rel_P=(to_robot(jc["joint-head"])[0] * k + (1 - k) * ground - P).round(0).tolist(),
          head_top_z_rel_P=round(float(R[:, 2].max()), 0),
          back_surface_x_rel_P=round(float(back_x - P[0]), 0),
          spine_y_rel_P=round(float(-P[1]), 0),
          watertight=bool(mesh.is_watertight), faces=int(len(mesh.faces)),
          a_pose_arm_deg=repose, arm_hang_deg=ARM_HANG)
json.dump(lm, open(os.path.join(out, "wearer_landmarks.json"), "w"), indent=1)
print(json.dumps(lm, indent=1))
