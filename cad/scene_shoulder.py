"""Scene builder for shoulder-region renders with render_three.py (rev 2026-10-30, armor pauldron session).
Usage: python3 scene_shoulder.py <out_prefix> [pauldron_dir] [title]
Poses the shoulder groups (F, M1, M2, M3 + balancer) at park and writes <out_prefix>.json, then renders it.
pauldron_dir defaults to ../stl/shoulder/pauldron (every pauldron*_R.stl / lame*_R.stl in it except the combined pauldron_R.stl)."""
import os, sys, json, glob, subprocess, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak
ST = os.path.join(HERE, "..", "stl"); SH = os.path.join(ST, "shoulder")
out = os.path.abspath(sys.argv[1]); pdir = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else os.path.join(SH, "pauldron")
title = sys.argv[3] if len(sys.argv) > 3 else "Third Shoulder: shoulder mount"
Q = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)
T = ak.frames(Q)
cache = os.path.join(os.path.dirname(out), "_scene_cache"); os.makedirs(cache, exist_ok=True)
parts = []
def add(mesh_or_path, color, metal=0.0, rough=0.55, M=None, name=None):
    if isinstance(mesh_or_path, str): p = mesh_or_path
    else:
        p = os.path.join(cache, (name or "m%d" % len(parts)) + ".stl"); mesh_or_path.export(p)
    d = dict(stl=os.path.abspath(p), color=color, metal=metal, rough=rough)
    if M is not None: d["matrix"] = np.asarray(M).tolist()
    parts.append(d)
# wearer: crop to the upper torso / right shoulder / neck so the camera frames the shoulder
body = trimesh.load(os.path.join(SH, "proxies", "wearer_makehuman.stl")); c = body.triangles_center
CLOSE = os.environ.get("CLOSE", "0") == "1"
keep = (c[:, 2] > -430) & (c[:, 2] < 140) & (c[:, 1] > -330) & (c[:, 1] < 160)
if CLOSE: keep = (c[:, 2] > -300) & (c[:, 2] < 60) & (c[:, 1] > -240) & (c[:, 1] < 130) & (c[:, 0] > -330)
add(body.submesh([np.where(keep)[0]], append=True), [214, 182, 160], rough=0.8, name="body")
HARN = [214 // 1, 120, 70]
for p_, col in ((os.path.join(SH, "clasp_frame_R.stl"), [70, 74, 82]), (os.path.join(SH, "scapular", "wing_R.stl"), [70, 74, 82]),
                (os.path.join(SH, "scapular", "spine_bridge.stl"), [70, 74, 82]), (os.path.join(SH, "chassis_plate_R.stl"), [168, 172, 178])):
    if os.path.exists(p_): add(p_, col, metal=0.6 if "plate" in p_ else 0.0, rough=0.35 if "plate" in p_ else 0.6)
G = ak.groups(include_fingers=False)
if ak.has_balancer(): ak.install_balancer(G)
GREY, DARK = [190, 194, 200], [52, 54, 60]
for g in (("F", "M1", "M2") if CLOSE else ("F", "M1", "M2", "M3")):
    for k, m in G[g].items():
        col = DARK if k.startswith(("ak_", "xm_")) else ([200, 120, 60] if k.startswith("tpu") else GREY)
        if k == "cowl" or k == "head": col = [120, 126, 136]
        add(m, col, metal=0.2 if col == GREY else 0.0, M=T[g], name=f"{g}_{k}")
if ak.has_balancer():
    BF = ak.balancer_frames(Q)
    for nm, (m, grp) in ak.balancer_parts().items():
        if nm in BF and not (CLOSE and grp == "M2"): add(m, [150, 156, 166], M=BF[nm], name="bal_" + nm)
ARMOR = [150, 158, 172]
for p_ in sorted(glob.glob(os.path.join(pdir, "*_R.stl"))):
    b = os.path.basename(p_)
    if b == "pauldron_R.stl": continue
    col = [176, 120, 64] if ("liner" in b or "leather" in b) else ([110, 114, 122] if "rivet" in b else ARMOR)
    add(p_, col, metal=0.85 if col == ARMOR else 0.3, rough=0.3 if col == ARMOR else 0.5)
_rv = os.path.join(SH, "proxies", "pauldron_rivets_R.stl")
if os.path.exists(_rv) and "archive" not in pdir: add(_rv, [40, 40, 44], metal=0.6, rough=0.35)
scene = dict(out_prefix=out, title=title, width=900, height=900, parts=parts, up=[0, 0, 1], time_budget=int(os.environ.get("TB", 90)), fill=0.82,
             views=[dict(name="rear34", dir=[1.0, 0.75, 0.55], label="rear-right 3/4"),
                    dict(name="side", dir=[0.05, 1.0, 0.12], label="from the right"),
                    dict(name="front34", dir=[-1.0, 0.8, 0.6], label="front-right 3/4"),
                    dict(name="top", dir=[0.25, 0.15, 1.0], label="from above"),
                    dict(name="rear", dir=[1.0, -0.05, 0.25], label="from behind"),
                    dict(name="low", dir=[0.9, 0.9, -0.35], label="from below-rear")])
json.dump(scene, open(out + ".json", "w"))
subprocess.run([sys.executable, os.path.join(HERE, "render_three.py"), out + ".json"], check=True)
