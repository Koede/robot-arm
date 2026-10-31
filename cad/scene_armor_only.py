"""Close-up of the armour parts alone on the body + chassis plate (no arm), for visual checks of the pauldron lames.
Usage: python3 scene_armor_only.py <out_prefix> [pauldron_dir] [title]"""
import os, sys, json, glob, subprocess, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__))
ST = os.path.join(HERE, "..", "stl"); SH = os.path.join(ST, "shoulder")
out = os.path.abspath(sys.argv[1]); pdir = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else os.path.join(SH, "pauldron")
title = sys.argv[3] if len(sys.argv) > 3 else "Third Shoulder: armour parts"
cache = os.path.join(os.path.dirname(out), "_scene_cache_armor"); os.makedirs(cache, exist_ok=True)
parts = []
def add(p, color, metal=0.0, rough=0.55):
    if not isinstance(p, str):
        q = os.path.join(cache, "m%d.stl" % len(parts)); p.export(q); p = q
    parts.append(dict(stl=os.path.abspath(p), color=color, metal=metal, rough=rough))
body = trimesh.load(os.path.join(SH, "proxies", "wearer_makehuman.stl")); c = body.triangles_center
keep = (c[:, 2] > -260) & (c[:, 2] < 40) & (c[:, 1] > -200) & (c[:, 1] < 110) & (c[:, 0] > -300)
add(body.submesh([np.where(keep)[0]], append=True), [214, 182, 160], rough=0.8)
for p_, col in ((os.path.join(SH, "clasp_frame_R.stl"), [70, 74, 82]), (os.path.join(SH, "chassis_plate_R.stl"), [168, 172, 178])):
    if os.path.exists(p_): add(p_, col, metal=0.6 if "plate" in p_ else 0.0)
ARMOR = {"crown": [150, 158, 172], "keel": [120, 128, 142], "spaulder": [176, 184, 196]}
for k, col in ARMOR.items():
    p_ = os.path.join(pdir, f"pauldron_{k}_R.stl")
    if os.path.exists(p_): add(p_, col, metal=0.8, rough=0.3)
_rv = os.path.join(SH, "proxies", "pauldron_rivets_R.stl")
if os.path.exists(_rv): add(_rv, [40, 40, 44], metal=0.6, rough=0.35)
scene = dict(out_prefix=out, title=title, width=900, height=900, parts=parts, up=[0, 0, 1], time_budget=int(os.environ.get("TB", 60)), fill=0.9,
             views=[dict(name="top", dir=[0.15, 0.1, 1.0], label="from above"),
                    dict(name="front34", dir=[-1.0, 0.8, 0.6], label="front-right 3/4"),
                    dict(name="side", dir=[0.05, 1.0, 0.12], label="from the right"),
                    dict(name="rear34", dir=[1.0, 0.75, 0.55], label="rear-right 3/4")])
json.dump(scene, open(out + ".json", "w"))
subprocess.run([sys.executable, os.path.join(HERE, "render_three.py"), out + ".json"], check=True)
