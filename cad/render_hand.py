"""Rev B renders: builds render_three.py scenes from stl/hand/scene_parts.json -> hand_<pose>_sheet.png (+ per-view PNGs) at the repo root."""
import json, os, sys, subprocess, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as K
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..")); OUT = K.OUT
PL = json.load(open(OUT + "/poses.json")); K.RHO_T = PL['RHO_T']
SC = dict(json.load(open(OUT + "/scene_parts.json")))
TITLE = dict(open='Open hand (human layout)', chuck4_d13='Chuck mode: 4 jaws (index, middle, ring, thumb) on a Ø13 mm bit',
             chuck3_d6='Chuck mode: 3 jaws (thumb, index, middle) on a Ø6 mm bit', point='Pointing (index extended, thumb on the middle finger)', thumbsup='Thumbs up', victory='Victory (index spread 15 deg, thumb holds ring + pinky)')
def views(name):
    if name == 'open': return [0, 0, 1], [("palm", [1, -0.15, 0.12], "palm"), ("side", [0.25, 1, 0.2], "thumb side"), ("back", [-1, 0.25, 0.25], "back"), ("iso", [0.9, 0.7, 0.5], "3/4")]
    if name.startswith('chuck'): return [0, 0, 1], [("axial", [1, 0.05, 0.08], "looking down the bit"), ("side", [0.3, 1, 0.25], "thumb side"), ("iso", [0.85, 0.6, 0.5], "3/4"), ("under", [0.7, -0.7, -0.2], "from below")]
    if name == 'point': return [0, 1, 0], [("emoji", [-1, 0, 0.0], "back of hand"), ("palm", [1, 0, 0.0], "palm side"), ("iso", [-0.8, 0.45, 0.6], "3/4")]
    if name == 'thumbsup':
        th = K.thumb(*PL['poses']['thumbsup']['th']); up = th['P3'] - th['P0']; up = (up/np.linalg.norm(up)).tolist()
        return up, [("emoji", [1, -0.2, 0.0], "palm side"), ("back", [-1, 0.0, 0.1], "back"), ("iso", [0.7, -0.7, 0.4], "3/4")]
    return [0, 0, 1], [("emoji", [1, -0.1, 0.1], "palm side"), ("back", [-1, 0.1, 0.15], "back"), ("iso", [0.85, 0.5, 0.4], "3/4")]
if __name__ == "__main__":
    names = sys.argv[1:] or list(SC)
    for name in names:
        up, vw = views(name)
        scene = dict(out_prefix=os.path.join(ROOT, f"hand_{name}"), title=TITLE[name], width=1000, height=1000, parts=SC[name], up=up,
                     views=[dict(name=n, dir=d, label=l) for n, d, l in vw], time_budget=44)
        f = f"/tmp/scene_{name}.json"; json.dump(scene, open(f, "w"))
        r = subprocess.run([sys.executable, os.path.join(HERE, "render_three.py"), f], capture_output=True, text=True); print(name, r.returncode, r.stdout[-200:], r.stderr[-400:], flush=True)
