"""4 renders of the MJ11 lead channels with the body (SPEC_j1r2_mj11_leads_shear s7).  usage: J1R2_ROOT=<root> python3 mj11_leads_render.py -> renders_j1r2/mj11_leads/*.png"""
import os, sys, json, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import clib as L, mj_frame as M, mj_concepts3 as K, mj11_leads as ML, mj_render as R
OUTD = os.path.join(M.REPO, "renders_j1r2", "mj11_leads"); os.makedirs(OUTD, exist_ok=True); CA = os.path.join(R.MD, "_render_cache_leads"); os.makedirs(CA, exist_ok=True)
MD = os.path.join(R.MD, "MJ11")
def save(m, n): p = os.path.join(CA, n + ".stl"); m.export(p); return p
def clip_body():
    b = L.body(); c = b.triangles_center; keep = (c[:, 2] > -330) & (c[:, 2] < -80) & (c[:, 1] > -200) & (c[:, 1] < 60) & (c[:, 0] > -200)
    return b.submesh([np.where(keep)[0]], append=True)
def electrodes(res):
    ATL = ML.ATL["sites"]; out = {"free": [], "rigid": [], "liner": []}
    for n, s in res["sites"].items():
        under = res["sites"][n]["under_rigid"]; cat = "rigid" if "rail" in under else ("liner" if under else "free")
        nrm = np.array(ATL[n]["n"], float); nrm /= np.linalg.norm(nrm)
        for e in (ATL[n]["a"], ATL[n]["b"]):
            d = trimesh.creation.cylinder(6.0, 1.6, sections=24); T = trimesh.geometry.align_vectors([0, 0, 1], nrm); T[:3, 3] = np.array(e) + nrm * 0.6; d.apply_transform(T); out[cat].append(d)
    return {k: trimesh.util.concatenate(v) for k, v in out.items() if v}
def parts(open_view):
    res = json.load(open(os.path.join(MD, "leads.json"))); g = ML.lead_geo(res); el = electrodes(res)
    P = [dict(stl=save(clip_body(), "body"), color=[214, 182, 160], rough=0.8)]
    w = trimesh.load(os.path.join(R.ST, "shoulder", "scapular", "wing_R.stl")); w = L.I(w, L.box(-300, 100, -150, -100.0, -500, 100))
    if open_view: w = trimesh.util.concatenate([L.I(w, L.box(-300, 100, -150, -100.0, -500, -265)), L.I(w, L.box(-300, 100, -150, -100.0, -118, 100))])     # rail hidden over the electrode zone so the discs show
    P.append(dict(stl=save(w, "rail"), color=[70, 74, 82], rough=0.6))
    if open_view:
        mk = []
        for psi in K.LEAD_PSI:
            ax = np.array([np.cos(np.radians(psi)), np.sin(np.radians(psi))]); c = np.array([K.Y1, K.Z1]) + 49 * ax; sx = float(M.back_x(c[0], c[1]))
            b = trimesh.creation.box([0.8, 8.0, 62.0]); T = trimesh.transformations.rotation_matrix(np.arctan2(ax[1], ax[0]) - np.pi / 2, [1, 0, 0]); b.apply_transform(T); b.apply_translation([sx + 1.0, c[0], c[1]]); mk.append(b)
        P.append(dict(stl=save(trimesh.util.concatenate(mk), "chan"), color=[0, 190, 230], rough=0.4))
        ring = trimesh.util.concatenate([trimesh.creation.cylinder(1.0, 3.0, transform=trimesh.transformations.translation_matrix([float(M.back_x(K.Y1 + 66 * np.cos(t), K.Z1 + 66 * np.sin(t))) + 1.0, K.Y1 + 66 * np.cos(t), K.Z1 + 66 * np.sin(t)])) for t in np.linspace(0, 2 * np.pi, 120) if K.Y1 + 66 * np.cos(t) >= K.RAIL_EDGE_Y])
        P.append(dict(stl=save(ring, "ring"), color=[40, 40, 40], rough=0.5))
    for k, mm in g.items(): P.append(dict(stl=save(mm, "ld_" + k), color=[255, 140, 20] if k != "jack_block" else [140, 205, 40], rough=0.4))
    for k, mm in el.items(): P.append(dict(stl=save(mm, "el_" + k), color={"free": [40, 170, 90], "rigid": [215, 40, 40], "liner": [240, 200, 30]}[k], rough=0.5))
    if not open_view:
        P.append(dict(stl=os.path.join(MD, "MJ11_liner_R.stl"), color=[225, 225, 232], rough=0.7))
        for k, c in (("seat_dish", [51, 87, 133]), ("pad", [51, 87, 133]), ("rail_spacers", [115, 120, 128])): P.append(dict(stl=os.path.join(MD, f"MJ11_{k}_R.stl"), color=c, rough=0.4))
    else: P.append(dict(stl=os.path.join(MD, "MJ11_rail_spacers_R.stl"), color=[115, 120, 128], rough=0.4))
    return P
def go(name, open_view, views, title):
    sc = dict(out_prefix=os.path.join(OUTD, name), title=title, width=900, height=900, parts=parts(open_view), up=[0, 0, 1], time_budget=50, fill=0.86, views=views)
    js = os.path.join(CA, name + ".json"); json.dump(sc, open(js, "w")); subprocess.run([sys.executable, os.path.join(M.REPO, "cad", "render_three.py"), js], check=True)
if __name__ == "__main__":
    go("A_assembled", False, [dict(name="rear34", dir=[1.0, 0.6, 0.45], label="rear-right 3/4"), dict(name="rear", dir=[1.0, -0.05, 0.1], label="from behind")], "MJ11 with leads: assembled (orange = leads, lime = jack block)")
    go("B_open", True, [dict(name="rear", dir=[1.0, -0.05, 0.1], label="cyan = channels, ring = liner edge"), dict(name="rear34", dir=[1.0, 0.5, 0.6], label="same, from above-lateral")],
       "Electrodes: green free, yellow under liner, red under rail")
