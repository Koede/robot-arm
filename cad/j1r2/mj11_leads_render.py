"""Renders of the MJ11 lead channels with the body (2026-11-03).  usage: J1R2_ROOT=<root> python3 mj11_leads_render.py -> renders_j1r2/mounts/MJ11_leads_*.png
Lead drawing: 3 mm flat tails drawn as D3 tubes (exaggerated); the infraspinatus pair is routed (accordion -> psi-225 channel -> exit -> dock); teres major is NOT routed (infeasible, see SPEC) and drawn as electrodes + path ghosts only."""
import os, sys, json, math, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import clib as L, mj_frame as M, mj_concepts as C, mj_concepts3 as K
import mj_render as R
REPO, ST = M.REPO, os.path.join(M.REPO, "stl"); MD = R.MD; CACHE = R.CACHE; OUTD = R.OUTD
ATL = json.load(open(os.path.join(ST, "shoulder", "emg_atlas.json")))["sites"]; LD = json.load(open(os.path.join(MD, "MJ11", "leads.json")))
Cc = np.array([K.Y1, K.Z1])
def xs(y, z): return M.back_x(y, z)
def tube(pts, r=1.4):
    ms = [trimesh.creation.cylinder(radius=r, segment=[pts[i], pts[i + 1]], sections=10) for i in range(len(pts) - 1)] + [trimesh.creation.icosphere(2, r) .apply_translation(p) or trimesh.creation.icosphere(2, r) for p in []]
    return trimesh.util.concatenate(ms)
def disc(p2, r=6.0, h=1.6, dx=0.0):
    m = trimesh.creation.cylinder(radius=r, height=h, sections=32); m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])); m.apply_translation([xs(*p2) + dx + 1.0, p2[0], p2[1]]); return m
def pt(y, z, dx=1.8): return np.array([xs(y, z) + dx, y, z])
def accordion(e, c, n_half=3, amp=12.0, extra=0.5, k=24):
    e, c = np.array(e, float), np.array(c, float); ax = c - e; Lx = np.linalg.norm(ax); u = ax / Lx; v = np.array([-u[1], u[0]]); pts = []
    for s in np.linspace(0, 1, n_half * k + 1):
        env = math.sin(math.pi * s) ** 0.5 if 0 < s < 1 else 0.0; pts.append(e + u * s * Lx + v * amp * env * math.sin(math.pi * n_half * s))
    return pts
def leads_geo():
    parts = {}
    # exit of the psi-225 channel at the y = -96 cut, then the dock and the run to the spine jack
    a = math.radians(225.0); r_in = 18.0; inlet = Cc + r_in * np.array([math.cos(a), math.sin(a)]); ex_r = (K.RAIL_EDGE_Y - Cc[0]) / math.cos(a); exit_ = Cc + ex_r * np.array([math.cos(a), math.sin(a)])
    jack = np.array([-146.0, exit_[1]])
    lead_meshes = []; el_meshes = []; ghost = []
    for off, e in zip((-1.8, 1.8), ("a", "b")):
        p = np.array(ATL["infraspin"][e][1:3]); el_meshes.append(disc(p))
        tr = ATL["infraspin"]["travel_mm"]; pl = LD["electrodes"][f"infraspin.{e}"]
        for lab in ("end_plus", "end_minus"): ghost.append(disc(np.array(pl[lab]), dx=0.0))
        side = np.array([math.cos(a + math.pi / 2), math.sin(a + math.pi / 2)]) * off
        path2 = accordion(p, inlet + side) + [inlet + side, exit_ + side, jack + side]
        lead_meshes.append(tube([pt(*q) for q in path2]))
    parts["leads_infra"] = dict(mesh=trimesh.util.concatenate(lead_meshes), col=[235, 190, 30], metal=0.1, rough=0.4)
    parts["electrodes_infra"] = dict(mesh=trimesh.util.concatenate(el_meshes), col=[40, 170, 90], metal=0.1, rough=0.5)
    parts["electrodes_infra_ghost"] = dict(mesh=trimesh.util.concatenate(ghost), col=[150, 210, 170], metal=0.0, rough=0.7)
    tm, tg = [], []
    for e in ("a", "b"):
        p = np.array(ATL["teres_maj"][e][1:3]); tm.append(disc(p)); pl = LD["electrodes"][f"teres_maj.{e}"]
        for lab in ("end_plus", "end_minus"): tg.append(disc(np.array(pl[lab])))
    parts["electrodes_teres"] = dict(mesh=trimesh.util.concatenate(tm), col=[220, 90, 60], metal=0.1, rough=0.5)
    parts["electrodes_teres_ghost"] = dict(mesh=trimesh.util.concatenate(tg), col=[235, 170, 150], metal=0.0, rough=0.7)
    d = trimesh.creation.box(extents=[8.0, 14.0, 26.0]); d.apply_translation([xs(exit_[0] - 5, exit_[1]) + 4.0, exit_[0] - 5.0, exit_[1]]); parts["lead_dock"] = dict(mesh=d, col=[200, 90, 40], metal=0.1, rough=0.5)
    return parts
def crop_body():
    b = L.body(); c = b.triangles_center; keep = (c[:, 2] > -300) & (c[:, 2] < -95) & (c[:, 1] > -175) & (c[:, 1] < 40) & (c[:, 0] > -175)
    m = b.submesh([np.where(keep)[0]], append=True); p = os.path.join(CACHE, "lead_body.stl"); m.export(p); return p
def sheet():
    """liner skin-face sheet (1 mm) with the trenches: the foam liner itself would hide the leads, so the renders show only its skin face."""
    Y, Z, skin, inner, plane = K.seat_surfaces(); sh = L.I(L.height_solid(Y, Z, skin, skin + 1.0, "x"), K._domain())
    cuts = [K.boxbeam(np.array([K.X0 - 40.0, a[0], a[1]]), np.array([K.X0 - 40.0, b[0], b[1]]), K.LEAD_W, 60.0, None, up=(1, 0, 0), ext=0.0) for a, b in K.trench_segments()]
    sh = K.DIFF(sh, *cuts); p = os.path.join(CACHE, "lead_liner_sheet.stl"); sh.export(p); return p
def main():
    R.shared(); G = leads_geo(); P = []; body = crop_body(); P.append(dict(stl=body, color=[214, 182, 160], rough=0.8))
    P.append(dict(stl=os.path.join(CACHE, "wing_rail.stl"), color=[70, 74, 82], rough=0.6))
    for k, g in G.items():
        p = os.path.join(CACHE, f"lead_{k}.stl"); g["mesh"].export(p); P.append(dict(stl=p, color=g["col"], metal=g["metal"], rough=g["rough"]))
    P.append(dict(stl=sheet(), color=[235, 235, 240], rough=0.7)); P.append(dict(stl=os.path.join(MD, "MJ11", "MJ11_rail_spacers_R.stl"), color=[120, 124, 130], rough=0.7))
    wr = trimesh.load(os.path.join(CACHE, "wing_rail.stl")); wr = L.I(wr, L.box(-300, 100, -300, 100, -300, -95)); wr.export(os.path.join(CACHE, "lead_rail.stl")); P[1] = dict(stl=os.path.join(CACHE, "lead_rail.stl"), color=[70, 74, 82], rough=0.6)
    full = list(R.shared()) + [dict(stl=os.path.join(MD, "MJ11", f"MJ11_{k}_R.stl"), color=c, rough=0.5) for k, c in (("pad", [52, 86, 132]), ("seat_dish", [52, 86, 132]), ("liner", [235, 235, 240]), ("rail_spacers", [120, 124, 130]), ("clasp_bracket", [52, 86, 132]))]
    full += [dict(stl=os.path.join(CACHE, f"lead_{k}.stl"), color=g["col"], rough=0.5) for k, g in G.items() if k in ("leads_infra", "lead_dock")]
    sA = dict(out_prefix=os.path.join(OUTD, "MJ11_leads_ctx"), title="MJ11 lead layout: context (pad, dish, rail, body)", width=900, height=900, parts=full, up=[0, 0, 1], time_budget=50, fill=0.86, views=[dict(name="rear34", dir=[1.0, 0.7, 0.5])])
    views = [dict(name="rear", dir=[1.0, 0.02, 0.05]), dict(name="oblique", dir=[0.8, -0.7, 0.35]), dict(name="low", dir=[0.7, 0.45, -0.75])]
    sB = dict(out_prefix=os.path.join(OUTD, "MJ11_leads"), title="MJ11 lead channels: dish and pad removed. Yellow = infraspinatus leads (accordion, psi-225 channel, exit, dock). Green = electrodes, pale = their position at full elevation; orange = teres major (not routable). Liner shown as its 1 mm skin face; leads and electrodes drawn on top of it for visibility", width=900, height=900, parts=P, up=[0, 0, 1], time_budget=50, fill=0.9, views=views)
    for n, s in (("ctx", sA), ("main", sB)):
        js = os.path.join(CACHE, f"scene_leads_{n}.json"); json.dump(s, open(js, "w")); subprocess.run([sys.executable, os.path.join(REPO, "cad", "render_three.py"), js], check=True)
main()
