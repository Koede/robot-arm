"""Plan view (y, z; looking at the back, +y to the right) of the MJ11 liner relief slots, electrode sweeps, lead paths, jack plate, rail, spacers (2026-11-03).
usage: python3 mj11_leads_plan.py -> renders_j1r2/mounts/MJ11_leads_plan.png"""
import os, json, math, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP, Circle, Rectangle
import mj11_leads as LD
REPO = LD.REPO; led = json.load(open(os.path.join(REPO, "stl", "j1r2_mounts", "MJ11", "leads.json")))
fig, ax = plt.subplots(figsize=(9, 9), dpi=130); ax.set_aspect("equal")
ax.add_patch(Rectangle((-138, -440), 38, 360, fc="#c9ced6", ec="#555", lw=0.8, label="paraspinal rail y -138..-100"))
dom = LD.domain_poly(); ax.add_patch(MP(np.array(dom.exterior.coords), fc="#eef0f4", ec="#223", lw=1.2, label="liner footprint (R66, cut y -96)"))
for psi in LD.OLD_CH_PSI:
    a = math.radians(psi); ax.plot([LD.Y1 + 18 * math.cos(a), LD.Y1 + 66 * math.cos(a)], [LD.Z1 + 18 * math.sin(a), LD.Z1 + 66 * math.sin(a)], color="#999", ls="--", lw=1)
ax.plot([], [], color="#999", ls="--", label="rev 2026-11-02 radial channels (retired)")
for k, p in LD.slot_polys().items():
    for g in (p.geoms if p.geom_type == "MultiPolygon" else [p]): ax.add_patch(MP(np.array(g.exterior.coords), fc="#f2b632", ec="#a06d00", alpha=0.75))
ax.add_patch(MP([(0, 0)], fc="#f2b632", ec="#a06d00", alpha=0.75, label="relief slot, 5.5 deep from the skin face"))
cols = dict(infraspin="#0a8a72", teres_maj="#c8321e", rhomboid="#8a3aa8", trap_mid="#5a9a3a", trap_low="#5a9a3a", lat_dorsi="#777")
for k in ("trap_mid", "rhomboid", "trap_low", "infraspin", "teres_maj", "lat_dorsi"):
    a, b, c, s = LD.pair(k); d = LD.dvec(*c); tr = s["travel_mm"]
    for q in (a, b):
        ax.add_patch(Circle(q, LD.DISC_R, fc=cols[k], ec="k", lw=0.6)); ax.add_patch(Circle(q + d * tr, LD.DISC_R, fc="none", ec=cols[k], lw=1, ls=":"))
    ax.annotate("", xy=c + d * tr, xytext=c, arrowprops=dict(arrowstyle="->", color=cols[k], lw=1.2)); ax.text(c[0] + 6, c[1] + 9, k, fontsize=7, color=cols[k])
for k in ("infraspin", "teres_maj"):
    P = LD.lead_path(k)[0]; ax.plot(P[:, 0], P[:, 1], color="k", lw=1.8)
ju = np.array([math.cos(math.radians(LD.JACK_PSI)), math.sin(math.radians(LD.JACK_PSI))]); jc = np.array([LD.Y1, LD.Z1]) + 68 * ju
ax.add_patch(Rectangle(jc - [13, 6], 26, 12, angle=0, fc="#e8891a", ec="k", label="jack plate (PLACEHOLDER) psi 240")); ax.text(jc[0] - 12, jc[1] - 20, "jack psi 240", fontsize=7)
for z in (-160, -220):
    for y in (-113, -105): ax.add_patch(Circle((y, z), 5, fc="#8a8f98", ec="k", lw=0.5))
ax.plot([], [], "o", color="#8a8f98", label="M4 spacers D10 (4 bosses)")
ax.plot([], [], color="k", lw=1.8, label="flex lead path (-d corridor, then along the liner edge)"); ax.plot([], [], "o", ms=6, mfc="none", mec="k", ls=":", label="electrode at end of travel (atlas)")
ax.set_xlim(-170, 40); ax.set_ylim(-320, -110); ax.set_xlabel("y (mm), wearer right to the right"); ax.set_ylabel("z (mm)")
ax.set_title("MJ11 liner, skin-side plan: relief slots, lead paths, jack (back view)", fontsize=10); ax.legend(fontsize=6.5, loc="lower left"); ax.grid(alpha=0.25)
out = os.path.join(REPO, "renders_j1r2", "mounts", "MJ11_leads_plan.png"); fig.savefig(out, bbox_inches="tight"); print(out)
