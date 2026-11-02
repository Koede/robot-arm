"""REF: rev 2026-10-30b baseline shoulder mount (plate + S1 base + crown/keel/spaulder), rebuilt as a PROXY from the specs. Template for the concept scripts."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
def build():
    P = {}
    # chassis plate 6061, 8 mm, outline from the spec seeds
    plate = prism(plate_outline(), "x", PLATE_X[0], PLATE_X[1])
    for (y, z) in S1_BASE_BLOCK["bolts"]: plate = D(plate, cyl("x", 2.1, -90, -78, c=(y, z)))
    P["plate_6061"] = dict(mesh=plate, mat="AL", group="mount", colour=COL["al"], contact=False)
    # S1 base block (PETG-CF): rear half-wrap of the AK70 bore, pocketed from below
    blk = box(-80, 0, -68, 68, -100, -62); blk = D(blk, cyl("z", 44.6, -101, -61)); blk = D(blk, box(-76, -49, -44, 44, -100.5, -66))
    P["s1_base_block"] = dict(mesh=blk, mat="PETGCF", fill=0.45, group="mount", colour=COL["steel"], contact=False)
    # --- armour (PA-CF, 1.8 mm walls) : crown (cup round the S1 stage), keel (dish under the motor), spaulder (draped plate over the shoulder top)
    plan = lambda r, x0: hull(cyl("z", r, 0, 1), box(x0, -8, -r + 2, r - 2, 0, 1))
    crown_o = U(hull(cyl("z", 73.5, -99, -50), box(-84, -10, -71, 71, -99, -50))); crown_i = hull(cyl("z", 71.7, -101, -52), box(-82.2, -10, -69.2, 69.2, -101, -52))
    crown = D(crown_o, crown_i, cyl("z", 66.5, -53, -48.0))
    P["armour_crown"] = dict(mesh=crown, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    keel_o = hull(cyl("z", 72.6, -115, -97), box(-84, -10, -70, 70, -115, -97)); keel_i = hull(cyl("z", 70.8, -113.2, -95), box(-82.2, -10, -68.2, 68.2, -113.2, -95))
    keel = U(D(keel_o, keel_i), box(-88, -80, -52, 52, -113, -105))
    P["armour_keel"] = dict(mesh=keel, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    X, Y, H = top_surface(-208, -84, -58, 88, 4.0, 4.0)
    ramp = -54.0 - 0.62 * (-84.0 - X); from scipy.ndimage import maximum_filter
    floor = np.maximum(maximum_filter(H, size=4, mode="nearest") + 12.5, ramp)
    floor = gaussian_filter(floor, 2.0, mode="nearest")
    sp = height_solid(X, Y, floor - 0.0, floor + 1.8, "z")
    oval = [(-84 - 62 + 62 * np.cos(a), 15 + 70 * np.sin(a)) for a in np.linspace(0, 2 * np.pi, 60, endpoint=False)]
    sp = I(sp, prism(oval, "z", -300, 100))
    P["armour_spaulder"] = dict(mesh=sp, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    return P
if __name__ == "__main__":
    P = build(); m = export_concept("REF", "Baseline rev 2026-10-30b (proxy)", P, notes=dict(spec_mass_g=dict(plate=278, armour=110.6)))
    for r in m["parts"]: print(r)
    print("mount mass", m["mount_mass_g"])
