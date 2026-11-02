"""C5: fit-adjustable cassette. Serrated adapter strips + shim packs let P slide +-10 z (1.5 mm tooth pitch), +-8 x (shim stack), 3 tilt wedges (0/+-5 deg, FIT only, lockable at 0). PROXY geometry."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, itertools, json
from clib import *
from scipy.ndimage import maximum_filter
XF, PITCH, DEPTH = -80.0, 1.5, 0.8
def zig(z0, z1):
    pts = []; n = int(round((z1 - z0) / PITCH))
    for i in range(n):
        z = z0 + i * PITCH; pts += [(XF, z), (XF - DEPTH, z + PITCH / 2)]
    pts.append((XF, z0 + n * PITCH)); return pts
def build():
    P = {}
    # ---- plate: slots (z +-10) + serrated strips at y +-62 (teeth ridges along y), keel holes removed
    plate = prism(plate_outline(), "x", PLATE_X[0], PLATE_X[1])
    zp = zig(-126.0, -54.0)
    for yc in (-62.0, 62.0):
        plate = D(plate, prism([(XF + 0.5, zp[0][1])] + zp + [(XF + 0.5, zp[-1][1])], "y", yc - 9.5, yc + 9.5))
        for zc in (-94.0, -68.0): plate = D(plate, prism([(-62.75, zc - 12.75 + 0.0), (-62.75, zc + 12.75)], "x", 0, 0)) if False else D(plate, hull(cyl("x", 2.75, -90, -78, c=(yc, zc - 10)), cyl("x", 2.75, -90, -78, c=(yc, zc + 10))))
    P["plate_6061_slotted"] = dict(mesh=plate, mat="AL", group="mount", colour=COL["al"], contact=False)
    # backing bars on the plate front (tapped M5) take the bolts through the slots
    bars = U(*[box(-91, -88, yc - 6, yc + 6, -108, -54) for yc in (-62.0, 62.0)])
    P["backing_bars_x2"] = dict(mesh=bars, mat="AL", group="mount", colour=COL["al"], contact=True)
    # ---- serrated adapter strips (6061, 5 mm + 0.8 teeth) at the two bolt columns, with a lower tab for the keel
    ad = []
    for yc in (-62.0, 62.0):
        s = box(XF, XF + 5, yc - 9, yc + 9, -116, -58)
        zt = zig(-116.0, -58.0); teeth = prism([(XF + 0.3, zt[0][1])] + zt + [(XF + 0.3, zt[-1][1])], "y", yc - 9, yc + 9)
        s = U(s, teeth)
        for zc in (-94.0, -68.0): s = D(s, cyl("x", 2.65, -85, -70, c=(yc, zc)))
        ad.append(s)
    P["adapter_strips_x2"] = dict(mesh=U(*ad), mat="AL", group="mount", colour=COL["teal"], contact=True)
    # ---- shim packs 8 mm nominal (0.5/1/2/4 laser-cut stainless-or-Al leaves) -> x range 0..16 (+-8)
    sh = []
    for yc in (-62.0, 62.0):
        s = box(XF + 5, XF + 13, yc - 9, yc + 9, -100, -62)
        for zc in (-94.0, -68.0): s = D(s, cyl("x", 2.65, -85, -60, c=(yc, zc)))
        sh.append(s)
    P["shim_packs_8mm_x2"] = dict(mesh=U(*sh), mat="AL", group="mount", colour=COL["accent"], contact=True)
    # ---- S1 base block shortened by the 13 mm stack (P unchanged at nominal)
    blk = box(-67, 0, -68, 68, -100, -62); blk = D(blk, cyl("z", 44.6, -101, -61)); blk = D(blk, box(-63, -49, -44, 44, -100.5, -66))
    for yc in (-62.0, 62.0):
        for zc in (-94.0, -68.0): blk = D(blk, cyl("x", 2.65, -70, -60, c=(yc, zc)))
    P["s1_base_block_short"] = dict(mesh=blk, mat="PETGCF", fill=0.45, group="mount", colour=COL["steel"], contact=False)
    # ---- spare tilt wedges (x-thickness 4 mm at the thin edge, 5 deg over 38 mm) ; fit-only
    for nm, sgn in (("wedge_p5", 1), ("wedge_m5", -1)):
        t0, t1 = 4.0, 4.0 + 38 * np.tan(np.radians(5))
        poly = [(XF + 5, -100), (XF + 5 + (t1 if sgn > 0 else t0), -100), (XF + 5 + (t0 if sgn > 0 else t1), -62), (XF + 5, -62)]
        w = U(*[prism(poly, "y", yc - 9, yc + 9) for yc in (-62.0, 62.0)])
        for yc in (-62.0, 62.0):
            for zc in (-94.0, -68.0): w = D(w, cyl("x", 2.65, -85, -60, c=(yc, zc)))
        P[nm + "_spare"] = dict(mesh=w, mat="PACF", group="spare", colour=COL["lime"], contact=True)
    # ---- armour: moves with P (crown on the block, keel on the adapter tab), rear edge pulled forward of the stack, TPU gaiter closes the 13 mm gap
    cut = box(-58, 100, -200, 200, -200, 100); crown_o = I(hull(cyl("z", 73.5, -99, -50), box(-58, -10, -71, 71, -99, -50)), cut); crown_i = I(hull(cyl("z", 71.7, -101, -52), box(-56.2, -10, -69.2, 69.2, -101, -52)), box(-70, 100, -200, 200, -200, 100))
    crown = D(crown_o, crown_i, cyl("z", 66.5, -53, -48.0))
    P["armour_crown"] = dict(mesh=crown, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    keel_o = I(hull(cyl("z", 72.6, -115, -97), box(-58, -10, -70, 70, -115, -97)), cut); keel_i = I(hull(cyl("z", 70.8, -113.2, -95), box(-56.2, -10, -68.2, 68.2, -113.2, -95)), box(-70, 100, -200, 200, -200, 100))
    keel = U(D(keel_o, keel_i), *[box(-77, -58, yc - 9, yc + 9, -118, -110) for yc in (-62.0, 62.0)])
    P["armour_keel"] = dict(mesh=keel, mat="PACF", group="armour", colour=COL["pacf"], contact=True)
    gai = D(box(-86, -54, -72, 72, -112, -50), box(-87, -53, -70.8, 70.8, -110.8, -51.2))
    P["gaiter_TPU"] = dict(mesh=gai, mat="TPU", group="armour", colour=COL["tpu"], contact=True)
    # spaulder: fixed to plate top edge + clasp seats; rear scalloped around the swept crown (+-8 x, +-10 z, +-5 deg) grown 3 mm
    X, Y, H = top_surface(-208, -84, -58, 88, 4.0, 4.0)
    ramp = -54.0 - 0.62 * (-84.0 - X); floor = np.maximum(maximum_filter(H, size=4, mode="nearest") + 12.5, ramp); floor = gaussian_filter(floor, 2.0, mode="nearest")
    sp = height_solid(X, Y, floor, floor + 1.8, "z")
    sp = I(sp, prism([(-84 - 62 + 62 * np.cos(a), 15 + 70 * np.sin(a)) for a in np.linspace(0, 2 * np.pi, 60, endpoint=False)], "z", -300, 100))
    cfgs = [("nom", np.eye(4))] + [(f"dx{dx:+d}dz{dz:+d}", tr([dx, 0, dz])) for dx in (-8, 8) for dz in (-10, 10)]
    Rt = lambda a: tr([XF, 0, -81]) @ rot(np.radians(a), [0, 1, 0]) @ tr([-XF, 0, 81]) @ np.eye(4)
    Rt = lambda a: tr([XF, 0, -81]) @ rot(np.radians(a), [0, 1, 0]) @ tr([-XF, 0, 81])
    cfgs += [("tilt+5", Rt(5)), ("tilt-5", Rt(-5)), ("tilt+5dz+10", tr([0, 0, 10]) @ Rt(5)), ("tilt-5dz-10", tr([0, 0, -10]) @ Rt(-5))]
    sw = []
    for _, T in cfgs: c = crown.copy(); c.apply_transform(T); sw.append(c)
    swept = hull(*sw); grown = hull(*[swept.copy().apply_translation(v) or swept.copy().apply_translation(v) for v in ([3,0,0],[-3,0,0],[0,3,0],[0,-3,0],[0,0,3],[0,0,-3])])
    sp = D(sp, grown)
    P["armour_spaulder"] = dict(mesh=sp, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    return P, cfgs
def sweep(P, cfgs):
    from scipy.spatial import cKDTree
    S = lambda m, n=5000: trimesh.sample.sample_surface(m, n)[0]
    fixed = {k: cKDTree(S(P[k]["mesh"], 25000)) for k in ("plate_6061_slotted", "armour_spaulder", "backing_bars_x2")}
    moving = ["armour_crown", "armour_keel", "s1_base_block_short"]; rows = []
    for nm, T in cfgs:
        r = dict(cfg=nm)
        for k in moving:
            m = P[k]["mesh"].copy(); m.apply_transform(T); r[k + "_skin"] = round(skin_gap(m, 4000)[0], 1)
            pts = S(m, 4000)
            for f, t in fixed.items():
                if k == "armour_keel" and f == "plate_6061_slotted": continue    # keel tab sits on the adapter, not on the plate
                if k == "s1_base_block_short" and f in ("plate_6061_slotted", "backing_bars_x2"): continue
                r[f"{k}->{f}"] = round(float(t.query(pts)[0].min()), 1)
        sp = P["armour_spaulder"]["mesh"]; r["spaulder_skin"] = round(skin_gap(sp, 4000)[0], 1) if nm == "nom" else None
        c = P["armour_crown"]["mesh"].copy(); c.apply_transform(T); r["crown_spaulder_interf_mm3"] = round(float(I(c, sp).volume), 1) if I(c, sp).is_volume else 0.0
        rows.append(r)
    return rows
if __name__ == "__main__":
    P, cfgs = build(); rows = sweep(P, cfgs)
    ref = json.load(open(os.path.join(ROOT, "stl/concepts/REF/metrics.json")))
    ex = dict(summary="Slotted 6061 plate + two serrated adapter strips + 8 mm shim packs + shortened S1 block: P re-fits +-10 z (1.5 mm teeth), +-8 x (shims 0.5/1/2/4), 3 wedge options (0/+-5 deg, fit only, lock at 0), same 4 x M5 columns; armour oversized to clear the range.")
    ex["static_sweep_mm"] = rows
    ex["ref_mount_mass_g_proxy"] = ref["mount_mass_g"]
    ex["pros"] = ["re-fit to Forest's measured body (I7) by loosening 4 bolts, no new plate/wing/pipeline run for P offsets within +-10 z / +-8 x", "wing, rail, clasp feet and plate outline unchanged",
                  "same 4 x M5 column pattern and S1 bore; stage, cowl, dome untouched", "tilt wedges give a fit option without re-cutting anything"]
    ex["cons"] = ["adapter+shims+backing bars+gaiter add ~84 g; net +21.7 g on the proxy basis after the shorter block and trimmed armour (spares: 2 wedges 17.8 g not counted)", "more joints: 3 more faces that can creep/loosen; PETG-CF block bearing on pad shims", "tilt wedge moves P 7 mm and tilts the S1 axis: head keep-out/keepout tables must be re-run per wedge", "armour needs a TPU gaiter and a bigger spaulder scallop (less flush by ~2 mm gap)"]
    ex["risks"] = ["serration y-slip is friction only (ridges run along y)", "backing bars on plate front may collide with clasp feet locations (clasp_feet.json not available)", "5 deg wedge breaks the 2026-10-12c 'mount tilt withdrawn' choice (Claude decision, not Forest's): option for FIT only, ships at 0, locked by omitting wedge"]
    ex["contract_changes"] = ["plate: 4 x M5 tapped holes -> 4 slots (5.5 x 25.5) + 2 serrated grooved strips (1.5 pitch x 0.8 deep, y +-9.5 about +-62, z -126..-54)", "plate: 2 x M4 keel csk holes at (+-50,-109) deleted (keel now bolts to adapter tabs)", "plate front: 2 backing bars (x -91..-88) tapped M5", "S1 base block 80 -> 67 mm deep, bolt holes become through-clearance", "spaulder rear edge scalloped around the swept crown", "crown/keel rear edge x -84 -> -58, rear of the round plan cut off flat (the 73.5 mm radius would hit the plate at dx -8); TPU gaiter closes to the plate"]
    ex["decision_conflicts"] = ["'Mount tilt withdrawn' (Claude, 2026-10-12c): reintroduced only as selectable fit wedges, default 0 deg", "no Forest-decision conflict: no backplate, no hip belt, 5 deg stop margin, DOF, actuators unchanged"]
    ex["gates_to_rerun"] = ["verify_armor + verify_shoulder at the adjustment extremes", "harness_loads (net +~22 g proxy -> top pad ~+0.06 kPa est; verify vs 14.7)", "keepout_map/elbow tables for any non-zero wedge", "scapular_frame tongue/clasp feet vs backing bars", "print/assembly sequence"]
    ex["assumptions"] = ["clasp F1/F4 seats assumed; ref plate outline from spec seeds", "6061 adapter strips CNC (printing 0.8 mm teeth is not credible)", "M5 8.8 bolts, usable preload 2 kN each (unverified; PETG-CF block creep not measured)", "mu(Al-Al, serrated) 0.3 unverified"]
    # calcs
    Fc = 4 * 2000.0; mu = 0.3; phi = np.degrees(np.arctan(mu)); alpha = 30.0
    Mb = 8.3 + 3.3 * 9.81 * 0.080; T_top = Mb / 0.032
    ex["calcs"] = [
      dict(name="joint separation moment", formula="M_sep = F_c * a ; a = half bolt-column span 26 mm/2 + ~3 mm = 16 mm (pivot at block lower edge: a=34 mm lower bound uses 32)", inputs=dict(F_c_N=Fc, a_mm=16), result=dict(M_sep_Nm=round(Fc * 0.016, 1), M_applied_Nm=round(Mb, 1), SF=round(Fc * 0.016 / Mb, 1))),
      dict(name="bolt tension from bending", formula="M = 8.3 (S2 cap) + m g L ; T_top = M / 32 mm (pivot bottom edge to top bolts)", inputs=dict(m_kg=3.3, L_m=0.080), result=dict(M_Nm=round(Mb, 1), T_top_total_N=round(T_top), per_bolt_N=round(T_top / 2))),
      dict(name="z-shear vs tooth jack-up", formula="F_slip = F_c * tan(alpha + atan(mu)) ; alpha 30 deg half-flank", inputs=dict(F_c_N=Fc, mu=mu), result=dict(F_slip_N=round(Fc * np.tan(np.radians(alpha + phi))), F_applied_N=round(3.3 * 9.81 * 3), SF=round(Fc * np.tan(np.radians(alpha + phi)) / (3.3 * 9.81 * 3)))),
      dict(name="y-shear (friction only)", formula="F_slip = mu F_c ; F_y ~ 0.5 g * 3.8 kg + 8.3/0.578", inputs=dict(mu=mu), result=dict(F_slip_N=round(mu * Fc), F_applied_N=round(0.5 * 9.81 * 3.8 + 8.3 / 0.578, 1))),
      dict(name="P-position error budget", formula="z quantisation +-pitch/2; x quantisation +-0.25 (0.5 mm leaf); tilt: dP = 80 sin(5 deg), tip = 580 sin(5 deg); slot clearance 0.25 -> angle 0.25/124 rad at 575 mm", result=dict(z_mm=0.75, x_mm=0.25, P_shift_per_5deg_mm=round(80 * np.sin(np.radians(5)), 1), tip_shift_per_5deg_mm=round(580 * np.sin(np.radians(5)), 1), tip_from_slot_slop_mm=round(0.25 / 124 * 575, 2)))]
    ex["printability"] = "adapter strips and backing bars: CNC 6061 (or waterjet + tooth cut); shim leaves laser-cut; wedges PA-CF flat on bed; crown/keel/spaulder as REF (all < 256 mm); teeth must NOT be printed."
    ex["fitting_procedure"] = "1) wear the harness with arm at park; 2) Forest measures acromion-to-P height and the P-to-back standoff (tape against a rigid jig, or a scan); 3) loosen 4 M5 (arm weight supported), slide the block in z by the delta (snap to 1.5 mm teeth), add/remove shim leaves for x, retighten to the torque in the spec (target 5 N.m, unverified); 4) re-zero S2 if a wedge is used; about 10 minutes, no re-print."
    nm_cons = {}
    meta = export_concept("C5", "C5 Fit-adjustable cassette (serrated slot mount)", P, notes=dict(basis="proxy volumes, same method as REF"), extra=ex)
    for r in meta["parts"]: print(r["name"], r["mass_g"], r["skin_min_mm"] if "skin_min_mm" in r else "")
    print("mount", meta["mount_mass_g"], "ref", ref["mount_mass_g"], "delta", round(meta["mount_mass_g"] - ref["mount_mass_g"], 1))
    for r in rows: print(r)
