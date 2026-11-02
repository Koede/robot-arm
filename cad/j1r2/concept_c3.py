"""C3 Spine column + outrigger beam (PROXY). Short hollow dorsal column on the rail pads/spine bridge, y -200..-100, z -222..-62 (spine plane y=-150),
hollow beam (PA-CF U-channel + 5 mm Al seat plate) from the column to the S1 base at the old plate face (x -80 rear face kept)."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, concept_ref as R
from clib import *
def build():
    P = {}
    # ---- column: hollow draped box, front face flat across y on the rail-pad top (17.8 mm off the highest back-surface point of each z row), 14 deep, 2.5 walls
    ZC = (-222.0, -62.0); YC = (-200.0, -100.0)
    Y, Z, X = rear_surface(YC[0], YC[1], ZC[0], ZC[1], 4.0, 8.0)
    xf = gaussian_filter(X.max(0), 2.0, mode="nearest") + 17.8; zs = Z[0]
    H0 = np.tile(xf, (Y.shape[0], 1)); outer = height_solid(Y, Z, H0, H0 + 14.0, "x")
    yi = np.arange(YC[0] + 2.5, YC[1] - 2.5 + 1e-6, 4.0); zi = np.arange(ZC[0] + 2.5, ZC[1] - 2.5 + 1e-6, 4.0); Yi, Zi = np.meshgrid(yi, zi, indexing="ij")
    xi = np.interp(zi, zs, xf); Hi = np.tile(xi, (len(yi), 1)); inner = height_solid(Yi, Zi, Hi + 2.5, Hi + 11.5, "x")
    col = D(outer, inner)
    col_full_g = col.volume / 1000 * DENS["PACF"]
    P["column_pacf"] = dict(mesh=col, mat="PACF", group="mount", colour=COL["teal"], contact=False, mass_g=round(col_full_g / 2, 1))   # one arm's half share
    # Al insert: two 20 x 3 mm flat bars inside the column walls (z -216..-68), tapped for 4 x M4 into the rail/bridge bosses
    for k, y in enumerate((-125.0, -175.0)):
        xm = float(np.interp(-142.0, zs, xf)) + 7
        P[f"column_al_bar{k}"] = dict(mesh=box(xm - 1.5, xm + 1.5, y - 10, y + 10, -216, -68), mat="AL", group="mount", colour=COL["al"], contact=False, mass_g=None)
        P[f"column_al_bar{k}"]["mass_g"] = round(20 * 3 * 148 / 1000 * DENS["AL"] / 2, 1)
    # ---- beam: PA-CF U-channel x -102..-85 (front, top, bottom walls 2.5), closed by the Al seat plate x -85..-80; y -110 (inside column) .. 76; z -116..-54
    yb0, yb1, zb0, zb1 = -110.0, 76.0, -116.0, -54.0
    ch = D(box(-102, -85, yb0, yb1, zb0, zb1), box(-99.5, -84.9, yb0 + 2.5, yb1 - 2.5, zb0 + 2.5, zb1 - 2.5))
    ch = U(ch, box(-99.5, -85, yb0, yb0 + 2.5, zb0, zb1))
    P["beam_pacf_channel"] = dict(mesh=ch, mat="PACF", group="mount", colour=COL["teal"], contact=False)
    seat = box(-85, -80, -72, 72, -108, -56)
    for (y, z) in S1_BASE_BLOCK["bolts"]: seat = D(seat, cyl("x", 2.1, -90, -78, c=(y, z)))
    P["beam_al_seat_plate"] = dict(mesh=seat, mat="AL", group="mount", colour=COL["al"], contact=False)
    # ---- unchanged parts of the baseline: S1 base block + armour (crown, keel, spaulder)
    ref = R.build()
    for k in ("s1_base_block", "armour_crown", "armour_keel", "armour_spaulder"): P[k] = ref[k]
    # ghost: left beam (mirror about y = -150) for the render, not counted in the mass
    gb = ch.copy(); gb.apply_transform(np.diag([1, -1, 1, 1.0])); gb.apply_translation([0, -300, 0])
    P["ghost_left_beam"] = dict(mesh=gb, mat="PACF", group="ghost", colour=(0.5, 0.5, 0.55), contact=False)
    return P, dict(col_full_g=col_full_g, xf=(float(xf.min()), float(xf.max())))
if __name__ == "__main__":
    P, aux = build()
    # ---- calcs
    E_pa, E_al, G_pa = 6000.0, 69000.0, 2000.0     # MPa, PA-CF printed (UNVERIFIED), 6061 (handbook), PA-CF shear (UNVERIFIED)
    n = E_al / E_pa; t = 2.5
    Iz_ch = 2 * (17 * t) * (62 / 2 - t / 2) ** 2 + t * 62 ** 3 / 12           # vertical bending of the channel (flanges + web)
    Iz_al = 5 * 52 ** 3 / 12; Iv_eq = Iz_ch + n * Iz_al
    Ih_ch = (62 * 22 ** 3 - 57 * 17 ** 3) / 12 * 0 + (2 * (2.5 * 17) * 0 + 0)   # placeholder, replaced below
    # horizontal bending (about z): front web (2.5 x 62) at x offset to neutral axis, flanges thin, Al plate (5 x 52)
    xs_ = {"web": (-100.75, 2.5 * 62), "al": (-82.5, 5 * 52)}; Ih = 2 * (17 * t ** 3 / 12 + 17 * t * 0)   # flanges negligible about z-bending apart from d^2 term
    def I_h(E_ratio):
        parts = [(-100.75, 2.5 * 62, 2.5 ** 3 * 62 / 12, 1.0), (-85 - 8.5, 2 * 17 * t, 2 * t * 17 ** 3 / 12, 1.0), (-82.5, 5 * 52, 52 * 5 ** 3 / 12, E_ratio)]
        EA = sum(a * e for _, a, _, e in parts); xn = sum(x * a * e for x, a, _, e in parts) / EA
        return sum(e * (i + a * (x - xn) ** 2) for x, a, i, e in parts)
    Ih_eq = I_h(n)
    W, Lb = 3.2 * 9.81, 100.0           # N (stage + arm mass ~3.2 kg ASSUMED), boom length column edge -> S1 axis
    res = {}
    for L_ in (100.0, 150.0):
        Mx = W * L_ / 1000 + 8.3; dv = (W * L_ ** 3 / 3 + 8300 * L_ ** 2 / 2) / (E_pa * Iv_eq)
        res[f"L{int(L_)}"] = dict(M_vert_Nm=round(Mx, 2), tip_defl_vert_mm=round(dv, 3),
                                   M_horiz_Nm=8.3, tip_defl_horiz_mm=round(8300 * L_ ** 2 / 2 / (E_pa * Ih_eq), 3),
                                   sigma_Al_flange_MPa=round(Mx * 1000 * (31) / (E_pa * Iv_eq) * E_al, 2),
                                   sigma_PACF_flange_MPa=round(Mx * 1000 * 31 / (E_pa * Iv_eq) * E_pa, 2) if False else round(Mx * 1000 * 31 / Iv_eq, 2))
    # beam torsion (S2 cap about y): open-ish U closed by plate -> treat as closed box 22 x 62, t 2.5 PA-CF
    Ac = (22 - 2.5) * (62 - 2.5); pm = 2 * ((22 - 2.5) + (62 - 2.5)); Jb = 4 * Ac ** 2 * t / pm
    twist = 8300 * 100 / (G_pa * Jb); tau = 8300 / (2 * Ac * t)
    # column: closed box 100 x 14, t 2.5
    Acc = (100 - 2.5) * (14 - 2.5); pc = 2 * ((100 - 2.5) + (14 - 2.5)); Jc = 4 * Acc ** 2 * t / pc
    tw_c = 8300 / (G_pa * Jc) * 160           # rad over the 160 mm height, one arm at the S1 cap
    Ic_x = (100 * 14 ** 3 - 95 * 9 ** 3) / 12  # about y (pitch moment S2) bending the column in x-z
    calcs = [
        dict(name="boom vertical bending + tip deflection", formula="M = W*L + M_S2cap ; delta = W L^3/(3 E I) + M_S2 L^2/(2 E I)", inputs=dict(W_N=round(W, 1), E_PACF_MPa=E_pa, E_Al_MPa=E_al, I_eq_mm4=round(Iv_eq), section="U-channel 17 x 62 x 2.5 PA-CF + 5 x 52 Al plate (modular ratio %.1f)" % n), result=res),
        dict(name="boom horizontal bending (S1 cap reaction)", formula="delta = M L^2/(2 E I_h)", inputs=dict(I_h_eq_mm4=round(Ih_eq)), result="in result above (M_horiz 8.3 N.m)"),
        dict(name="boom torsion (S2 cap about the beam axis y)", formula="J = 4 A^2 t / p ; tau = T/(2 A t) ; theta = T L/(G J)", inputs=dict(T_Nm=8.3, L_mm=100, G_PACF_MPa=G_pa, A_mm2=round(Ac), p_mm=round(pm), t_mm=t), result=dict(J_mm4=round(Jb), tau_MPa=round(tau, 2), twist_deg=round(np.degrees(twist), 3), tip_error_at_575mm_mm=round(twist * 575, 2))),
        dict(name="column torsion, one arm vs two arms", formula="T_z = S1 cap reaction; mirrored arms: pseudovector components parallel to the mirror plane (M_x, M_z) cancel in the column, M_y (S2 reaction) adds", inputs=dict(T_one_Nm=8.3, J_col_mm4=round(Jc), G=G_pa),
             result=dict(one_arm_twist_deg_over_160mm=round(np.degrees(tw_c), 3), two_arms_symmetric_Tz_Nm=0.0, two_arms_symmetric_Mx_Nm=0.0, M_y_two_arms_Nm=16.6, column_Iy_mm4=round(Ic_x), note="asymmetric (one arm moving) is the design case; gravity moment W*0.15 = %.1f N.m cancels when both arms hang symmetric" % (W * 0.15))),
        dict(name="load path", formula="W_arm_total ~ 2 x 31 N (assumed)", inputs={}, result="Column front face rides on the rail pads (y -138..-100 and mirror) so the vertical load enters the paraspinal strips (rail model 12.7 kPa at 5.79 kg/side) and the top pad (C7/T1 + clasp top facet) sees the same N_top = W of the harness model; the boom changes where the moment enters, not the sum of vertical loads. Added S2-pitch reaction (16.6 N.m both arms at cap) now goes into the column as a x-z bending couple: rail pads +/- %.0f N over a 160 mm lever (rail pad row spacing 100 mm). The top-pad margin (2.0 %%) is NOT improved: the column adds mass at the top." % (16600 / 160 / 1)),
    ]
    meta = export_concept("C3", "Spine column + outrigger beam", P, {"proxy": "REF-basis masses; column counted as half (one arm)"}, {})
    ref_g = 565.6
    delta = round(meta["mount_mass_g"] - ref_g, 1)
    extra = dict(
        summary="Short 160 mm hollow dorsal column (z -222..-62, y -200..-100) on the rail pads/spine bridge, with a 186 mm PA-CF/Al outrigger beam carrying the S1 base on the old x -80 face; replaces the lateral plate + wing tongue and routes both arms' symmetric gravity moments into each other across the spine.",
        mass_delta_vs_REF_g=delta, mount_mass_g=meta["mount_mass_g"], ref_mount_mass_g=ref_g,
        column_length_mm=160, column_length_reason="z -62 (just below C7/T1, keeps the cervical/neck keel clear and avoids the occiput) to -222; only ~160 mm of thoracic spine is bridged by a rigid part (upper thoracic segments move least, ribcage-stiffened; ROM figures UNSOURCED this session) - a full-height column would stiffen trunk flexion, so it is kept short.",
        contract_changes=["Chassis plate + wing tongue (y -78..-46) deleted; the beam's Al seat plate keeps the x -80 rear face and the 4 x M5 pattern (y +-62, z -94/-68) but is 5 mm (was 8 mm) and 144 x 52 (was 158 x 100).", "Clasp feet (bolt pairs on the plate front) lose their seats: need a new landing (beam front wall / column) - NOT designed.", "Keel feet (2 x M4 at y +-50, z -109) land on the 62 mm tall channel, needs local solid bosses; spaulder rear feet (2 x M3 into the top edge) need solid lands in the 2.5 mm top wall.", "Wing bolts and the wing tongue seat are gone; the wing (rail bridge) is replaced by the beam, so wing/spine-bridge interface re-drawn; left and right booms share one column."],
        decision_conflicts=["No big backplate: pass (column 100 x 160 is a spine-bridge-sized member, not a backplate) but judge it against Forest's intent.", "No hip belt: pass. Load path still shoulder clasp + paraspinal pads.", "Trunk ROM: a rigid 160 mm member over T1-T8 adds some flexion stiffness (unquantified).", "Nerve-sensitive spine: column bridges the spinous line with >= 10 mm gap (17.8 mm over the highest back point per row, skin gap reported in metrics); centre relief required.", "Nothing rigid on moving skin: beam front is >= 22 mm off the back surface over the medial scapular border (-102 vs -124.6 max back x in y -100..76, z -116..-54): pass."],
        calcs=calcs,
        printability=dict(column="190 x 100 x 14 hollow draped box prints in the 256 mm cube; lay it on its flat rear-ish side (x up) with layers along z, 2.5 mm walls = 6 perimeters, tree supports inside none needed if printed with the open ends sealed by caps", beam="186 x 62 x 17 channel: print on its front face (open side up, no supports), Al plate bolts on; PA-CF, annealed"),
        pros=["Symmetric gravity moment (W*0.15 = %.1f N.m per arm) cancels in the shared column; plate (278.8 g) deleted" % (W * 0.15), "Lateral wing tongue and its wing-plate interface removed (wing-plate gap 0.65 mm, clasp-plate 0.2 mm are the tightest interfaces today)", "Stiff closed box column: one-arm twist only %.2f deg over 160 mm" % np.degrees(tw_c)],
        cons=["Clasp feet and keel/spaulder feet need new seats", "Boom tip error from torsion at the S2 cap %.1f mm at the 575 mm tip" % (twist * 575), "Rigid midline member across the spine: fit, donning (left/right now coupled), comfort", "Top-pad margin 2 % not improved"],
        risks=["PA-CF E and G are UNVERIFIED, deflections scale 1/E", "Column fit to Forest's back (I7) is a draped part - re-run on his mesh", "Spinal/nerve comfort and trunk ROM unmeasured", "Left/right coupling: one failed pad loads both arms' stage"],
        gates_to_rerun=["verify_harness (pressures, new mass and moment path)", "body_loads (stall limit with the column)", "verify_shoulder/verify_armor (spaulder/keel seats)", "sweep_full for the arm vs the beam/column", "print table", "assembly sequence (clasp, column, beams)"],
        assumptions=["Stage + arm mass beyond the base 3.2 kg (ASSUMED, not from harness_loads)", "L = 100 mm from the column edge to the S1 axis (150 mm to the spine line gives the second result set)", "REF proxy masses (armour proxy 161.6 g vs spec 110.6 g)", "Rail pad stack top 13.5 mm off the skin; column front face at +17.8 mm over the highest back point of each row", "Left boom ghost only; column counts as half"])
    meta["extra"] = extra; json_ = __import__("json"); json_.dump(meta, open(os.path.join(ROOT, "stl/concepts/C3/metrics.json"), "w"), indent=1, default=float)
    for r in meta["parts"]: print(r["name"], r["mass_g"], r.get("skin_min_mm"), r["watertight"], r["fits_256"])
    print("mount", meta["mount_mass_g"], "delta", delta); print(json_.dumps(res), round(np.degrees(twist), 3), round(np.degrees(tw_c), 3))
