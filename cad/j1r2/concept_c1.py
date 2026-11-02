"""C1 Monocoque saddle (plate-less): ribbed PA-CF saddle replaces the 6061 plate and is the wing's outboard end."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
import concept_ref
def build():
    R = concept_ref.build(); P = {}
    outline = plate_outline(); sh = sg.Polygon(outline)
    skin = prism(outline, "x", -86, -80)                                   # 6 mm skin, rear face -80 seats the S1 base (unchanged)
    ribs = []
    for z in (-62, -80, -98, -116, -134): ribs.append(box(-92, -86, -90, 90, z - 1.5, z + 1.5))        # 3 mm ribs along y, 6 deep
    for y in (-70, -50, -30, 0, 30, 50, 70): ribs.append(box(-92, -86, y - 1.5, y + 1.5, -160, -50))   # ribs along z
    rim = D(prism(outline, "x", -92, -86), prism([(p[0] + (0 if False else 0), p[1]) for p in sg.Polygon(outline).buffer(-3).exterior.coords], "x", -93, -85))
    web = I(U(*ribs), prism(outline, "x", -93, -85))
    sad = U(skin, web, rim)
    for (y, z) in S1_BASE_BLOCK["bolts"]: sad = D(sad, cyl("x", 3.9, -92.5, -79.5, c=(y, z)))   # M5 knurled brass insert pocket D7.8 (insert OD 7-8, unverified)
    doubler = D(box(-88, -86, -70, 70, -103, -59), *[cyl("x", 2.6, -89, -85, c=(y, z)) for (y, z) in S1_BASE_BLOCK["bolts"]])
    sad = D(sad, box(-88, -86, -70, 70, -103, -59))                        # recess for the doubler
    P["saddle_PACF"] = dict(mesh=sad, mat="PACF", fill=0.9, group="mount", colour=COL["pacf"], contact=False)
    P["steel_doubler_2mm"] = dict(mesh=doubler, mat="STEEL", group="mount", colour=COL["steel"], contact=False)
    P["s1_base_block"] = R["s1_base_block"]; P["armour_crown"] = R["armour_crown"]; P["armour_spaulder"] = R["armour_spaulder"]
    keel = R["armour_keel"]["mesh"]
    keel = U(D(keel, box(-89, -79.5, -53, 53, -114, -104)), box(-80, -74, -52, 52, -113, -105))   # feet re-seated on the saddle rear face (M4, 2x)
    P["armour_keel"] = dict(mesh=keel, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    return P, sad
if __name__ == "__main__":
    P, sad = build()
    g = 9.81; W = (1.566 + 2.2) * g; Mc = W * 0.080; Msum = Mc + 8.3
    pitch = 0.026; Tpair = Msum / pitch; Tb = Tpair / 2; Tdyn = 2 * Tb; shear = np.hypot(8.3 / 0.124, W) / 4
    # section: skin 158x6 + 7 z-ribs 3x6 (crosswise, conservative: y-ribs ignored)
    b, t, nr, rw, rh = 158.0, 6.0, 7, 3.0, 6.0
    A = [(b * t, t / 2), (nr * rw * rh, t + rh / 2)]; yb = sum(a * y for a, y in A) / sum(a for a, _ in A)
    I_ = b * t**3 / 12 + A[0][0] * (A[0][1] - yb) ** 2 + nr * rw * rh**3 / 12 + A[1][0] * (A[1][1] - yb) ** 2; c = t + rh - yb
    sig = Msum * 1e3 * c / I_
    mass_ref = 565.6; m = export_concept("C1", "Monocoque saddle (plate-less)", P, notes=dict(ref_mount_g=mass_ref), extra={})
    mass = m["mount_mass_g"]; d = round(mass - mass_ref, 1)
    extra = dict(
        summary="6061 plate replaced by a ribbed PA-CF saddle (6 mm skin + 6 mm ribs, x -92..-80) that is the wing's outboard end; 2 mm steel doubler (140x44) backs the 4 x M5 knurled brass inserts; armour keel feet re-seated on the saddle rear face.",
        mass_delta_g_vs_REF_proxy=d, mount_mass_g=mass, ref_mount_g=mass_ref, spec_basis="spec plate 278 g -> saddle %.0f g + doubler %.0f g (proxy volumes)" % (m["parts"][0]["mass_g"], m["parts"][1]["mass_g"]),
        contract_changes=["Plate part number removed: wing tongue seat, 4 x M5 base pattern, clasp-foot M4 pairs, keel M4 feet now land in/behind the saddle (inserts instead of tapped 6061); plate.json and chassis_axilla.py retired; wing/saddle become one print or a lap joint.", "Clasp feet (front face) need M4 brass inserts in the saddle skin; skin must thicken to >=6 mm at those feet (done by 6 mm skin)."],
        decision_conflicts=["None of: no backplate, no hip belt, 270 deg clasp, 5 deg rule, ball-styled joints. Saddle outline equals the plate outline (158x100), so it is not a backplate. Rigid part sits at x >= -92, >= 26 mm off skin; nothing new on moving skin.", "Touches 'plate 6061' choice made by Claude (not Forest)."],
        calcs=[dict(name="load case", formula="W=(m_arm+m_stage) g; Mc=W*0.080 m; M_y=Mc+S2 cap", inputs="m_arm 1.566 kg, m_stage 2.2 kg (assumed), cap 8.3 N.m", result="W=%.1f N, Mc=%.2f N.m, M_y=%.2f N.m" % (W, Mc, Msum)),
               dict(name="bolt group", formula="T_pair=M_y/p (p=26 mm z pitch), per bolt=T_pair/2; dynamic x2; shear=sqrt((8.3/0.124)^2+W^2)/4 N", inputs="p 0.026 m, y pitch 0.124 m", result="per-bolt tension %.0f N static, %.0f N at x2; shear %.0f N; conservative (ignores base-block bearing on the saddle face)" % (Tb, Tdyn, shear)),
               dict(name="insert pull-out", formula="SF=F_pull/F_bolt", inputs="~1000 N typical for knurled brass in PA-CF (UNVERIFIED vendor figure)", result="SF %.1f static, %.1f at x2 (marginal at x2; doubler loads the inserts through the steel, so bench pull-test required)" % (1000 / Tb, 1000 / Tdyn)),
               dict(name="saddle section", formula="I=sum(b h^3/12 + A d^2); sigma=M c/I", inputs="skin 158x6, 7 cross ribs 3x6, centroid %.2f mm, c %.2f mm, PA-CF flexural strength ~100 MPa (UNVERIFIED), y-running ribs ignored" % (yb, c), result="I=%.0f mm4, sigma=%.1f MPa -> SF %.1f vs 100 MPa; mid-thickness assumed isotropic, layer lines across the 6 mm skin are the weak direction" % (I_, sig, 100 / sig))],
        harness=dict(rule="120 g per arm = 0.30 kPa", delta_g=d, delta_top_pad_kPa=round(d / 120 * 0.30, 2), top_pad_kPa_est=round(14.28 + d / 120 * 0.30, 2), limit=14.7),
        printability="Saddle 158x100x12: fits 256 cube; print skin flat on bed (rear face -80 down) so layers run in the plane of the skin; ribs up, no supports; bolt tension then acts across layers in the skin only through the steel doubler. Wing lap or one-piece print needs rotation (wing ~ 62x105 as proxy) - keep as lap joint.",
        pros=["~%.0f g lighter per arm than the 6061 plate (spec 278 g)" % (278.8 - m["parts"][0]["mass_g"] - m["parts"][1]["mass_g"]), "no waterjet/tap step; wing and plate one design, re-fit (I7) is a re-run", "top pad pressure margin improves"],
        cons=["load path now PA-CF across layers; inserts and doubler carry everything", "loses the 6061 stiffness for the S2 8.3 N.m reaction", "creep under sustained weight (PA-CF) not characterised"],
        risks=["insert pull-out figure unverified", "PA-CF creep/anneal distortion of a 158x100 saddle", "wing lap stiffness"],
        gates_to_rerun=["verify_harness / harness_loads (mass)", "verify_armor (keel/spaulder seats)", "verify_assembly", "body_loads (stall limit uses plate stiffness assumptions)", "print_table"],
        assumptions=["proxy geometry, not the verified STLs", "stage mass 2.2 kg assumed", "rib pattern/sizes are first-pass", "proxy basis overstates armour mass (REF armour 161.6 g vs spec 110.6 g)"])
    import json; m["extra"] = extra; json.dump(m, open("stl/concepts/C1/metrics.json", "w"), indent=1, default=float)
    for r in m["parts"]: print(r["name"], r["mass_g"], r["watertight"], r.get("skin_min_mm"))
    print("mount", mass, "delta", d, "sigma", round(sig, 1), "Tb", round(Tb))
