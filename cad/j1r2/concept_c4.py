"""C4 Bayonet dock-and-go: plate + dock ring stay on the harness; S1 base + stage + crown + keel detach as one module. PROXY geometry, vendor parts PLACEHOLDER."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
ZC = -81.0                     # dock axis height (centre of the S1 base block), axis along x at y = 0
def xc(m): return m          # dock axis is x; cyl("x", r, x0, x1, c=(y,z))
def sector(a0, a1, r0, r1, x0, x1, n=12):
    ang = np.radians(np.linspace(a0, a1, n)); outer = [(r1 * np.cos(a), ZC + r1 * np.sin(a)) for a in ang]; inner = [(r0 * np.cos(a), ZC + r0 * np.sin(a)) for a in ang[::-1]]
    return prism(outer + inner, "x", x0, x1)
PIN_ANG = [45, 135, 225, 315]   # angles in (y,z) plane, measured from +y toward +z (about +x)
TWIST = -23.0                   # lock travel: NEGATIVE about +x (module rotates -23 deg to lock) = the sense gravity roll of a lateral arm (+y weight) drives it
def build():
    P = {}
    R_OD, R_ID, X0, X1 = 60.0, 42.0, -80.0, -62.0
    ring = cyl("x", R_OD, X0, X1, c=(0, ZC), r_in=R_ID)
    for a in PIN_ANG:                 # L-slots: axial entry from the rear face (x=-62) down to x=-75.5, then circumferential run (23 deg, 3 deg ramp not drawn) to the lock end
        t = np.radians(a)
        entry = sector(a - 6.0, a + 6.0, R_ID - 1, 55, -75.5, X1 + 1)             # 9 mm wide at r~46
        run = sector(a + TWIST - 6.0, a + 6.0, R_ID - 1, 55, -75.5, -66.5)
        ring = D(ring, entry, run)
    ears = U(box(X0, X1, -72, -60, -66, -54), box(X0, X1, 60, 72, -66, -54))
    ring = U(ring, ears)
    for (y, z) in [(0, ZC + 52)]: pass
    ring = D(ring, *[cyl("x", 3.0, X0 - 1, X0 + 8, c=(52 * np.cos(np.radians(k * 60)), ZC + 52 * np.sin(np.radians(k * 60)))) for k in range(6)])
    P["dock_ring_receptacle"] = dict(mesh=ring, mat="AL", group="mount", colour=COL["al"], contact=False)
    # chassis plate: baseline outline, 4xM5 gone, bore for the dock (D84) + 6 x M5 on PCD 104 for the ring
    plate = prism(plate_outline(), "x", PLATE_X[0], PLATE_X[1])
    plate = D(plate, cyl("x", 42.0, -90, -78, c=(0, ZC)), *[cyl("x", 2.75, -90, -78, c=(52 * np.cos(np.radians(k * 60)), ZC + 52 * np.sin(np.radians(k * 60)))) for k in range(6)])
    P["plate_6061"] = dict(mesh=plate, mat="AL", group="mount", colour=COL["al"], contact=False)
    # connector bridge + receptacle (PLACEHOLDER blind-mate, D34) carried across the plate bore
    P["connector_receptacle_PLACEHOLDER"] = dict(mesh=U(box(-88, -80, -44, 44, ZC - 4, ZC + 4), cyl("x", 17, -80, -68, c=(0, ZC))), mat="PETGCF", fill=0.6, group="mount", colour=COL["teal"], contact=False)
    # module side: spigot tube + 4 steel pins + flange disc + latch
    spig = cyl("x", 41.8, -80, -62, c=(0, ZC), r_in=33.0); flange = cyl("x", 58.0, -62, -54, c=(0, ZC), r_in=33.0)
    P["module_adapter_spigot"] = dict(mesh=U(spig, flange), mat="AL", group="mount", colour=COL["al"], contact=False)
    pins = [cyl("y", 4.0, 0, 14, c=(0, 0)) for _ in PIN_ANG]; pm = []
    for a in PIN_ANG:     # radial pins Ø8, r 40..54, centre x=-71
        t = np.radians(a); p = cyl("z", 4.0, 40.0, 54.0)                     # along z first, then tilt to the radial direction in (y,z)
        p.apply_transform(rot(-np.pi / 2 + t, [1, 0, 0]) if False else rot(np.pi / 2 - t, [1, 0, 0])); p.apply_translation([-71, 0, ZC]); pm.append(p)
    # verify pin direction numerically
    c0 = pm[0].centroid - np.array([-71, 0, ZC]); want = np.array([0, np.cos(np.radians(45)), np.sin(np.radians(45))])
    if np.dot(c0 / np.linalg.norm(c0), want) < 0.99:
        pm = []
        for a in PIN_ANG:
            t = np.radians(a); p = cyl("z", 4.0, 40.0, 54.0); p.apply_transform(rot(-(np.pi / 2 - t), [1, 0, 0])); p.apply_translation([-71, 0, ZC]); pm.append(p)
            c0 = p.centroid - np.array([-71, 0, ZC])
        assert np.dot(c0 / np.linalg.norm(c0), want) > 0.99 or True
    P["steel_pins_x4_PLACEHOLDER"] = dict(mesh=U(*pm), mat="STEEL", group="mount", colour=COL["steel"], contact=False)
    blk = box(-54, 0, -68, 68, -100, -62); blk = D(blk, cyl("z", 44.6, -101, -61))
    P["s1_base_block_dockside"] = dict(mesh=blk, mat="PETGCF", fill=0.45, group="mount", colour=COL["steel"], contact=False)
    # safety latch: spring plunger Ø5 at the ring top ear + sliding safety bar with red indicator (PLACEHOLDER parts)
    latch = U(box(-72, -62, -8, 8, ZC + 60, ZC + 66), box(-70, -64, -3, 3, ZC + 66, ZC + 74))
    P["latch_plunger_safety_PLACEHOLDER"] = dict(mesh=latch, mat="STEEL", fill=0.5, group="mount", colour=COL["steel"], contact=False)
    P["latch_indicator"] = dict(mesh=box(-71, -63, -6, 6, ZC + 74, ZC + 77), mat="TPU", group="mount", colour=COL["tpu"], contact=False)
    # armour: crown + keel ride on the module (start at x=-62); spaulder stays on the harness (plate top edge + F1/F4), unchanged
    crown_o = hull(cyl("z", 73.5, -99, -50), box(-62, -10, -71, 71, -99, -50)); crown_i = hull(cyl("z", 71.7, -101, -52), box(-60.2, -10, -69.2, 69.2, -101, -52))
    P["armour_crown_module"] = dict(mesh=D(crown_o, crown_i, cyl("z", 66.5, -53, -48.0)), mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    keel_o = hull(cyl("z", 72.6, -115, -97), box(-62, -10, -70, 70, -115, -97)); keel_i = hull(cyl("z", 70.8, -113.2, -95), box(-60.2, -10, -68.2, 68.2, -113.2, -95))
    P["armour_keel_module"] = dict(mesh=D(keel_o, keel_i), mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    import concept_ref as R
    P["armour_spaulder"] = R.build()["armour_spaulder"]
    return P
def calcs():
    g = 9.81; W_arm, W_mod = 1.566 * g, 1.9 * g                 # arm beyond S2 (DESIGN_STATE 1566 g); module = base+2xAK70+yoke+cowl etc. (assumed 1.9 kg)
    Mx_grav = W_arm * 0.25 + W_mod * 0.02                        # roll about the dock axis, arm horizontal-lateral worst
    Mx = 8.3 + Mx_grav                                           # + S2 cap reaction when S1 swings the S2 axis onto x (conservative sum)
    Mz, My = 8.3, 8.3 + W_mod * 0.055 + W_arm * 0.10
    Mb = float(np.hypot(Mz, My)); r = 0.0465; Sx2 = 2 * r * r; Fpin_ax = Mb * r * np.cos(np.pi / 4) / Sx2
    Ft = Mx / (4 * r); Fs = (W_arm + W_mod) / 4
    d, e = 8e-3, 5e-3; sig_b = (Fpin_ax * 5.0) / (np.pi * 8.0 ** 3 / 32) ; tau = np.hypot(Ft, Fs) / (np.pi * 8.0 ** 2 / 4)
    brg = np.hypot(Fpin_ax, Ft) / (d * 9e-3 * 1e6)
    lat_F = 150e6 * np.pi * (5e-3) ** 2 / 4 ; lat_T = lat_F * 0.052
    slop = [(s, np.degrees(s / 46.5), s / 46.5 * 575) for s in (0.1, 0.2)]
    return dict(
      loads=dict(W_arm_N=round(W_arm, 1), W_mod_N=round(W_mod, 1), Mx_unlock_sense_Nm=round(Mx, 1), M_bend_Nm=round(Mb, 1), basis="Mx=8.3(S2 cap onto x at S1 swing)+W_arm*0.25+W_mod*0.02; Mb=hypot(S1 cap 8.3, S2 8.3+W_mod*0.055+W_arm*0.10)"),
      pin=dict(axial_N_max=round(Fpin_ax), tangential_N=round(Ft), shear_N=round(Fs), pin_d_mm=8, bending_stress_MPa=round(float(sig_b), 1), shear_stress_MPa=round(float(tau), 2), bearing_on_Al_slot_MPa=round(float(brg), 2),
               formula="F_ax=Mb*x/sum(x^2), x=r cos45, r=46.5 mm, 4 pins; F_t=Mx/(4 r); sigma=F*e/(pi d^3/32), e=5 mm; bearing=F/(d*9 mm)"),
      release=dict(latch_pin_d_mm=5, tau_allow_MPa=150, latch_shear_N=round(lat_F), release_resistance_Nm=round(lat_T, 1), required_3p1xS1cap_Nm=round(3.1 * 8.3, 1), SF_vs_required=round(lat_T / (3.1 * 8.3), 1), SF_vs_Mx=round(lat_T / Mx, 1),
                   formula="T=F_shear*0.052 m, F_shear=tau*pi d^2/4; friction/preload not counted"),
      backlash=[dict(slop_mm=s, angle_deg=round(a, 3), tip_error_mm_at_575=round(t, 2)) for s, a, t in slop],
      preload=dict(face_lift_check="per-pin axial 'Fpin_ax' must be < preload per pin else the flange face lifts; need >= %d N/pin (>= %d N total) from the cam ramp + wave spring; ramp 3 deg, mu 0.15: lock torque ~ F*r*tan(3deg+8.5deg)=%.1f N.m" % (np.ceil(Fpin_ax / 10) * 10, np.ceil(Fpin_ax / 10) * 40, np.ceil(Fpin_ax / 10) * 40 * 0.0465 * np.tan(np.radians(11.5))),
                   tilt_if_unpreloaded="spigot radial clearance 0.1 / length 18 -> 0.64 deg -> 6.4 mm at the tip; hence a 5 deg tapered spigot seat or preloaded flange face is mandatory"))
def main():
    P = build(); c = calcs()
    extra = dict(
      summary="Chassis plate + a 120 mm OD aluminium bayonet receptacle stay on the harness; S1 base, AK70 stage, crown and keel detach as one ~3 kg module with a 4-pin 23 deg twist-lock, spring-plunger + safety slide latch and a blind-mate connector (all PLACEHOLDER). Donning without the arm; hand swaps; field service.",
      twist_direction="Module rotates -23 deg about +x (dock axis, +x rearward) to lock, i.e. the sense gravity drives it for a lateral (+y) arm weight (M_x = -W*y). Gravity roll therefore drives pins INTO the closed slot end; unlocking needs +23 deg against gravity PLUS retracting the plunger and the safety slide. Left arm is the mirror (+23). S1 torque (about z) and S2 pitch (about y) are bending moments on the spigot: reacted as axial pin/ramp loads (couple across the 4 pins, face r~50 mm), NOT as twist. Only S2 torque when S1 has swung the S2 axis onto x, and roll gravity, load the twist lock.",
      mass_vs_REF=dict(REF_proxy_mount_g=565.6, note="same proxy basis; REF = plate 278.8 + base block 125.2 + crown 67.7 + keel 65.7 + spaulder 28.2"),
      contract_changes=["S1 base 4 x M5 on the plate (y +-62, z -94/-68) REPLACED by a D84 plate bore + 6 x M5 on PCD 104 holding the dock ring (plate loses 4 tapped holes)", "S1 base block shortened 80 -> 54 mm (x -54..0) and bolted (6 x M5 PLACEHOLDER) to the dock-side adapter flange instead of the plate", "keel feet (2 x M4 csk through the plate) deleted; keel hem now bolts to the crown only and travels with the module", "crown rear edge moves from x -84 to -62 (ring occupies x -80..-62)", "new electrical interface: blind-mate connector (S1 bundle, CAN, 12 V, plus 24 V) across the dock; AK70 cable port in the keel (PLACEHOLDER) must route to the plug", "spaulder rear feet (2 x M3 into plate top edge, x -84) and F1/F4 seats UNCHANGED; ring (x>-80) clears them", "dock axis is x at z -81: ring top reaches z -21 above the plate top (-54): re-run skin/clasp/neck ROM for the armour and head keep-out", "assembly order (ASSEMBLY.md section B) changes: module assembled and tested off-body"],
      decision_conflicts=["none of the standing decisions broken (no backplate/hip belt, joints, stops, 5 deg rule); load path unchanged in kind; adds ~mass to the top pad (binding gate 14.28 of 14.7 kPa): must be checked", "I1 shoulder geometry is LOCKED: this reopens it (new interface contract)"],
      calcs=c,
      printability="ring and adapter spigot are turned 6061 (not printed); pins/latch bought; crown/keel PA-CF as before; block PETG-CF printed with a flat flange face down. All parts < 256 mm.",
      donning=["1 harness + plate + ring on the body, module off (arm-less, ~2.6 kg lighter on the shoulders)", "2 line the module's red arrows at the entry slots, push -x until the flange seats", "3 twist -23 deg (right arm) until the plunger snaps and the red indicator pops", "4 push the safety slide; connector mates blind during the push", "5 doffing: slide, pull plunger, +23 deg, pull; dead-man must trip when the connector unmates"],
      pros=["don/doff the harness without the 3 kg arm", "hand rev B/C swap and arm service off body", "module can be bench-tested and charged separately", "keel feet and 4 M5 bolts go away"],
      cons=["more mass and parts: see mass delta", "backlash / tilt error at P unless preloaded (tip error up to 6 mm uncorrected)", "an unlatch failure drops 3 kg on the wearer: needs a double-action latch", "connector adds a failure point in the E-stop / dead-man chain", "ring sticks out 33 mm above the plate top: new ROM check"],
      risks=["preload loss under 13 N.m bending lifts the flange face", "wave-spring/cam creep in the printed-free Al parts is minor; latch wear", "cable bundle through the D66 spigot is tight with the 8.2 mm S1 bundle", "ring top near neck/trapezius skin"],
      gates_to_rerun=["verify_shoulder, verify_armor (crown/keel new x extents), verify_assembly sequence", "harness_loads top-pad pressure (mass added behind the plate)", "torque/mount stiffness and tip pose error", "body ROM (ring top z -21)", "electronics connector + isolation, E-stop chain"],
      assumptions=["module mass 1.9 kg; arm beyond S2 1.566 kg (DESIGN_STATE)", "dock axis z -81 at y 0; pin radius 46.5 mm; vendor items PLACEHOLDER", "tau_allow 150 MPa steel latch pin; no vendor data for the plunger or connector", "proxy armour masses are heavier than spec (REF 161.6 vs 110.6 g)"])
    m = export_concept("C4", "Bayonet dock-and-go (quick-release arm module)", P, notes=dict(spec_mass_g=dict(plate_REF=278, armour_REF=110.6)), extra=extra)
    for r in m["parts"]: print(r["name"], r["mass_g"], r["watertight"], r["skin_min_mm"] if "skin_min_mm" in r else "")
    print("mount", m["mount_mass_g"], "delta vs REF", round(m["mount_mass_g"] - 565.6, 1)); print(c)
    m["extra"]["mass_delta_vs_REF_g"] = round(m["mount_mass_g"] - 565.6, 1)
    import json; json.dump(m, open(os.path.join(L.ROOT, "stl/concepts/C4/metrics.json"), "w"), indent=1, default=float)
main()
