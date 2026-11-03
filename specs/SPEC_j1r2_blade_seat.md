# SPEC: MJ11 blade-seated pancake (SHOULDER / WING, concept), 2026-11-02
Request (Forest): "make the shoulder pancake mount/holder flat facing against the shoulder blade so the large flat bit is up against the back."
Decisions: Forest chose "rides on the blade" = OVERRIDE of "nothing rigid on skin that moves with the scapula" for this concept only; work read-only (no push). Nothing adopted; J1 rev 2 and MJ1..MJ10 untouched. Status CONCEPT.

## Why the pad could not just be pressed flat where it was
At the J1 rev 2 pad (+10, -30) the pad face is 50..67 mm (mean 62) off the skin: it sits above the shoulder top, and x0 is set by the S2 sweep (radius 154) crossing the blade bulge. A flat pad only lies on the back if the S1' axis moves onto the blade. The dish absorbs the skin shape, so flatness is not the limit; workspace is.

## MJ11 (pad y1,z1 = -60,-190; standoff 12)
- pad: R66 x 6 PA-CF disc, trimmed flat at y -121 (clears spine-bridge ears 3.3 mm). S1' AK70 on its rear face (PLACEHOLDER flange).
- seat dish: PA-CF shell, outer face the flat plane x0 = -97.5, inner face skin + 8 mm; thickness 3.1..35.7 mm (mean 14), height field, prints flat face down, no supports.
- liner: 8 mm closed-cell foam (0.12 g/cm3 assumed) + PTFE-fabric slip face, 3 lead channels (psi 225/270/315) for flex electrode leads.
- rail link: 4 x M4 through 2 spacer pairs (4.8 / 8.3 mm) straight into new rail bosses (PLACEHOLDER; WING contract change). No riser, knee or truss.
- clasp bracket: feet to the pad rim at psi 120.
## Results (mj_eval, same harness as round 1/2)
Mount mass 187 g (pad 93, dish 57, liner 10, spacers 3, bracket 24); no balancer.
Hand-tip sag park / reach 0.85 / 1.81 mm (body-only 0.64 / 1.16); REF 10.4 / 18.2, MJ4 2.4 / 5.3. Pad tilt 0.15 deg. Liner pressure increment <= 2.2 kPa; rail unload -3.1 kPa.
M4 bolt worst 231 N park / 312 N reach (typical insert pull-out 1 kN UNVERIFIED: SF 3.2, 1.6 with x2 dynamic).
Stage clearance (hard ranges): S1' group 7.5 mm, arm vs dish 5.7, vs liner 7.1. Clasp bracket vs arm sweep 2.0 mm inside the passive box +-5 deg (target 3, failure < 1): open.
Skin: dish min normal gap 5.2 mm (x-offset liner thins on slopes), pad 10.5; liner 0 by design (-1.8 = precompression). Rule for MJ11 = 5 mm, no violations. Wearer ROM: no first contact (arm abduction/flexion/scaption, neck). Fixed depth behind back 18.7 mm.
Workspace (real-parts sweep, j1r2 method): reach-free 0.786, passive box 0.466 (S1' -95..5, S2 -15..17.4) vs J1 rev 2 0.898 / 0.776. S2 cannot go past park: the body and rail block forward swing from a mid-back base.
Position trade (real parts, box cov / reach): (-60,-190) .47/.79; (-45,-140) .52/.83; (-30,-140) .59/.85; (-15,-165) .65/.88. More lateral = more workspace but the dish edge reaches y +36..+51 (wearer's arm), and the pad then needs a rail tab. Files: stl/j1r2_mounts/mj11_possweep.jsonl.
## Open items
1. Gravity torque unchanged (S1' 7.31, S2 6.59 N.m, SF 1.14/1.26): no balancer (round-2 status), needs Forest's decision.
2. Skin shear: skin slides 56-96 mm under the liner; friction estimate depends on strap preload and slip-fabric mu (PTFE ~0.1, assumption). Electrode flex leads exit via the 3 channels: not designed in detail; the pad covers rhomboid/infraspinatus/lower-trap sites.
3. Fairing re-fitted 2026-11-03 (section below). Cable run, firmware tables, harness_loads re-run, thermal not done.
4. Rail bosses, AK70 flange, foam/fabric grades PLACEHOLDER. Normal-offset (not x-offset) liner would keep 8 mm on slopes.
Code: cad/j1r2/mj_concepts3.py, mj11_scan.py, mj11_sweep.py, mj11_possweep.sh; mj_frame/mj_eval/mj_render patched (J1_PAD=y1,z1,standoff env). Run: J1R2_ROOT=<root> J1_PAD=-60,-190,12 python3 mj_eval.py MJ11.

## Fixed fairing re-fit (2026-11-03, branch claude/mj11-fairing-2026-11-03)
Code: `mj_concepts3.fairing_mesh / fairing_split` (parts `fairing_ring`, `fairing_hood`), checks `mj11_fairing_check.py` -> `stl/j1r2_mounts/MJ11/fairing_check.json`. The J1 rev 2 `fairing_fixed` / `fairing_lap` stay excluded (superseded).
Geometry: 1.8 mm PA-CF shell about the S1' axis, R66.2 inner / 68.0 outer (0.2 mm over the pad rim), trimmed flat at y -123.0 (pad trim -121 + 0.2 + 1.8), front edge flush with the dish plane x0 -97.5, cap at x -10.3 (XO + 31). No riser, no rail lap.
- S2 slot: swept volumes of the J1 rev 2 recipe (S2 clamp cylinder, yoke bar, clamp plate) at 5 deg steps over S1' -100..10 (union of the task box -80..5 and the sweep box -95..5, each +-5) with a 3.2 mm margin. Result: a closed hood on the medial/top side + a 10.5 mm lip ring all round; the lateral/lower side is open where the S2 stage swings.
- Clasp bracket window: bar hub->anchor dilated 3.5 mm (min distance 3.5). Cable gland D13.2 through the flat medial wall at x -78.5, z -202 (PLACEHOLDER route, grommet). 3 lead channels (psi 225/270/315) untouched: 10-11 mm from the fairing (channels sit in the liner, x < -107; fairing x >= -97.5).
- Medial setback: for y < -100 the front edge starts at x -92.5 instead of -97.5. Reason: spine-bridge ears reach y -124.3 at x <= -95.5, so a flush edge would be 1.3 mm from them; with the setback 3.8 mm. It also lifts the lip 5 mm off the rail spacers (0.3 -> 4.8 mm).
- Fixing: 3 x M3 x 8 radial, thread-forming into the pad rim at x0 + 3, psi 45 / 270 / 340 (PLACEHOLDER: pad holes not cut; load is the fairing weight only). Hood to ring: 4 mm inner-half tongue + rebate, 0.15 mm gap, 2-part epoxy / MMA.
- Split for printing: one piece cap-down needs support under the lip arc (486 mm2 overhang, a 1.8 mm lip 77 mm up in air); mouth-down the cap roof is unsupported. Ring + hood printed apart need none: hood cap down (bed 7611 mm2, 19 mm2 of tongue-edge overhang), ring standing on its rear edge (103 mm2 = 0.9 mm rebate ledge, 50% supported). Both far inside 256 mm (ring 10.5 x 131 x 136, hood 80.7 x 95.2 x 100.2). Walls 1.8 mm (4 perimeters), PA-CF solid, density 1.15 g/cm3 (as mj_eval).
Results: mass ring 7.0 + hood 41.9 = 48.9 g; MJ11 mount 187 -> 236 g (no balancer). Watertight, 1 component each.
Clearances (mm; gates 3 moving/parts/harness, 2 skin): S1' group (M1) 3.2 over S1' -100..10 (contiguous free S1' -102..+12 from park); S2 stage/arm (M2) 21.3 over S1' -100..10 x S2 -25..40 (5 deg); skin 23.4; wing rail 8.9; spine bridge 3.8; clasp bracket 3.5; rail spacers 4.8; S1' AK70 16.7; S1' stop posts 6.2; liner 15.9; designed contact pad / dish 0.2 (radial rim gap).
Sweep (mj11_sweep.py, real parts incl. fairing): passive box unchanged, S1' -95..5, S2 -15..17.4, cov 0.466; reach-free coverage 0.786 -> 0.706 (the hood now stops S1' beyond -102/+12; the hard stops are inside, so no usable range is lost).
mj_eval stage clearance had excluded everything within R68.5 of the S1' axis (the J1 rev 2 pad zone), which hid the fairing; fixed (fairing parts are checked in full), and MJ11 now uses its own hard ranges (S1' -100..10, S2 -25..40). Open: clasp bracket vs arm sweep 0.2 mm at S1' 10 / S2 35 (known; inside the passive box 2.0 mm, outside S2 17.4 + 5 it fails) -- not the fairing.
Still to do: fairing STL fit with the real AK70 connector/flange, cable run through the gland, thermal (closed hood over a 621 g motor: no vents yet), fairing screw holes in the pad.
