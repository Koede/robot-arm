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
3. Fairing not re-fitted (excluded from render and sweep). Cable run, firmware tables, harness_loads re-run, thermal not done.
4. Rail bosses, AK70 flange, foam/fabric grades PLACEHOLDER. Normal-offset (not x-offset) liner would keep 8 mm on slopes.
Code: cad/j1r2/mj_concepts3.py, mj11_scan.py, mj11_sweep.py, mj11_possweep.sh; mj_frame/mj_eval/mj_render patched (J1_PAD=y1,z1,standoff env). Run: J1R2_ROOT=<root> J1_PAD=-60,-190,12 python3 mj_eval.py MJ11.

## Clasp bracket re-fit (2026-11-03, SHOULDER / CLASP)
Problem: the 2026-11-02 bracket (hub -> pad-rim bar 24 x 6, anchor r 58, overhang 6, psi 120) was 2.03 mm from the S1' group over the whole passive box +-5 deg. Cause: the bar slopes rearward toward the pad (hub x -126, anchor x -94.5), so its tip reached x -89.3 while the S1' stop-ring front face (r 44.6..60) sits at x -87.3. The pair M1 | bracket was also "designed contact" at park (< 3 mm) and so silently excluded from the sweep gates.
Search (cad/j1r2/mj11_bracket_search.py, log stl/j1r2_mounts/MJ11/bracket_search.jsonl, 220 candidates): width 14/18/20/24, psi 100..140, anchor radius 58..66, overhang 0..6, thickness 5/6, optional via. Feet F5/F4/F1 and the ~270 deg coverage unchanged (same foot bars).
 * psi <= 110: bar hits the S1' stop ring (0..0.9 mm). psi >= 130: bar closes on the rail strip (< 1 mm). Window psi 116..124.
 * Anchor radius is the lever on M1 (moving the tip off the ring face): r 58 -> 2.0 mm, r 66 -> 4.8..5.3 mm. Width and overhang barely matter; thinner t = 5 buys +0.5 mm but costs 42 % bending stiffness, so t stays 6 and width stays 24.
Chosen: psi 119, anchor r 66 (pad rim), overhang 6, bar 24 x 6 PA-CF, via none (a via point never helped: the hub -> rim straight bar is already the shortest path clear of the blade). Defaults in mj_concepts3.py (MJ11_CLASP_PSI / _W / _T / _RA / _EXT / _VIA env override).
Results (gate 3 mm parts, 5 mm skin MJ11 rule), passive box +-5 deg = S1' -100..10, S2 -20..22.4:
 * bracket vs S1' group (M1) 4.82 mm (was 2.03; now a gated pair, no longer designed contact); vs arm (M2) 17.0; vs wrist/hand (M3) 18.8.
 * vs wing rail strip 5.1 mm (was 4.1..4.2), vs spine bridge 20.1 mm; skin gap blade 5.4, rail 21.8 mm (rule 5 mm: no violation). Bracket to liner 5.6 mm.
 * Real-parts passive box unchanged: S1' -95..5, S2 -15..17.4, coverage 0.466 (not below 0.466).
 * Frame: mount mass 186.2 g (bracket 23.8 g), tip sag 0.84 / 1.80 mm, clasp_seat stress 1.75 / 1.82 MPa: unchanged.
New finding (previously masked by the designed-contact exclusion): the S1' group first reaches the 3 mm gate on the bracket at S1' = +13.4 deg (1 mm at 17.6, touching at 19.6), i.e. 3.4 deg beyond the +10 deg hard stop. Reach-free coverage reads 0.706 (was 0.786 with the pair excluded). The firmware S1' keep-out must carry +13 deg (not the old soft 30) for this pad.
Lap with the pad rim is 815 mm3 (was 1433): the bar tip now overhangs the rim; the bar-to-pad fastening (lap + 2 x M3, PLACEHOLDER) is not designed.
