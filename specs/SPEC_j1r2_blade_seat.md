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
Stage clearance (hard ranges): S1' group 7.5 mm, arm vs dish 5.7, vs liner 7.1. Clasp bracket vs arm sweep: see the 2026-11-03 section (fixed, 4.7 mm).
Skin: dish min normal gap 5.2 mm (x-offset liner thins on slopes), pad 10.5; liner 0 by design (-1.8 = precompression). Rule for MJ11 = 5 mm, no violations. Wearer ROM: no first contact (arm abduction/flexion/scaption, neck). Fixed depth behind back 18.7 mm.
Workspace (real-parts sweep, j1r2 method): reach-free 0.786, passive box 0.466 (S1' -95..5, S2 -15..17.4) vs J1 rev 2 0.898 / 0.776. S2 cannot go past park: the body and rail block forward swing from a mid-back base.
Position trade (real parts, box cov / reach): (-60,-190) .47/.79; (-45,-140) .52/.83; (-30,-140) .59/.85; (-15,-165) .65/.88. More lateral = more workspace but the dish edge reaches y +36..+51 (wearer's arm), and the pad then needs a rail tab. Files: stl/j1r2_mounts/mj11_possweep.jsonl.
## Open items
1. Gravity torque unchanged (S1' 7.31, S2 6.59 N.m, SF 1.14/1.26): no balancer (round-2 status), needs Forest's decision.
2. Skin shear: skin slides 56-96 mm under the liner; friction estimate depends on strap preload and slip-fabric mu (PTFE ~0.1, assumption). Electrode flex leads exit via the 3 channels: not designed in detail; the pad covers rhomboid/infraspinatus/lower-trap sites.
3. Fairing not re-fitted (excluded from render and sweep). Cable run, firmware tables, harness_loads re-run, thermal not done.
4. Rail bosses, AK70 flange, foam/fabric grades PLACEHOLDER. Normal-offset (not x-offset) liner would keep 8 mm on slopes.
Code: cad/j1r2/mj_concepts3.py, mj11_scan.py, mj11_sweep.py, mj11_possweep.sh; mj_frame/mj_eval/mj_render patched (J1_PAD=y1,z1,standoff env). Run: J1R2_ROOT=<root> J1_PAD=-60,-190,12 python3 mj_eval.py MJ11.

## Clasp bracket re-fit (2026-11-03, branch claude/mj11-clasp-bracket-2026-11-03)
Problem: the 24 x 6 hub -> anchor bar was 2.0 mm from the S1' group (M1) inside the passive box +-5 deg (target 3, failure < 1). It was closest at the pad-rim anchor (x -89, y -82, z -151). Because 2.0 < the 3 mm gate at park, mj11_sweep had silently EXCLUDED the pair as a designed contact.
Search (grid: psi 100..140 step 5, width 14..24 step 2, thickness 4/5/6, anchor radius 50/58; feet F5/F4/F1 positions and the psi-120 anchor side of the ~270 deg clasp unchanged; feet bars stay 16 x 6; via points not needed): distance to M1 over S1' -100..10, S2 -20..22.4 (2.5 deg), rail/bridge distance, skin gap.
Trade: raising psi (130-140) or widening to 16 raises M1 clearance but the bar runs into the rail (0.2-1.9 mm at psi >= 130); psi < 115 hits the arm (< 0.5 mm). Best balance: **psi 120 unchanged, hub bar 14 x 6 mm** (CLASP_W env MJ11_CLASP_W, default 14; CLASP_T 6; CLASP_R 58).
Result (clasp bracket, inside the passive box +-5 deg): arm sweep M1 4.7 mm (was 2.0), M2 17.2, M3 18.8; rail 6.2 (was 4.1), spine bridge 26.0, skin gap 5.4 mm over the blade (rule 5). All three gates pass.
Real-parts sweep (mj11_sweep.py): passive box unchanged, S1' -95..5, S2 -15..17.4, coverage 0.466 (no shrink). reach-free coverage reads 0.706 instead of 0.786 only because the clasp pair is now counted (it was excluded before), not because anything got worse.
Frame: clasp_seat member stress 2.5 / 2.8 MPa park / reach (was 1.7 / 1.9; PA-CF, no concern). Bracket mass 18.7 g, mount mass 181 g (was 187). Tip sag 0.93 / 1.88 mm.
Note: M3 vs bracket still shows frontier contact (0.0 mm) at poses OUTSIDE the passive box; the box excludes them. Code: mj_concepts3.clasp3_geo, mj_concepts.clasp_bracket_frame(w, t).
