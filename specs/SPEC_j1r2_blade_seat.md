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
- clasp bracket: feet to the pad rim at psi 120; hub->anchor bar 16 x 5 (rev 2026-11-03, see section below).
## Results (mj_eval, same harness as round 1/2)
Mount mass 187 g (pad 93, dish 57, liner 10, spacers 3, bracket 24); no balancer.
Hand-tip sag park / reach 0.85 / 1.81 mm (body-only 0.64 / 1.16); REF 10.4 / 18.2, MJ4 2.4 / 5.3. Pad tilt 0.15 deg. Liner pressure increment <= 2.2 kPa; rail unload -3.1 kPa.
M4 bolt worst 231 N park / 312 N reach (typical insert pull-out 1 kN UNVERIFIED: SF 3.2, 1.6 with x2 dynamic).
Stage clearance (hard ranges): S1' group 7.5 mm, arm vs dish 5.7, vs liner 7.1. Clasp bracket clearance fixed 2026-11-03 (below).
Skin: dish min normal gap 5.2 mm (x-offset liner thins on slopes), pad 10.5; liner 0 by design (-1.8 = precompression). Rule for MJ11 = 5 mm, no violations. Wearer ROM: no first contact (arm abduction/flexion/scaption, neck). Fixed depth behind back 18.7 mm.
Workspace (real-parts sweep, j1r2 method): reach-free 0.786, passive box 0.466 (S1' -95..5, S2 -15..17.4) vs J1 rev 2 0.898 / 0.776. S2 cannot go past park: the body and rail block forward swing from a mid-back base.
Position trade (real parts, box cov / reach): (-60,-190) .47/.79; (-45,-140) .52/.83; (-30,-140) .59/.85; (-15,-165) .65/.88. More lateral = more workspace but the dish edge reaches y +36..+51 (wearer's arm), and the pad then needs a rail tab. Files: stl/j1r2_mounts/mj11_possweep.jsonl.
## Open items
1. Gravity torque unchanged (S1' 7.31, S2 6.59 N.m, SF 1.14/1.26): no balancer (round-2 status), needs Forest's decision.
2. Skin shear: skin slides 56-96 mm under the liner; friction estimate depends on strap preload and slip-fabric mu (PTFE ~0.1, assumption). Electrode flex leads exit via the 3 channels: not designed in detail; the pad covers rhomboid/infraspinatus/lower-trap sites.
3. Fairing not re-fitted (excluded from render and sweep). Cable run, firmware tables, harness_loads re-run, thermal not done.
4. Rail bosses, AK70 flange, foam/fabric grades PLACEHOLDER. Normal-offset (not x-offset) liner would keep 8 mm on slopes.
Code: cad/j1r2/mj_concepts3.py, mj11_scan.py, mj11_sweep.py, mj11_possweep.sh; mj_frame/mj_eval/mj_render patched (J1_PAD=y1,z1,standoff env). Run: J1R2_ROOT=<root> J1_PAD=-60,-190,12 python3 mj_eval.py MJ11.

## Clasp bracket clearance fix (2026-11-03, SHOULDER / CLASP)
Problem: bracket (feet F5/F4 -> hub F1 -> pad rim, psi 120, 24 x 6 bars) was 2.0 mm from the moving stage inside the passive box +-5 deg. The offender was the S1' group (M1, ~constant over S1'), not the arm (M2: 17.0 mm).
Search (cad/j1r2/mj11_clasp_search.py, results stl/j1r2_mounts/MJ11/clasp_search.json): psi 105..140 step 2.5, hub->anchor bar width 14..24, thickness 4/5/6, clearance over S1' -100..10 x S2 -20..22.4 (2.5 deg). Feet positions, foot bars (16 x 6) and the ~270 deg coverage are unchanged. psi < 115 hits M1 (<0.5 mm); psi >= 130 loses the rail gate (width 24: 0.5 mm); skin gap (5.3 mm) is set by the foot bars and is flat in the search. Via points not needed.
Chosen: psi 120 (unchanged), hub->anchor bar 16 x 5 (MJ11_CLASP_W / MJ11_CLASP_T in mj_concepts3.py).
| gate | before | after | need |
|---|---|---|---|
| bracket to S1' group, passive box +-5 | 2.03 | 4.05 | >= 3 |
| bracket to arm sweep (M2) | 17.0 | 17.0 | >= 3 |
| to wing rail / spine bridge | 4.2 / 22.3 | 6.2 / 25.5 | >= 3 |
| skin gap (blade) | 5.4 | 5.3 | >= 5 |
Bracket mass 24.3 -> 18.4 g (mount 180.8 g). Frame: member stress <= 2.84 MPa (clasp bars), hand-tip sag park / reach 0.93 / 1.87 mm (was 0.85 / 1.81), M4 bolt worst 238 / 321 N (was 231 / 312).
Real-parts sweep (mj11_sweep.py): passive box UNCHANGED, S1' -95..5, S2 -15..17.4, coverage 0.466. Reach-free coverage reads 0.706 (was 0.786): not a regression of the bracket. At 24 mm the S1' group sat 2.5 mm from the bar at park, under the 3 mm gate, so the pair was excluded as designed contact and M1 contact at S1' >= ~15-20 deg was hidden. At 16 mm the pair is live and the frontier shows it (outside the passive box; hard stop at 10 deg). Open: if S1' > 15 deg is wanted, move the anchor or re-check the S1' yoke vs bar there.
