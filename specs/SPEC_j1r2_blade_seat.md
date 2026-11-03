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
Stage clearance (hard ranges): S1' group 7.5 mm, arm vs dish 5.7, vs liner 7.1. Clasp bracket vs arm sweep 2.0 mm inside the passive box +-5 deg (target 3, failure < 1): open, CLOSED 2026-11-03 (see section below).
Skin: dish min normal gap 5.2 mm (x-offset liner thins on slopes), pad 10.5; liner 0 by design (-1.8 = precompression). Rule for MJ11 = 5 mm, no violations. Wearer ROM: no first contact (arm abduction/flexion/scaption, neck). Fixed depth behind back 18.7 mm.
Workspace (real-parts sweep, j1r2 method): reach-free 0.786, passive box 0.466 (S1' -95..5, S2 -15..17.4) vs J1 rev 2 0.898 / 0.776. S2 cannot go past park: the body and rail block forward swing from a mid-back base.
Position trade (real parts, box cov / reach): (-60,-190) .47/.79; (-45,-140) .52/.83; (-30,-140) .59/.85; (-15,-165) .65/.88. More lateral = more workspace but the dish edge reaches y +36..+51 (wearer's arm), and the pad then needs a rail tab. Files: stl/j1r2_mounts/mj11_possweep.jsonl.
## Open items
1. Gravity torque unchanged (S1' 7.31, S2 6.59 N.m, SF 1.14/1.26): no balancer (round-2 status), needs Forest's decision.
2. Skin shear: skin slides 56-96 mm under the liner; friction estimate depends on strap preload and slip-fabric mu (PTFE ~0.1, assumption). Electrode flex leads exit via the 3 channels: not designed in detail; the pad covers rhomboid/infraspinatus/lower-trap sites.
3. Fairing not re-fitted (excluded from render and sweep). Cable run, firmware tables, harness_loads re-run, thermal not done.
4. Rail bosses, AK70 flange, foam/fabric grades PLACEHOLDER. Normal-offset (not x-offset) liner would keep 8 mm on slopes.
Code: cad/j1r2/mj_concepts3.py, mj11_scan.py, mj11_sweep.py, mj11_possweep.sh; mj_frame/mj_eval/mj_render patched (J1_PAD=y1,z1,standoff env). Run: J1R2_ROOT=<root> J1_PAD=-60,-190,12 python3 mj_eval.py MJ11.

## Clasp bracket re-fit (2026-11-03, SHOULDER/CLASP)
Search (cad/j1r2/mj11_clasp_search.py; feet F5/F4 -> hub F1 bars 16x6 and the ~270 deg clasp coverage unchanged): hub -> pad-rim bar width 14..24, anchor psi 110..130 (grid) / 100..140 (coarse), thickness 5/6/8, via points not needed. Gates: arm sweep (S1' group + arm) over the passive box +-5 deg hard-stop expansion (S1' -100..10, S2 -20..22.4, 2.5 deg steps) >= 3 mm; wing rail and spine bridge >= 3 mm; skin >= 5 mm (MJ11 rule).
The binding pair is the S1' group (a body of revolution about the S1' axis, so the gap is constant in S1'): the bar's pad-rim end sits 2.0 mm from it at width 24. Narrowing frees the margin; psi trades it against the rail (psi >= 130 loses the rail gate, psi <= 110 loses the S1' gate).
| bar (psi, w x t) | arm sweep | rail | spine bridge | skin |
| 120, 24x6 (old) | 2.0 | 4.1 | 22.3 | 5.4 |
| **119, 16x6 (new)** | **3.8** | **6.2** | **26.2** | **5.4 (5.3 with 1.5 mm sampling)** |
Neighbours psi 118 / 120 at 16x6: 3.78 / 3.83 arm, 6.9 / 5.9 rail. 14x6 would give 4.1 arm but is not needed. Bar stress (frame model) 2.3 / 2.4 MPa park / reach (was 1.7 / 1.9; PA-CF is ~100 MPa); mount mass 182 g (bracket 19.8 g, was 24.3); tip sag 0.91 / 1.86 mm (was 0.85 / 1.81); M4 bolt worst 236 / 319 N (was 231 / 312). First contact of the bracket with the S1' group at S1' = +13 deg (hard stop at the box edge 5 + 5 = 10 deg).
Real-parts sweep (mj11_sweep.py): passive box UNCHANGED, S1' -95..5, S2 -15..17.4, coverage 0.466. Reach-free coverage reads 0.706 (was 0.786). This is a correction, not a loss of workspace: with the old 24 mm bar the pair M1|clasp_bracket was closer than the 3 mm gate at park (2.05 mm), so the sweep classified it as designed contact and excluded it; at 16 mm it is 3.8 mm at park, so the pair is now checked and its frontier (S1' >= 13 deg, outside the box) removes reach cells at S1' 20..40 deg. Mount stays within the box rule of the spec.
Files: stl/j1r2_mounts/MJ11/MJ11_clasp_bracket_{R,L}.stl, clasp_search.json, metrics.json, sweep.json; renders_j1r2/mounts/MJ11_*.png. Env overrides: MJ11_CLASP_PSI / MJ11_CLASP_W / MJ11_CLASP_T. mj_concepts.clasp_bracket_geo/frame take w, t (defaults 24, 6: REF and other concepts unchanged).
Not done: clasp bracket vs the (unrebuilt) fairing, full-arm sweep / firmware tables, wearer clasp fit with the new bar width on the real strap (the 16 mm bar is narrower than the 24 mm bar the hub was drawn for: check the hub joint, UNVERIFIED).
