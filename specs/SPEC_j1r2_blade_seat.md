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
3. (Fairing re-fitted 2026-11-03, see below.) Cable run, firmware tables, harness_loads re-run, thermal not done.
4. Rail bosses, AK70 flange, foam/fabric grades PLACEHOLDER. Normal-offset (not x-offset) liner would keep 8 mm on slopes.
Code: cad/j1r2/mj_concepts3.py, mj11_scan.py, mj11_sweep.py, mj11_possweep.sh; mj_frame/mj_eval/mj_render patched (J1_PAD=y1,z1,standoff env). Run: J1R2_ROOT=<root> J1_PAD=-60,-190,12 python3 mj_eval.py MJ11.

## Fixed fairing re-fit (2026-11-03, SHOULDER, unattended run)
Branch claude/mj11-fairing-2026-11-03. Code: `cad/j1r2/mj_concepts3.py` (`fairing_geo`, `fairing_clearance`), `cad/j1r2/mj11_fairing.py` (checks -> `stl/j1r2_mounts/MJ11/fairing.json`). Replaces the J1 rev 2 `fairing_fixed` (riser/lap cup) in `mj11_sweep.py` REPLACED and `mj_render.py` SKIP; two new MJ11 parts.
- **fairing_body** (PA-CF 1.8 mm, 28.4 g): front ring flush with the dish plane x0 (-97.5) wrapping the pad rim R66 (inner R66.2, outer R68.0); medial flat at the pad trim y -121 (wall inside the pad outline, so spine-bridge clearance stays 3.3 mm); wall sectors the S1' sweep leaves (slot swept -100..40 deg S1', 2.5 deg stations, 3.2 mm gap round the S2 clamp, its flange and the yoke bar); 3 pad tabs (psi 225/270/315, M3 x 8 into pad rear-face inserts, PLACEHOLDER); 3 cover lugs (psi 85/125/165, 45 deg cones); house-shaped window for the clasp bracket (psi 119, 39 mm wide, 3.7 mm clear); 4 notches at the rail-spacer ends (1.5 mm clear); cable gland D13.2 in the medial flat (axis y, x -51.5, PLACEHOLDER until the cable run is redone). No riser, no lap.
- **fairing_cover** (14.1 g): flat R68 disc at x -12.1..-10.3 (XO + 31), swept slot through it, 4 mm spigot into the body, 3 x M3 x 8.
- Liner lead channels (psi 225/270/315) and the dish are untouched: channels sit at x <= x0 - 10 inside the liner, the fairing starts at x0 (14.7 mm clear of the liner, 0.2 mm designed contact with dish and pad).
## Results
| check | value |
|---|---|
| mass | 42.5 g (body 28.4 + cover 14.1), mount total 229 g (was 187) |
| clearance to moving groups, S1' -100..40 x S2 -25..40 (slot range) | M1 3.20 mm (S2 clamp), M2/arm 21.3 mm |
| same, task box +-5 stops (S1' -85..10, S2 -25..40) | M1 3.20, M2 21.3 mm |
| rev 2 wide hard ranges (S1' -95..35, S2 -30..80) | M1 3.20, M2 16.1 mm |
| skin | body 18.8 mm, cover 96 mm (gate 2) |
| wing rail / spine bridge | 4.03 / 3.28 mm (gates 3) |
| stage / pad / bracket / spacers | AK70 10.7, stop posts 6.2, bracket 3.80, spacers 1.51 mm |
| sweep (mj11_sweep) | passive box unchanged S1' -95..5, S2 -15..17.4 (cov 0.466); reach-free 0.786 unchanged |
Design note: a slot swept only to S1' +10 cost reach-free coverage (0.786 -> 0.706) and 8 g more fairing; the slot is therefore cut over the full rev 2 hard range (-100..40).
Printability: body prints x0 face down (85.4 mm tall, 129 x 136 mm footprint, fits 256); no supports except: 3 pad-tab ledges (9.5 mm cantilevers, 596 mm2 total overhang incl. gland arch and bracket-window gable, all short bridges / 45 deg). The medial flat wall is held up by a 50 deg gable edge (apex 36 mm above the pad rear face) so it needs no support; consequence: a triangular medial opening (51 mm wide) exposes the pad rear and motor rim. Cover prints flat, rear face down, no overhang. Both watertight, one component each.
## Open
Pad-tab M3 inserts (pad is 6 mm: needs 3 mm short inserts), cover lug taps, gland position and cable run, tab-ledge print test; gable window vs a closed medial plate (separate flat part) is Forest's call. Fairing mass is lumped in the frame model (+43 g mount).
