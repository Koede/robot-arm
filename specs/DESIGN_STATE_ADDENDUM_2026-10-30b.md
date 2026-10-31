# DESIGN_STATE addendum, 2026-10-30b: armoured pauldron (SHOULDER / WING)

Paste into DESIGN_STATE.md: §5 (compact faceted shoulder → pauldron), §10 (gates), §11 (open items), §12 (decision log), §13 (changelog). This follows the 2026-10-22 addendum practice; DESIGN_STATE.md itself is not edited.

- Repo: github.com/Koede/robot-arm, branch `claude/armor-pauldron-2026-10-30` (not merged; Forest to review).
- Spec: `specs/SPEC_shoulder_housing.md` rev 2026-10-30.

## Request (Forest, 2026-10-30)
- "Let's focus on refining the shoulder mount. It shouldn't be a shelf, it should be more like a shoulder piece of a knight's armor."
- Then: "Rethink with opus, really make smart ergonomic choices that are also aesthetically cool."

## What changed
- **Pauldron (group F) rebuilt as plate armour, three PA-CF parts** (`cad/pauldron.py` rev 2026-10-30, SDF CAD-as-code):
  - **crown**: domed cop, rolled bezel, two flared and bulged stepped lames, ears for the spaulder; 61.5 g.
  - **keel**: two stepped, fluted lames tapering into the plate; plate flange + 6 insert bosses on the old foot holes; 74.3 g.
  - **spaulder**: three sculpted lames from the crown onto clasp seats F1/F4; rolled and curled edges; rear edge scalloped round the socket; front cut back along Forest's own arm and neck motion envelope; 35.6 g.
  - **Total 171.4 g**, against the 182 g pauldron the harness model already carries. Harness pressures do not rise; top pad ≈ −0.03 kPa.
- **Interfaces kept:**
  - plate feet: same 6 hole positions;
  - clasp seats F1/F4: same centres and normals, M4 × 12; the pad is now Ø24 round on the Ø24 clasp boss (was 24 × 24 square);
  - socket rim z −54 / bore r 66;
  - assembly order (crown slides on along −x before the AK70 S1, as the old core did).
- **Plate (`chassis_axilla.py`): six pauldron-foot holes changed from tapped 3.3 to Ø4.5 clearance + 90° countersink Ø9.0.** The old note said "screws from the plate front" into tapped holes, which cannot clamp. Now ISO 10642 M4 × 12 runs from the plate front into M4 heat-set inserts in the keel. Plate volume −0.65 cm³; everything else unchanged.
- **Old parts** (core / arm_lat / arm_med, rev 2026-10-13 and 2026-10-23) are archived in `stl/archive/pauldron_rev2026-10-23/`. The rev 2026-10-23 lateral arm had shrunk to 3.5 cm³ and no longer reached the F4 seat.
- **New scripts:**
  - `cad/verify_armor.py`: gates; `ROM_ONLY=1` runs the range-of-motion study only.
  - `cad/scene_shoulder.py`: three.js shoulder renders; `CLOSE=1` for the close-up.
- **Renders:** `armor_close_sheet.png`, `armor_full_sheet.png`, `armor_before_after.png` (repo root).

## Gates (`stl/shoulder/pauldron/verify_armor.json`): ALL PASS
| Gate | Result |
|---|---|
| Static clearances | Stage contents ≥ 3.1 mm. Plate 3.5 (keel 5.5 outside its bolted flange). Clasp ≥ 4.1 outside the seats. Wing ≥ 8.7. Spine bridge ≥ 52. Skin: crown 50.8, keel 35, spaulder 16.7 (rule 10). Keel lip in crown hem 0.79 (designed). |
| S1 sweep, hard 5..150 (M1 + balancer M1) | crown 4.33 / keel 37.0 / spaulder 5.58 mm |
| Arm sweep, soft box (M2 / M3 + balancer M2) | crown 18.0 / keel 40 / spaulder 8.9 mm (old pauldron 19.4) |
| Wearer ROM: arm, GH share to 110° | Abduction and flexion: no contact. Scaption: 107.5° at the F4 seat, where the clasp boss under it is already reached at 95°. No new restriction. |
| Wearer ROM: neck | Lateral flexion first contact 50.0° (old 50.0, normal 42 ± 8). Rotation 90°, flexion 70°, extension 70°: no contact. |
| Walls (ray test) | p5: crown 1.41, keel 1.43, spaulder 1.56 mm (nominal 1.6 / 1.65). Crown p1 1.04 at hem / riser edges. |
| Print | All watertight, all inside the 256 cube. Best orientation and support area per part are in the JSON (crown −x up, 55 cm²; keel +x up, 12 cm²; spaulder +x up, 16 cm²). |
| Mass | 171.4 g ≤ 182 g |
| Assembly paths | Crown −x slide: 3.1 mm. Keel +z rise: 0.49 mm. Spaulder −z drop: 2.2 mm. |

**Method notes.**
- **Wearer ROM model:** rigid rotation of the MakeHuman arm about the glenohumeral centre. The range is the GH share of full elevation, 110° (GH:ST = 1.7:1, McClure 2001, already in §4 sources; normative abduction 171° / flexion 165°, Acta Med Iran 2005). The head rotation is blended over the cervical span. Neck norms: Apti et al. 2023, n = 300 (lateral flexion 42 ± 8°, rotation 71 ± 10°, flexion 58°, extension 59°).
- **Why not 180°:** the first run used GH-only rotation to 180°. That drove the deltoid skin through the acromion and severed the F4 seat. The scapula-relative envelope is the right one for armour seated on the acromion.

## Decisions
- **Claude (2026-10-30), pending Forest's veto:**
  - three-part plate-armour architecture, sculpted rather than articulated lames;
  - mass held at or below the 182 g harness budget;
  - spaulder outline cut by the wearer-ROM envelope (110° GH, neck tilt 50°);
  - no haute-piece beside the neck;
  - seat pads Ø24 round;
  - plate holes changed to clearance + countersink.
- **No standing decision reversed:** ball-and-socket styling kept (the crown frames the turning socket), nothing rigid on moving skin, no backplate, no hip belt, clasp 270° unchanged, DOF and actuator count unchanged.

## Open items (new)
1. **Clasp F4 node vs overhead reach (existing, not changed).** In the GH-share model the clasp's F4 boss on the acromion is reached at ~95–102° GH (~150–160° total elevation). The armour adds nothing beyond it, but the clasp node itself limits overhead reach. Check on the first fit (I7).
2. `sweep_full.py` (both modes) and the firmware keep-out tables with the new pauldron for the elbow, wrist and hand groups. The S1 / M2 / M3 gates pass in `verify_armor.py`.
3. **PLACEHOLDER:** the S1 bundle port in the keel (22 × 12 slot, medial) waits for the measured AK70-10 cable exit.
4. **Bench:** M4 / M3 heat-set pull-out in annealed PA-CF; keel lip slip fit on the first print.
5. The repo's DESIGN_STATE.md is a reconstruction. Its load-path line said "hip belt"; corrected (no backplate, no hip belt is a standing decision).
6. **GitHub:** `gh` CLI token invalid. The branch is pushed with git; the PR was not opened.
