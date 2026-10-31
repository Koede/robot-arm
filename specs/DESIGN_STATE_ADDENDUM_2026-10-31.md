# DESIGN_STATE addendum, 2026-10-31: flush armour, pauldron rev 2026-10-30b (SHOULDER / WING)

Paste into DESIGN_STATE.md: §3 (pauldron line), §10 (gates), §12 (open items 3, 4, 9), decision log, changelog. Follows the 2026-10-30b addendum; this supersedes its geometry and numbers.

- Repo: github.com/Koede/robot-arm, branch `claude/armor-pauldron-2026-10-30`.
- Spec: `specs/SPEC_shoulder_housing.md` §0, rev 2026-10-30b.

## Request (Forest, 2026-10-30)
"Wayyyyyyy too bulky, this thing should sit flush against the shoulder as much as possible."

## What changed
| | Rev a (2026-10-30) | Rev b (flush) |
|---|---|---|
| Armour mass | 171.4 g | **110.6 g** |
| Envelope (x × y × z) | 287 × 186 × 120 | **277 × 149 × 77** |
| Lowest point | z −163 | z −115 |
| Spaulder skin distance, median | 52.3 mm | 36.9 mm |
| Chassis plate | 337 g | **278 g** |

- **Crown** hugs the stage contents (convex hull plus bore, G_IN 3.5). Flare 1.2, no bulge, no ears. 28.7 g.
- **Keel** is a 14 mm dish under the S1 motor with 2 plate feet. The lip follows the crown flare with a 0.7 mm gap. 41.9 g.
- **Spaulder** is draped on the obstacle floor:
  - floor: skin + 10.5, clasp / wing / plate + 3.5;
  - smoothing: 36 mm grey closing, then a 7 mm Gaussian;
  - 1.5 mm steps;
  - 2 rear feet on the plate top edge.
  - 40.0 g.
- **WING interface change (chassis plate, `chassis_axilla.py` rev b; contract in `stl/shoulder/plate.json`):**
  - pauldron feet 6 → 2: M4 csk at (y, z) = (±50, −109);
  - new 2 × M3 tapped holes in the plate top edge at y ±40, x −84;
  - plate outline trimmed to 158 × 104.
  - S1 base bolts, wing bolts, clasp seats F1/F4 and socket rim are unchanged.
- **Walls 1.6 → 1.8 mm.** At 1.6 the flush shells failed the 1.4 mm p5 wall gate (1.37–1.39).
- **Verifier** (`verify_armor.py`): rev-b masks (spaulder rear feet seat on the plate, no ear posts), plus exact interference gates (manifold boolean volume = 0) for every armour pair and each part vs the plate. This gate caught an 8.6 mm³ keel-lip / crown-hem overlap in the first rev-b build; it is fixed.
- **New script:** `cad/scene_armor_only.py` (armour-only close-up renders).
- **Renders:**
  - `armor_flush_close_sheet.png`: six views with the body mesh;
  - `armor_flush_parts_sheet.png`: armour without the arm;
  - `armor_flush_before_after.png`: rev a vs rev b.

## Gates
`stl/shoulder/pauldron/verify_armor.json`. The full table is in SPEC §0.1.

**ALL PASS** (full table in SPEC §0.1):
- **Static clearance:** stage contents 3.08 / 3.15 / 12.09 mm (crown / keel / spaulder).
- **Interference** (exact boolean): 0 mm³ for every armour pair and each part vs the plate.
- **Spaulder:** clasp 4.43, skin 15.47 (rule 10), wing 5.79, plate outside the feet 4.12 mm.
- **Keel lip in the crown hem:** 0.46 mm.
- **S1 hard sweep:** crown 4.35, spaulder 4.18 mm.
- **Arm soft sweep:** crown 18.03, spaulder 4.18 mm.
- **Wearer ROM:** unchanged from rev a.
  - Arm abduction / flexion: no contact.
  - Scaption: 107.5° at the F4 seat (clasp boss 95°).
  - Neck lateral flexion: 50°.
  - Rotation / flexion / extension: no contact.
- **Walls p5:** crown 1.53, keel 1.54, spaulder 1.4 mm (gate 1.4; nominal 1.8).
- **Mass:** 110.6 g.
- **Assembly paths:** crown 3.15, keel 0.57, spaulder 4.42 mm.

## Harness
Per arm: −120 g (armour −61, plate −59) against the masses in the harness model. On the rev-a basis (p_top ∝ total mass, 5.875 kg), the top pad goes from ≈ 14.61 to ≈ 14.31 kPa, against the 14.7 kPa limit; the margin grows from 0.6 % to 2.7 %. This is an estimate: `harness_loads.py` is not in the repo.

## Decisions
- **Claude (2026-10-30b), pending Forest's veto:**
  - spaulder no longer bolted to the crown; it bolts to the plate top edge instead (removes the ears and their height);
  - 2 plate feet instead of 6;
  - plate trimmed;
  - 10 mm skin rule over the shoulder top kept.
- **For Forest:** the spaulder's height over the trapezius is now set by the 10 mm skin rule and the 3.5 mm clasp clearance, not by styling. Lowering the skin rule to 6 mm would bring it about 4 mm lower; that is his call.
- **No standing decision reversed.**

## Open items
1. `sweep_full.py` (both modes) and `gen_tables.py` on the rev-b armour (existing item 3).
2. `harness_loads.py` confirmation of the top-pad estimate (existing item 4).
3. Keel S1 cable port: still PLACEHOLDER (item 9).
4. **Bench:**
   - M3 tapped thread in the 6061 plate top edge (8 mm wall, the hole is centred on it, 2.75 mm each side of Ø2.5);
   - lip slip fit at a 0.7 mm gap;
   - insert pull-out (existing).
