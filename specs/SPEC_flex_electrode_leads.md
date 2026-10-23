# SPEC: Flexible electrode leads, rigid rail removed over the electrodes (rev 2026-10-23, target: SENSING + WING + ELECTRONICS, cross-subsystem)

Decision: Forest, 2026-10-23. "Remove the 3D printed skeleton over the electrodes. Electrodes become translation invariant: the chassis carries plug-in jacks, a flexible cable runs from each jack to its electrode wherever it is placed. Keep the electrodes where they are for now. Continue with the flexible wires and the chassis without the exoskeleton over the electrodes."

Status: DECIDED, NOT BUILT. No CAD has been changed in this revision. The gates in section 6 have not been run. Supersedes `SPEC_rail_facing.md` section 2 (stack), 2.1 (lead slot, medial skive), 4 and 5 as they apply to the rail. Keeps `SPEC_rail_facing.md` section 5 electrode contact budget and section 7 acceptance tests, reworded for a cable.

Units: mm, kPa, N, J. y_rel = y + 150 (distance from the spine plane, lateral positive); z from P.

## 0. What is removed, kept and added

| Item | Action | Where it lives now |
|---|---|---|
| Wing rail over the three under-rail pairs (trap_mid, rhomboid, trap_low), `scapular_frame.py` RAIL y_rel 12..50, z -440..-80 | Removed | the wing has no rigid material over the electrode zone |
| Rail foam stack: 6.0 mm firm, 10 to 6 mm relief ramp, PTFE facing 0.13 mm | Removed (not needed without the rail) | - |
| Rail lead slot 2.5 x 14 mm through the wall, lead shroud on the AFE pod | Removed | - |
| Electrode positions (atlas rev c: trap_mid, rhomboid, trap_low pairs; all others) | Kept | `stl/shoulder/emg_atlas.json`, unchanged |
| Compression base-layer panel (tricot) carrying the electrodes | Kept, now the only thing between the electrode and the skin | garment |
| Electrode snaps (2 mm touch-proof) | Kept | electrode end of each lead |
| Flexible lead run per electrode pair | Added | section 2 |
| Plug-in jacks on the chassis (keyed headers) | Added | section 3 |
| AFE pods (currently on the rail at z -320) | Relocated to the still zone (section 4) | spine bridge |

Electrodes are translation invariant: an electrode can move anywhere along the base-layer panel and the lead still reaches a jack, because the lead is a flexible run with service slack (section 2.3), not a rigid housing with a fixed foam footprint.

## 1. Consequence for the harness load path (the number that matters most)

The rail carried part of the wing's load into the back: harness model rail peak 12.7 kPa at the 5.79 kg per side mass (`verify_harness`, rev f2). Removing it reroutes that load. The rev f2 top pad margin is only 2.0 % (14.4 vs 14.7 kPa). Expected outcome: the top-pad gate fails unless the reroute goes somewhere that reduces the top pad. This is not computed. The 2-D `harness_loads.py` model and the 3-D `body_loads.py` (rail friction rows) must be re-run with the rail removed before anything else is locked.

Options for the reroute, to be decided by the re-run (not by assumption):
- A (test first): the wing stays on the still paraspinal strip and spine bridge only. Load goes to the paraspinal pads and the clasp. Expect higher F1/F4 and the strap; test the top pad first.
- B: a small rigid pad on the still strip only (y_rel <= 12, below the electrode zone) if A fails the top-pad or paraspinal gate. A pad at y_rel <= 12 is outside the trap_mid medial disc overhang and is allowed by the contact rule (still zone).
- Not allowed: any rigid pad over the electrode zone (y_rel 12..50, z -440..-80) or within 20 mm of an electrode. That is the rule this decision enforces.

## 2. Flexible lead run

### 2.1 Conductors (requirement; product not selected)
- Per electrode: a shielded 3-conductor flexible lead (signal, reference / DRL, shield), 28 AWG stranded, silicone jacket, at least 100 % elongation capability at the electrode end (stranded and laid, not solid).
- Shield driven by the bias amp, never tied to the motor bus (unchanged rule, `SPEC_electronics.md` section 7).
- Electrode end: 2 mm touch-proof snap (unchanged). Jack end: keyed connector (section 3).

### 2.2 Routing
- Leads run on the outside of the base-layer panel, stitched or laminated flat, never across the rail's old edge (the rail is gone, so the rule becomes: never over the electrode zone's rigid neighbours).
- Minimum 50 mm from any motor or power conductor and from the battery lead (project rule, `SPEC_electronics.md` section 7.1). Recheck the battery lead at x -30 after the pods move (section 4).
- Bend radius at least 10 x lead OD at every bend, including the service loop.

### 2.3 Service slack (sets the cable length)
The electrode moves with the skin; the jack is fixed on the still zone. The lead sees the full skin travel at its electrode end. Skin travel at full elevation (DESIGN_STATE section 4; follow fractions are assumptions):

| Electrode site | Skin travel (mm) | Slack required, 1.25 x travel (mm) |
|---|---|---|
| trap_mid | 56 | 70 |
| rhomboid | 62 | 78 |
| trap_low | 76 | 95 |
| infraspinatus | 70 | 88 |
| lateral border | 96 | 120 |

Slack is an accordion loop (4 half-turns, +/- 18 mm) or a serpentine on the base layer, sized from the table. Worst case for this electrode set: 95 mm (trap_low). The old rail design gave the under-rail leads at least 90 mm slack (`SPEC_rail_facing.md` section 2.1 fallback), so this is the same order and not a new constraint.

Any electrode moved to a new site uses its own row. The loop is re-sized from the table, not reused.

### 2.4 Fatigue
At least 5 000 strokes over the full travel with resistance change below 10 % and no open circuit (acceptance T14-3d, unchanged).

## 3. Chassis jacks

- Form: one keyed 16-way header per 16 channels, so 3 per side for 23 pairs (matches `SPEC_electronics_fit.md` section 5 and `bom.csv`). Pinout per header in `SPEC_electronics.md` (to be written for the flex run).
- Mounting: on the still zone (spine bridge or the paraspinal strip, never over the blade), on a 2-boss jack plate printed in PETG-CF. Heads recessed; strain relief on each lead at the jack.
- Count vs spare: 23 channels per side plus one spare ADS input (`SPEC_electronics.md` section 6). Spare pins are free on the 48-way set.
- Placeholder: the exact header part number and pinout are not selected. Mark as PLACEHOLDER in code and spec until selected.

## 4. AFE pods relocation (requires CAD and a fit check)

The AFE pods were at z -320 on the rail, standoff 11.6 mm, with a lead shroud over the slot. With the rail gone they have nothing to stand on and sit over the moving blade. Decision: move both AFE pods to the spine bridge (still zone).
- Requirement: skin gap at least 10 mm, clearance at least 10 mm to the battery box and cradle, at least 50 mm from the battery lead, at least 3 mm to all moving groups (arm sweep, `pods_arm_sweep.py`).
- The pods' leads are now the flex runs of section 2, so the pod's lead shroud is replaced by the jack headers of section 3.
- Position is a target. The spine bridge has the main box (3.27 mm clearance to the spine bridge, `SPEC_electronics_fit.md`). The AFE pods will not fit beside it without a re-fit. Open: a fit check is needed; this is the first CAD task of the next session.

## 5. Superseded items (for the record)

- `SPEC_rail_facing.md` section 2 stack, 2.1 edges and lead slot, 3 lambda argument (fabric vs film), 4 friction (sliding facing): not applicable without the rail. The base-layer panel now follows the skin directly.
- `SPEC_rail_facing.md` section 5 contact budget: the collar at most 1.5 kPa on the two medial discs and 2 kPa elsewhere still hold (electrode contact is a garment property). The rail load term (2.6 kPa within 17 mm) is removed; the within-17 mm band budget now relies on the collar alone, which needs a T14-3e re-measure.
- `SPEC_harness.md` section 2 wing zones: the rail row is removed; the bridge and tongue rows remain.
- `DESIGN_STATE.md` section 6 base-layer sites: unchanged (positions kept).

## 6. Gates for this change (not yet run)

| Gate | Result | Status |
|---|---|---|
| Skin gap per zone (at least 2 mm; wing on still zone only) | not computed | open |
| Arm sweep vs wing, pods, flex runs (at least 3 mm) | not computed | open |
| Harness: top pad at most 14.7 kPa, rail term removed, paraspinal strip (DRL) | expected to fail on the top pad (2.0 % margin before) | open, key |
| Body loads 3-D with rail removed (`body_loads.py`, rail rows off) | not run | open |
| Within-17 mm band at most 4.3 kPa (collar only) | not run | open, needs T14-3e |
| Pod clearances after relocation (`_pods_overlap_all.py`) | not run | open |
| Lead separation at least 50 mm from motor and battery wiring | not run | open |
| Lead fatigue and impedance (T14-3c, T14-3d) | bench | open |
| Print table (wing now without rail; new jack plate) | not run | open |
| Mass and CG (wing mass falls; pods and jacks added) | not run | open |

## 7. What must be done next (order)

1. Restore the pipeline in the workspace (`cad/*.py` from this repository, MakeHuman base mesh, cadquery install per README).
2. Run `harness_loads.py` and `body_loads.py` with the rail removed (no CAD change needed for this run: set the rail rows off). This decides option A or B in section 1 before any geometry is cut.
3. Cut the rail in `scapular_frame.py` and remove the lead slot. Re-run `scapular_frame` -> `electronics_fit` -> `sensor_pods` -> `_pods_overlap_all` -> `electronics_model` -> `pods_arm_sweep` -> `verify_harness` -> `print_table`.
4. Add the spine-bridge jack plate and the flex-run model (placeholder header, slack per section 2.3).
5. Renders of the posed arm, wing and base layer; send to Forest.

## 8. Open items and assumptions

- Electrode skin travel per site: the follow fractions are assumptions (DESIGN_STATE section 4). T14-3b measures them.
- Flex-cable product, header part and pinout: not selected; no purchase links this session.
- AFE pod position on the spine bridge: a target, not a fit.
- The rail's contribution to the load path is not known until step 2 of section 7 is run.
