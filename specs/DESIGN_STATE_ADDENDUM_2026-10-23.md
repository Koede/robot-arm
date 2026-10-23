# DESIGN_STATE addendum, 2026-10-23 (paste into DESIGN_STATE.md section 11 and the changelog; DESIGN_STATE.md itself not edited, per the 2026-10-22 addendum practice)

Subsystem: cross-subsystem integration (SENSING electrodes + WING rail + ELECTRONICS jacks + HARNESS load path).

## Changelog
- 2026-10-23: Forest's decision. Remove the rigid wing rail (and its foam / facing stack and the 14 mm lead slot) over the electrodes at y_rel 12..50, z -440..-80. Electrodes stay where they are (atlas rev c, unchanged). Electrodes become translation invariant: chassis keyed jacks, flexible shielded leads to each electrode, service slack per site (70 / 78 / 95 / 88 / 120 mm for trap_mid / rhomboid / trap_low / infraspinatus / lateral border). AFE pods move off the rail to the spine bridge (target; fit not run). Spec: `specs/SPEC_flex_electrode_leads.md`. No CAD changed, no pipeline run, no renders, no STLs this session.

## Decisions (who decided)
- Forest (2026-10-23): rigid skeleton over the electrodes removed; electrodes stay put; flexible wires to plug-in chassis jacks; electrodes translation invariant. Consistent with the standing rule "nothing rigid presses on skin that moves with the scapula" (the rail was the one exception; it is now removed).
- Claude (2026-10-23, consequence of the above, pending Forest's veto): AFE pods relocated to the spine bridge (still zone); harness load reroute tested first on option A (paraspinal strip + clasp), option B (small still-strip pad) only if A fails.

## Open items (new, in priority order)
1. Harness top pad (key): rail removed; top pad was 14.4 vs 14.7 kPa (2.0 % margin). Run `harness_loads.py` and `body_loads.py` with rail rows off before any geometry cut. Decides option A or B (spec section 1).
2. Restore the workspace: this repository's `cad/` scripts; `pip install --break-system-packages cadquery trimesh manifold3d scikit-image rtree python-fcl scipy fast-simplification`; MakeHuman base mesh into `external/makehuman_base/base.obj` (README gives the symlink).
3. Cut the rail in `cad/scapular_frame.py` (remove the RAIL band, the lead slot, and the splice plate if it only served the rail's z split). Re-run the pods chain.
4. AFE pod spine-bridge fit (target clearances: skin at least 10, battery at least 10, lead at least 50 mm, arm sweep at least 3 mm). Likely conflicts with the main box (3.27 mm clearance to the spine bridge).
5. Flex-run model and jack plate (placeholder header; pinout to `SPEC_electronics.md`).
6. Collar-only contact budget (within 17 mm band): T14-3e re-measure.
7. Electronics items carried forward: rev f2 open items 0n (a)-(g) unchanged, except AFE lead service loop and entry glands (0n(c)) are replaced by the flex-run routing above.
8. GitHub push: done for the two new specs in this repository. The design-state file itself is not in this repository and remains in the project only.

## Superseded by this addendum
- `SPEC_rail_facing.md` section 2, 2.1, 3, 4 (rail parts); section 5 rail term; section 7 T14-3 applies to the flex run.
- `SPEC_harness.md` section 2 rail row; section 7b lead routing (rail slot); section 8 donning step 6 (lead through foam slit / rail slot), now "lead to jack".
- `DESIGN_STATE.md` section 11 item 0(a) (rail facing) closed by removal; section 11 item 0n(c) (AFE lead service loop) replaced.
