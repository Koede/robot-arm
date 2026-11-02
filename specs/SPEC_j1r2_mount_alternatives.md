# SPEC: J1 shoulder mount alternatives (SHOULDER / WING, concept study), 2026-11-02

**Request (Forest, 2026-11-02):** "Explore alternative designs for the shoulder mount." He chose: the mount of the J1 pancake / side-hung shoulder (J1 rev 2, IN DESIGN), concepts + key numbers, no full sweep.
Status: CONCEPT. The adopted baseline (rev b ball + plate + flush armour) and J1 rev 2 are untouched; nothing here is adopted.

- Code: `cad/j1r2/mj_frame.py` (frame model + harness), `mj_concepts.py` (7 concepts: geometry + members), `mj_eval.py` (all checks), `mj_render.py`.
- Results: `stl/j1r2_mounts/<ID>/` (STLs R + L, metrics.json), `stl/j1r2_mounts/summary.json`.
- Renders: `renders_j1r2/mounts/overview_mounts.png`, `<ID>_sheet.png`.
- Run: `J1R2_ROOT=<work root> python3 cad/j1r2/mj_eval.py` (~4 min, 2 cores), then `mj_render.py`. `clib.ROOT` now reads `J1R2_ROOT` (see docs/ENV_SETUP.md).

## 1. What the J1 rev 2 mount gets wrong (REF, found by this study)

1. **Too soft.** All arm load enters the top 30 mm of the real wing rail (38 x 7 mm PA-CF on 7.5 mm foam) through 4 standoffs.
   - Lateral reach (S1' -90, S2 0, 0.5 kg): the pad tilts 2.4 deg and the hand tip sags **18 mm**; 11 mm of that is the harness rolling on soft tissue.
   - The rail top **unloads by 18.8 kPa**: it lifts off the back unless the strap preload is higher than that.
2. **Neck.** The rail-top lap and its skirt sit behind C7. In the rigid-neck model, neck extension first touches at **27.5 deg** (normal 59).
3. **Clasp orphaned.** The clasp's 4 feet bolted to the retired chassis plate. Their old seat (x -88) lies inside the S2 motor sweep, and the S2 stop posts reach x -117 at 81..92 mm from the S1' axis.
4. **Wing clash.** The verified wing's bridge and tongue (over the blade) occupy the corridor the J1 riser uses (0.1 mm). J1 rev 2 swept against a proxy wing, so this was never seen.
5. **Balancer.** The spring tube (radial, 236 mm) still has no home: it rises to z +283 beside the head, and the arm sweep reaches it.

## 2. The corridor (governs every concept)

- **Over the blade:** below the pad the back bulges to x -117..-127, so the 20 mm blade rule needs x >= -97. That is exactly where the S2 motor sweep starts (clamp face x -96.8).
- **Usable paths:**
  - leave the pad medially (psi 165..205 deg, outside the S2 sweep);
  - cross the blade corner at about (x -101, y -98, z -96);
  - land on the rail's lateral strip (y -124..-100) below z -95.
- **Off-limits:** the rail's medial strip at z -241..-112 is taken by the spine-bridge ends, and anything medial of y -110 above z -95 restricts neck extension.
- **Common to all concepts:**
  - wing trimmed to its rail strip (bridge + tongue retired, as the plate is);
  - clasp re-seated: hub = F1 foot at x -126, F4 at -126, F5 at -123, all bars 6 mm PA-CF;
  - R1 foot dropped (lifted standoff node).

## 3. Concepts

- **REF:** J1 rev 2 as built. The clasp bracket to the riser is an ASSUMED fix, not in the J1 rev 2 CAD.
- **MJ1, spine spar with the balancer inside.**
  - A straight PA-CF box spar (40 x 44 x 2.4, z -105..-300) on 4 x 2 standoffs to the rail's lateral strip.
  - A 2-segment box knee (50 x 16 x 2.4) runs from the pad's medial rim through the blade corner.
  - The balancer spring runs at 4:1 (151 mm tube) inside the spar. A cord goes over a spar-head sheave and a pad sheave to the unchanged anchor A, so the arm torque balance is unchanged. The spring force doubles to about 437 N.
- **MJ2, shoulder-top truss:** REF plus two CF struts (12 x 1) from the pad's low rim to posts on the F1 / F4 top seats.
- **MJ3, bilateral yoke:** REF plus a CF tube (22 x 1.5) pad-to-pad, across T2 30 mm off the skin, via the blade-corner knees.
- **MJ4, carbon K-truss:** 4 pultruded CF tubes (14 x 1.5). Two run from the pad's medial rim to a node over the blade corner, then two from it to 3-bolt rail clamps at z -112 / -215. The riser is gone.
- **MJ5, quick-release dock:**
  - a knee as MJ1 ends in a 6061 dovetail with an over-centre cam (PLACEHOLDER part);
  - a 6061 receiver (22 x 105 x 3) sits on the rail's lateral strip;
  - the whole arm module, clasp brackets included, comes off in one piece.
- **MJ6, monocoque:**
  - one closed PA-CF shell (1.8 mm skins) is pad + riser + fairing;
  - its inner skin is 20 mm (3-D) over the blade and sits on the rail's lateral strip (6 x M4);
  - the balancer would be a flat spiral-spring + cam-drum cassette in the shell (NEW type, not designed).

## 4. Results (`summary.json`)

| | Mount + balancer (g) | Hand-tip sag park / reach (mm) [body-only] | Pad tilt at reach (deg) | Rail-top unload at reach (kPa) | Interface bolt, worst (N) | Stage clearance M1 / arm (mm) | Wearer ROM first contact | Skin rule shortfalls | Depth of fixed parts (mm) |
|---|---|---|---|---|---|---|---|---|---|
| **REF** J1 rev 2 as built: rail-top lap | 98 + 307 = 406 | 10.4 / 18.2 [7.5 / 10.9] | 2.38 | -18.8 | 16 | 4.2 / 14.0 | neck rotation R 90, neck extension 27.5 | fairing_lap: rail 2.8/5.0; clasp_bracket: blade 5.3/20.0 | 8 |
| **MJ1** Spine spar, balancer inside | 339 + 136 = 475 | 5.6 / 8.1 [0.7 / 1.1] | 0.69 | -4.4 | 160 | 4.2 / 14.0 | neck rotation R 87.5, neck extension 67.5 | spar: blade 16.9/20.0; knee: blade 14.2/20.0; clasp_bracket: blade 5.3/20.0 | 70 |
| **MJ2** Shoulder-top truss to F1 / F4 | 179 + 307 = 486 | 11.9 / 14.8 [8.0 / 10.5] | 1.94 | -16.2 | 22 | 4.2 / 9.0 | arm abduction 110, arm scaption 30 105, neck lat flex R 47.5, neck rotation R 90, neck extension 30 | fairing_lap: rail 2.7/5.0; clasp_bracket: blade 5.3/20.0 | 7 |
| **MJ3** Bilateral yoke across T2 | 190 + 307 = 498 | 4.1 / 5.0 [3.0 / 2.1] | 1.03 | -7.0 | 20 | 4.2 / 14.0 | neck rotation R 90, neck extension 30 | fairing_lap: rail 2.8/5.0; yoke_tube: blade 16.8/20.0; yoke_nodes: blade 16.2/20.0; clasp_bracket: blade 5.3/20.0 | 19 |
| **MJ4** Carbon K-truss | 172 + 307 = 479 | 2.4 / 5.3 [0.8 / 1.2] | 0.49 | -4.9 | 322 | 4.2 / 14.0 | none in range | tubes: blade 13.4/20.0; nodes: blade 16.6/20.0; clasp_bracket: blade 5.7/20.0 | 21 |
| **MJ5** Quick-release dock | 236 + 307 = 544 | 2.1 / 5.0 [0.8 / 1.4] | 0.57 | -5.7 | cam dovetail (contact) | 4.2 / 14.0 | neck extension 67.5 | riser_dock: blade 14.6/20.0; clasp_bracket: blade 5.3/20.0 | 31 |
| **MJ6** Monocoque: the fairing is the structure | 461 + 0 = 461 | 2.6 / 3.9 [0.9 / 1.4] | 0.44 | -6.0 | 77 | 2.3 / 10.2 | neck lat flex R 30, neck rotation R 47.5, neck extension 70 | monocoque: blade 18.7/20.0; clasp_bracket: blade 5.3/20.0 | 26 |

How to read the table:
- "Body-only" = mount members x1000 stiffer; what is left is the harness rolling on soft tissue.
- Rail unload = drop in contact pressure from the strap preload (negative = lifting).
- Stage clearance is checked over the J1 rev 2 recommended HARD ranges (S1' -95..35, S2 -30..80, S2 <= 50 at S1' <= -85).
- Wearer ROM uses the verify_armor model: rigid GH to 110 deg; head blended over the cervical span. That set also rotates upper-trapezius skin, so it is conservative for MJ6.
- Norms: lateral flexion 42 +- 8, rotation 71, extension 59 (Apti 2023).
- Bolt forces are static. With x2 dynamic, MJ4's 322 N clamp bolts give SF 1.55 against the 1 kN typical M4-insert pull-out (UNVERIFIED); every other concept is >= 3.

Balancer homes:
- **REF, MJ2–MJ5 (radial tube, unchanged):** rises to z +283 beside the head (93 mm at neutral); the arm sweep reaches it.
- **MJ1:** the spring is off the shoulder entirely. The fixed cord runs 15 mm from the S1' group and 38 mm from the arm sweep, but only 4 mm from the fairing, so it needs the channel that is modelled.
- **MJ6:** a cassette fits the shell, but no such balancer is designed yet.

## 5. Reading the results

- **Stiffness comes from where the load enters, not from adding struts.** Every concept that lands below z -95 on the rail's lateral strip (MJ1, MJ4, MJ5, MJ6) cuts the body-only sag from 11 to about 1 mm. Struts added on top of the rail-top lap (MJ2) barely help: 18 -> 15 mm.
- **MJ3's yoke** is the only cheap way to stiffen REF itself (18 -> 5 mm). It ties the two arms' roll moments across the spine, but it keeps REF's neck problem.
- **MJ4 is the best stiffness per gram:** 172 g, 5.3 mm at reach, no wearer-ROM contact in range. Its weak point is the clamp bolt load.
- **MJ6 is the stiffest and cleanest-looking**, but heavy (461 g). It is 2.3 mm from the S2 group at S1' -95 (fails the 3 mm gate), restricts neck lateral flexion / rotation in this model, and depends on an undesigned balancer.
- **MJ1 is the only concept that solves the balancer**, at the cost of a 70 mm spar column beside the spine. That stays well inside the 112 mm the J1 stage already projects.
- **MJ2 puts structure on the shoulder top.** It stops your arm at 105–110 deg GH and neck lateral flexion at 47.5.
- **MJ5 adds the quick-release** (donning: harness on first, then dock the 2 kg module) for +60 g over MJ4, and its stiffness is about MJ4's.

## 6. Recommendation (Claude, pending Forest's decision)

Build **MJ4 + MJ1's balancer**: a carbon K-truss whose rail-side leg is the balancer tube.
- The spring sits at 4:1 in a 41 mm liner tube along the rail's lateral strip.
- The cord runs over the K node to sheave A.
- Optionally make the K node the MJ5 dock, so the module comes off as one piece.

Expected (by superposition of MJ4 and MJ1, not yet modelled): about 300 g of mount + balancer housing, about 5 mm hand-tip sag at reach, no neck or arm contact, and the balancer off the shoulder.

## 7. Open items

1. **Not run:** whole-arm sweep (E / wrist / hand groups vs the new parts), harness_loads.py, and the real strap preload, which decides whether the 4–7 kPa rail unload lifts the rail.
2. **Blade-edge shortfalls (y -84..-100):** 3–7 mm short of 20 mm in MJ1 / MJ3 / MJ4 / MJ5, 1.3 mm in MJ6. Fix by local shaping.
3. **F5 clasp foot:** the clasp's own node sits 12.7 mm off the skin, so bars landing on it are short of 20 mm in every concept. Clasp-side fix: raise the F5 foot or drop it.
4. **WING contract change (all concepts):**
   - wing bridge + tongue retired;
   - new M4 bosses on the rail's lateral strip;
   - clasp feet re-seated (x -123 / -126, R1 dropped).
   Both sides must be re-verified on adoption.
5. **PLACEHOLDER parts:** AK70 flange, dovetail / cam (MJ5), spiral cassette (MJ6), CF tube end bonds (pull-out to test). M4 insert pull-out in PA-CF still needs bench verification.
6. **Model limits:** the frame model is a comparative estimate (foam E 0.15 MPa from scapular_frame.json, tissue lumped). Pressures are increments about the preload. The hand / forearm is rigid in the tip-sag figure.
