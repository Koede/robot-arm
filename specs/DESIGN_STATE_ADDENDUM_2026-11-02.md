# DESIGN_STATE addendum, 2026-11-02: J1 mount alternatives (SHOULDER / WING, concept)

Paste into DESIGN_STATE.md: section 3 (J1 rev 2 note), open items, changelog. Spec: `specs/SPEC_j1r2_mount_alternatives.md`.

## Request (Forest, 2026-11-02)
"Explore alternative designs for the shoulder mount." He chose: the mount of the J1 pancake shoulder; concepts + key numbers.

## Findings about J1 rev 2 as built (REF)
- **Mount stiffness:** the hand tip sags 18 mm at lateral reach (pad tilt 2.4 deg), and the rail top unloads 18.8 kPa.
- **Neck:** extension first contact at 27.5 deg (normal 59). The cause is the rail-top lap behind C7.
- **Clasp:** its 4 feet are orphaned. Their old seat is inside the S2 motor / stop-post sweep.
- **Wing clash:** the verified wing's bridge + tongue clash with the J1 riser. The J1 sweep used a proxy wing.
- **Balancer:** the spring still has no home. The radial tube rises beside the head, and the arm sweep reaches it.

## Concepts (all verified against the same harness model and the J1 stage)
| | Mount + balancer (g) | Hand-tip sag at reach | Status |
|---|---|---|---|
| MJ1 spine spar, balancer inside | 475 | 8.1 mm | Solves the balancer; 70 mm spar column |
| MJ2 shoulder-top truss | 486 | 14.8 mm | Stops your arm at 105–110 deg GH |
| MJ3 bilateral yoke | 498 | 5.0 mm | Keeps REF's neck problem |
| MJ4 carbon K-truss | 479 | 5.3 mm | Best stiffness per gram; no ROM contact |
| MJ5 quick-release dock | 544 | 5.0 mm | Donning in two steps |
| MJ6 monocoque | 461 | 3.9 mm | Fails the 3 mm stage gate (2.3); balancer undesigned |

Numbers, gates and caveats: spec section 4.

## Decisions
- **Claude, pending Forest's veto (concept-level, nothing adopted):**
  - wing trimmed to its rail strip;
  - clasp re-seat (F1 hub x -126, F4 -126, F5 -123, R1 dropped);
  - mounts land on the rail's lateral strip below z -95.
- **Recommendation for Forest:** MJ4 K-truss with MJ1's in-tube balancer (optionally MJ5's dock at the K node).

## Open items
- Whole-arm sweep and harness_loads with the chosen mount.
- Blade-edge shortfalls of 3–7 mm (concept-level).
- F5 clasp foot 12.7 mm off the skin.
- WING contract change.
- PLACEHOLDER parts (AK70 flange, dovetail / cam, spiral cassette, CF tube bonds); M4 insert pull-out still to verify.

## Changelog
- **2026-11-02 (J1 mount study):**
  - added `cad/j1r2/mj_frame.py`, `mj_concepts.py`, `mj_eval.py`, `mj_render.py`;
  - `clib.ROOT` reads `J1R2_ROOT`;
  - outputs in `stl/j1r2_mounts/` and `renders_j1r2/mounts/`.
