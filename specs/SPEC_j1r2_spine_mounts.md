# SPEC: J1 pancake mount, round 2: spine-led concepts, no balancer (2026-11-02)

Subsystem: SHOULDER / WING + HARNESS (concept study). Continues `SPEC_j1r2_mount_alternatives.md` (round 1, MJ1..MJ6). Owner decisions: Forest ("no balancer maybe"; "reinforce on the spine more or even extend the spine to cover more of the back, bridge from there"). Geometry choice is Forest's call: nothing here is locked.

Code: `cad/j1r2/mj_concepts2.py` (registers MJ7, MJ8, MJ10 into `mj_concepts.CONCEPTS`), evaluated by `mj_eval.py` (balancer kind `none` added), rendered by `mj_render.py`. Same frame solver, loads, gates and body model as round 1.

## Why the spine, and what it can and cannot buy (round 1 numbers)
- Foam / body is not the limit: body-only floor 0.7-1.4 mm hand-tip sag at reach; rail pressure <= 2.4 kPa.
- The structure and its bolts are: MJ4 has 322 N on a 3-bolt pair spaced 103 mm, CF tubes at 62 MPa, rail unload -4.9 kPa.
- So the spine is used to spread the load over a long stiff member and tie the left and right rails into one ladder. It is not used to add foam area.

## Concepts (right side, per-side mass; midline members are modelled once and split half per side)
| | what | mass/side | hand-tip sag park / reach | pad tilt (reach) | peak interface force | rail unload (reach) | depth behind back |
|---|---|---|---|---|---|---|---|
| MJ4 (round 1, no balancer) | K-truss to rail clamps z -112 / -215 | 172 g | 2.36 / 5.27 mm | 0.49 deg | 322 N bolt group (3 x M4) | -4.9 kPa | 21 mm |
| MJ7 | spine ladder: PA-CF box spar on the midline + 2 new rungs; both pads' 22 mm knees meet at the spar head | 226 g | 3.07 / 5.23 | 0.48 | posts <= 20 N | -1.9 kPa | 40 mm |
| MJ8 | MJ4 with the lower leg moved to rail z -320 (lever 103 -> 208 mm) | 181 g | 2.48 / 4.73 | 0.44 | 370 N bolt group (worse) | -4.4 kPa | 17 mm |
| MJ10 | K-truss whose two lower legs land on the spar (z -120, -180) | 196 g | 2.41 / 5.06 | 0.52 | posts <= 20 N | -1.8 kPa | 55 mm |

Findings
1. A longer lever does not help the bolts (MJ8): the lower leg is a tube, so most of the moment goes through the tube bending, not a couple across the bolt pair. Hand-tip sag improves 0.5 mm only.
2. A spine ladder removes the concentrated bolt group: the 322 N pair becomes 5 posts at <= 20 N, and rail bending stress drops from 5.3 to 0.6 MPa, the spine-bridge straps from ~97 N to ~18 N axial. The load is shared with the left rail through the rungs.
3. It does not make the shoulder stiffer: reach sag stays 4.7 to 5.2 mm because the pad-to-spine members (printed knee or 14 x 1.5 CF tubes) set it. Going below ~4 mm needs the MJ6 shell-class stiffness (461 g).
4. Cost: +20 to +55 g per side and 19 to 34 mm more depth behind the back than MJ4.
5. A straight pultruded CF mast (tried as MJ9, dropped): a straight tube cannot follow the lumbar curve and ends up 50 mm proud of the back.

## Gates (REF window as round 1; designed contact excluded)
- Skin: spar floats 26 mm over the spinous-process groove (no load on the spine or the nerve-sensitive midline); only the clasp bracket (shared by every concept, feet are designed contact) is under the 20 mm blade rule.
- Clearance to the real wing rail and spine bridge: >= 4.3 mm for all new parts except rungs / posts that bolt onto them (designed contact). The real spine-bridge ears reach x -96 / -101 (y -124..-140, z -112..-130): the knee crosses over them.
- Neck / head: MJ10 and MJ8 no first contact in the wearer ROM test; MJ7 neck rotation first contact at the end of range (90 deg), same as REF.
- Stage clearance: S1' group 4.2 mm (as REF), arm sweep 14 mm.
- No balancer: shoulder motors carry the full gravity torque (7.31 / 6.59 N.m vs rated 8.3, SF 1.14; gate needs >= 2). Needs Forest's decision (gate waiver, firmware torque cap, or a later cassette balancer). Not decided here.

## Standing-decision check
- "No big backplate, no hip belt": the spine ladder is an open member (36 mm spar + 40 mm rungs), floating, with foam contact only on the existing paraspinal rails. It is not a plate. A wider lattice reaching the flanks was not built: foam pressure is not the limit.
- Nothing rigid on skin over the moving blade: unchanged (knees pass 11 to 19 mm off the blade skin, as MJ1 / MJ4 do).

## PLACEHOLDER
- S1' AK70 flange pattern, pultruded CF tube sections and the M3 / M4 patterns of every new bought part.
- The frame model keeps the round 1 idealisation of the spine bridge as 60 x 5 straps; the real mesh is deeper (x -116..-96) and was used only in the clearance check.
