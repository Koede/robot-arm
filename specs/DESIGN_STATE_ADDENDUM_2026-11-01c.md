# DESIGN_STATE addendum, 2026-11-01c: SHOULDER rev 2 = J1 flat pancake / side-hung joint (IN DESIGN, not adopted)
Paste into DESIGN_STATE.md: registry (SHOULDER, WING, CONTROL), section 3 (joints), decision log, changelog, open items. Specs: SPEC_flat_shoulder_joint_study.md, SPEC_shoulder_mount_alternatives.md (mount study C1-C6, superseded by Forest's redirection), SPEC_j1r2_integration.md, SPEC_j1r2_kinematics.md, SPEC_j1r2_balancer.md.
## Request (Forest, 2026-11-01)
"rethink the shoulder joint itself, maybe we don't want it to be a big ball on my shoulder but instead a sleek integrated design that sits flatter to my back" -> chose J1 ("Pancake side hung maybe") -> "Assure the connections happen properly and integrate this into our full design" -> "Then do a full collision sweep to optimize local geometry" (overrides the 2026-11-01 "without a full collision sweep").
## Registry
- SHOULDER: rev b (ball + plate + flush armour) REMAINS the adopted design. **Rev 2 (J1) is IN DESIGN**, pad (y1, z1) = (+10, -30), depth 112.4 mm behind the back (181), printed fixed about 149 g. Not locked; waits on Forest's decision about the workspace trade below.
- WING: the 6061 chassis plate and tongue seat are retired in rev 2; the pad bolts through 4 standoffs to 4 new M4-insert bosses on the paraspinal rail top end (contract change).
- CONTROL: kinematics changes (S1' about +x, non-concurrent S3 offset 105.75 mm; S2 soft max 75 recommended). Firmware and tables NOT regenerated.
## Decisions
- **Forest:** drops the ball-styled / concurrent-at-P shoulder for this concept; picks the pancake side-hung layout.
- **Claude (pending Forest's veto):** no axis tilt (cost 3.5 pt coverage + 7.7 mm); pad at y +10, z -30 (y +30 gives passive-box coverage 0.83 for a 20 mm longer riser); one pad-anchored spring for both S1' and S2; obsolete yoke column anchor dropped.
## Sweep result (170,100 poses, soft range, first-contact method)
Reach-free coverage of the baseline arm directions 0.898 with software keep-out, 0.776 with passive stops alone (S1' -75..25, S2 -15..50). Shoulder-level first contacts: upper arm vs head 47.1 mm at S1' 35 / S2 40 / S3 90; arm vs skin at S1' -95 / S2 50; yoke vs upper arm at S2 -30 / S3 -70. Hand/wrist groups need tier tables (hand envelope conservative).
## Open items
Harness/body loads re-run (loads now enter the rail top end; top-pad margin was 2 %); balancer packaging (230 mm tube) and re-fit at the new pad; cable bend radius 9.1 < 12.6 mm; riser skin gap 17 mm < 20 mm; tier tables and firmware headers; flange patterns PLACEHOLDER; GitHub push (repo access denied).
## Changelog
- 2026-11-01c: mount study C1-C6, flat-joint study J1-J6, J1 rev 2 integrated, whole-arm sweep, pad optimisation.
