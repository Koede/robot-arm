#!/usr/bin/env python3
"""run_all.py - dependency-aware pipeline runner for the Third Shoulder (encodes DESIGN_STATE section 9, rev d).

    python3 cad/run_all.py --list                      stages, chain (default / manual:<group>), deps, est. minutes, fresh/stale status
    python3 cad/run_all.py --dry-run                   what WOULD run (stale default-chain stages + everything downstream) and the time it costs
    python3 cad/run_all.py                             run every stale default-chain stage in dependency order
    python3 cad/run_all.py --stage verify_claw         force verify_claw, then re-run whatever of its downstream is stale
    python3 cad/run_all.py --stage claw_hand --with-upstream    ... and any stale upstream of it first
    python3 cad/run_all.py --stage cowl --no-downstream         just that stage
    python3 cad/run_all.py --group balancer            force every stage of a manual group (balancer, freefall, sensitivity, harness_study, analysis, claw_search, arm_wiring)
    python3 cad/run_all.py --force                     everything in the default chain (manual stages still need --with-manual / --group)
    python3 cad/run_all.py --adopt [NAME..]            record the CURRENT files as up to date without running (use once after restoring the
                                                       project into a fresh container, or after a hand edit you know is harmless)
    python3 cad/run_all.py --forget NAME               drop a stage's record (next run treats it as never run)
    flags: --keep-going (continue with independent stages after a failure)  --with-manual (include every manual stage)
           --json (machine-readable --list/--dry-run)  --root DIR (repo root, default = parent of cad/)

CHAINS.  'default' stages are the section 9 regeneration chain and are scheduled whenever they are stale.  'manual' stages (slow searches, the S2
balancer I1b chain, free-fall mitigation studies, mu sensitivity runs, harness design-study drivers, the optional controls/electronics analyses)
are NEVER auto-scheduled: they run when named with --stage / --group, with --with-manual, or when a MANUAL upstream of theirs runs in the same
invocation (so --stage body_mitigations also re-renders body_mitigations.png).  A default stage that turns stale leaves its manual consumers
STALE in --list but does not run them.  Manual stages with optional=True whose script is not in cad/ yet (controls/electronics scripts that live
in other workspaces until promoted) are listed as 'absent' and are never planned.

STALENESS (per stage).  key = sha256( command incl. env, hash of each input script, hash of each declared data input, and the OUTPUT digest of every
upstream stage as recorded when this stage last ran ).  A stage is stale when it was never run, a required output is missing, a script or data input
changed, or an upstream stage's outputs changed (real run: compared byte-for-byte, so an upstream that regenerated identical files does NOT
trigger its downstream; dry-run: an upstream that will run counts as changed, so the dry-run is an upper bound).
'soft' inputs are files a stage reads but that are produced DOWNSTREAM of it (a read-back of the previous run's result, e.g. clasp_shoulder.py reads
emg_atlas.json for the AC-joint landmark); they are listed, never hashed, because hashing them would make the graph cyclic.  If a soft input
changes materially, re-run the reading stage by hand (the notes say which).
State lives in <root>/.pipeline_state.json; logs in <root>/logs/pipeline/<stage>.log.  Each stage runs with env TS_STAGE_KEY / TS_RUN_ID / TS_STAGE so
gates.py can tag every gate record with the stage and inputs it was computed from (gen_state_table.py then flags gates older than their inputs).

Output globs: a leading '?' marks an output that is expected but not required to exist (names I could not confirm from the sources).
est = rough minutes ('doc' = quoted in DESIGN_STATE section 9, 'guess' = my estimate, marked * in --list).
Rebased 2026-10-14 (f7) onto DESIGN_STATE rev d section 9; inputs/outputs of every new stage read from the live scripts.
"""
import argparse, glob, hashlib, json, os, subprocess, sys, time

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# ----------------------------------------------------------------------------------------------------------------------------------------------
# Shared script groups (every stage that imports these re-runs when they change)
# ----------------------------------------------------------------------------------------------------------------------------------------------
SWEEP = ["sweep_shoulder.py", "rom_limits.py"]
KIN = ["arm_kin.py", "claw_asm.py", "claw_synth.py"] + SWEEP                  # arm_kin.groups() builds the whole arm incl. the C3 claw
ARM_STAGES = ["shoulder_stage", "upper_arm_elbow", "distal_arm", "claw_asm", "distal_stops", "ball_stops"]
HARNESS_STAGES = ["wearer", "clasp", "chassis", "scapular_frame"]
GEOM = ARM_STAGES + HARNESS_STAGES + ["connector", "sensor_pods", "cowl", "pauldron"]
# body_loads.py is imported (module level reads the wearer mesh, clasp_sites, pad_foam.npz, scapular_frame, clasp_cradle, landmarks, emg_atlas
# landmarks, verify_distal.json, and harness_loads' masses) by body_mitigations, pad_compare, rail_eval, balancer_calc (lazy) and balancer_freefall.
BL_SCRIPTS = ["body_loads.py", "harness_loads.py", "biomimetic_shoulder.py", "facet.py", "rom_limits.py"]
BL_DEPS = ["harness_loads", "verify_distal", "keepout_map", "muscle_atlas", "clasp", "chassis", "scapular_frame", "sensor_pods", "wearer"]
SHD = "stl/shoulder/"
BAL = "stl/balancer/"

# ----------------------------------------------------------------------------------------------------------------------------------------------
# Stage table, in a valid topological order.  step = DESIGN_STATE section 9 item ('*' = placed differently from section 9, see note).
# keys: cmd (shell, run from ROOT) | env (VAR=value prefix, part of the hashed command) | scripts (in cad/, or path from ROOT if it contains '/')
#       data (globs from ROOT hashed as inputs) | soft (files read but produced downstream: listed, not hashed) | deps (stage names whose outputs
#       are inputs) | out (globs from ROOT) | est (min) | est_src | manual + group (not auto-run) | optional (script may be absent from cad/)
#       iso (python -I) | needs (external files that must exist) | step | note
# ----------------------------------------------------------------------------------------------------------------------------------------------
def S(name, step, cmd, scripts, deps=(), out=(), est=1.0, est_src="guess", data=(), manual=False, group=None, needs=(), note="", iso=False,
      env="", soft=(), optional=False):
    if not cmd.startswith(("g++", "cd ", "cp ", "sh ")):
        cmd = ("python3 -I " if iso else "python3 ") + cmd
    if env:
        cmd = env.strip() + " " + cmd
    return dict(name=name, step=step, cmd=cmd, scripts=list(scripts), deps=list(deps), out=list(out), est=est, est_src=est_src, data=list(data),
                manual=bool(manual), group=(group or ("manual" if manual else None)), needs=list(needs), note=note, soft=list(soft), optional=bool(optional))


STAGES = [
    # ---- 1 body
    S("wearer", "1", "cad/wearer_makehuman.py --back_to_p 108", ["wearer_makehuman.py"], iso=True, est=1,
      needs=["external/makehuman_base/base.obj"], data=["external/makehuman_base/base.obj"],
      out=[SHD + "proxies/wearer_makehuman.stl", SHD + "proxies/wearer_landmarks.json"]),
    # ---- 0 atlas rules (section 9 step 0: must precede the clasp; it only needs the body + landmarks, it exits before reading clasp_sites.json)
    S("atlas_rules", "0", "cad/muscle_atlas.py --rules", ["muscle_atlas.py"], deps=["wearer"], iso=True, est=0.5,
      out=[SHD + "emg_rules.json"], note="F2/F3 seat on these rule points in clasp_shoulder.py; USE_ATLAS=0 (env of the clasp stage) restores the old sites."),
    # ---- 2 mechanical chain
    S("shoulder_stage", "2", "cad/shoulder_stage.py", ["shoulder_stage.py"] + SWEEP[1:], est=1,
      out=[SHD + "s1_base_R.stl", SHD + "s1s2_yoke_R.stl", SHD + "s3_bracket_R.stl", SHD + "s3_lid_R.stl", SHD + "proxies/ak_s1_body.stl"]),
    S("upper_arm_elbow", "2", "cad/upper_arm_elbow.py", ["upper_arm_elbow.py"], deps=["shoulder_stage"], est=1,
      out=["stl/upperarm/ua_lat_R.stl", "stl/upperarm/fork_drive_R.stl", "stl/upperarm/proxies/xm_elbow.stl"]),
    S("distal_arm", "2", "cad/distal_arm.py", ["distal_arm.py"], deps=["upper_arm_elbow"], est=2,
      out=["stl/distal/distal.json", "stl/distal/forearm_dist_R.stl", "stl/distal/w2_yoke_R.stl", "stl/distal/proxies/sts_w3_R.stl"]),
    S("ball_stops", "2", "cad/ball_stops.py", ["ball_stops.py"], deps=["shoulder_stage"], est=1,
      out=[SHD + "stops/s1_lug_R.stl", SHD + "stops/s1_post_a_R.stl", SHD + "stops/tpu_s2_pos_R.stl", "?" + SHD + "stops/s2_pad_pos_R.stl", "?" + SHD + "stops/s2_pad_neg_R.stl"]),
    # ---- 2b claw (claw_synth is the 5-min DE search; its JSON is stored in the project and normally reused)
    S("claw_synth", "2b", "cad/claw_synth.py", ["claw_synth.py"], est=5, est_src="doc", manual=True, group="claw_search", out=["cad/claw_synth.json"],
      note="DE search; writes claw_synth.json. Manual: reuse the stored JSON unless the claw kinematics change."),
    S("claw_hand", "2b", "cad/claw_hand.py", ["claw_hand.py", "claw_synth.py"], deps=["distal_arm"], data=["cad/claw_synth.json"], est=2,
      out=["stl/distal/claw/claw.json", "stl/distal/claw/hand_tray_R.stl", "stl/distal/claw/claw_rotor_R.stl"]),
    S("claw_asm", "2b", "cad/claw_asm.py", ["claw_asm.py", "claw_synth.py"], deps=["claw_hand"], data=["cad/claw_synth.json"], est=1,
      out=["?stl/distal/claw/claw_asm.json"], note="library-style script; outputs not confirmed. Pose API used by arm_kin / verify_claw / sweeps."),
    S("distal_stops", "2*", "cad/distal_stops.py", ["distal_stops.py", "arm_kin.py"], deps=["distal_arm", "claw_asm", "ball_stops"], est=3,
      out=["stl/distal/stops_MW3/*.stl", "stl/distal/stops_MW2/*.stl", "?stl/distal/stops_M2/*.stl"],
      note="Section 9 lists this BEFORE the claw; item 0h shows stops_MW3 depends on the claw hard angles, so it runs after claw_asm. "
           "stops_M2 (S3 posts + TPU pads) is read by balancer.py postcut()."),
    # ---- 1 (late): keep-out map needs Z_TIP from distal.json
    S("keepout_map", "1*", "cad/keepout_map.py", ["keepout_map.py"] + SWEEP, deps=["wearer", "distal_arm"], est=4,
      out=[SHD + "keepout_map.npz"],
      note="Section 9 puts it in step 1 but it reads Z_TIP from stl/distal/distal.json (written in step 2), so it runs after distal_arm."),
    # ---- 3 harness.  rev d: clasp writes the contoured-facet foam map + skirts; connector reads only clasp outputs; chassis reads the atlas landmarks.
    S("clasp", "3", "cad/clasp_shoulder.py --facets_only", ["clasp_shoulder.py", "facet.py", "bodyprobe.py"], deps=["wearer", "atlas_rules"], est=3,
      soft=[SHD + "emg_atlas.json"],
      out=[SHD + "clasp_facets_R.stl", SHD + "clasp_sites.json", SHD + "clasp_feet.json", SHD + "clasp_paths.json", SHD + "clasp_cradle.json", SHD + "pad_foam.npz",
           "?" + SHD + "skirts/tpu_skirt_*", "?" + SHD + "clasp_pads.json"],
      note="do not pipe through head (SIGPIPE kills it before the JSONs are written). Soft input: emg_atlas.json landmarks (ACR_top) place the F4 AC-joint "
           "foam relief; the atlas needs clasp_sites.json, so re-run clasp by hand if the atlas LANDMARKS move (I7 refit)."),
    S("connector", "3", "cad/organic_connector.py", ["organic_connector.py", "bodyprobe.py"], deps=["clasp", "wearer"], est=2,
      out=[SHD + "clasp_frame_R.stl"],
      note="reads clasp_paths/sites/feet/cradle + clasp_facets_R.stl only (no chassis input), so SPEC_strap_facets' order clasp -> connector -> chassis "
           "and section 9's clasp -> chassis -> connector are both valid."),
    S("muscle_atlas", "4*", "cad/muscle_atlas.py", ["muscle_atlas.py"], deps=["wearer", "clasp"], est=2, out=[SHD + "emg_atlas.json"],
      note="Section 9 step 4, moved up: chassis_axilla.py (rev d) reads emg_atlas.json landmarks (NOTCH, ACR_top) for the strap clearances, and "
           "body_loads.py reads them too. Needs only the body, landmarks and clasp_sites.json."),
    S("chassis", "3", "cad/chassis_axilla.py", ["chassis_axilla.py", "bodyprobe.py"], deps=["wearer", "clasp", "shoulder_stage", "muscle_atlas"], est=1,
      out=[SHD + "chassis_plate_R.stl", SHD + "strap_axilla.json"], note="rev d: 32 mm strap slots, strap route clearances + on-skin line pressure (env STRAP_T, default 10 N)."),
    # ---- 4 scapular frame, pods, atlas renders
    S("scapular_frame", "4", "cad/scapular_frame.py", ["scapular_frame.py", "scapula_motion.py"], deps=["wearer", "chassis", "connector"], est=2,
      out=[SHD + "scapular/wing_R.stl", SHD + "scapular/spine_bridge.stl", SHD + "scapular/scapular_frame.json"],
      note="rev d: RAIL y 12-50, z -440, relief recess + soft strip (zones.RAIL.relief read by body_loads)."),
    S("electronics_fit", "4", "cad/electronics_fit.py", ["electronics_fit.py"], deps=[], est=0.5, out=["stl/electronics/electronics_fit.json", "stl/pods/pod_dims.json"],
      note="rev f2: component library + box layouts; WRITES pod_dims.json (inner box dims, E-stop / XT60 positions, contents masses) that sensor_pods.py reads. Run it BEFORE sensor_pods."),
    S("sensor_pods", "4", "cad/sensor_pods.py", ["sensor_pods.py"], deps=["scapular_frame", "clasp", "electronics_fit"], est=1, out=["stl/pods/pods.json"]),
    S("electronics_model", "4", "cad/electronics_model.py", ["electronics_model.py"], deps=["electronics_fit", "sensor_pods"], est=0.1, out=["stl/electronics/electronics_proxies_check.json", "renders/electronics_layout.png"],
      note="rev f2: proxy solids for every component, inside/overlap check, posed viewing STL, layout render."),
    S("pods_arm_sweep", "4", "cad/pods_arm_sweep.py", ["pods_arm_sweep.py"], deps=["sensor_pods", "arm_kin"] if False else ["sensor_pods"], est=8, out=["stl/pods/pods_arm_sweep.json"],
      note="rev f2: right arm over a soft-limit grid vs main box, battery box and both AFE pods (gate 3 mm)."),
    # ---- 2d in-arm wiring (rev f3, manual group "arm_wiring"; slow searches: arm_wire_search.py, arm_anchor_feas.py and arm_chain_patch.py are run by hand, see SPEC_arm_wiring.md)
    S("arm_hub", "2d", "cad/arm_hub.py", ["arm_hub.py", "arm_wire_spec.py"], deps=ARM_STAGES, est=0.5, manual=True, group="arm_wiring",
      out=["stl/electronics/arm_hub_R.stl", "stl/electronics/arm_hub.json"], note="hub board H1 position/clearance; upper_arm_elbow.py reads arm_hub.json for the hub posts (re-run upper_arm_elbow after a hub change)."),
    S("arm_chain", "2d", "cad/arm_chain.py", ["arm_chain.py", "arm_wiring_lib.py", "arm_route.py", "arm_keepout.py", "arm_wire_spec.py"], deps=["arm_hub"], est=25, manual=True, group="arm_wiring",
      out=["stl/electronics/arm_chain.json"], note="needs wire_search_<J>*_K.json and anchor_feas.json (hand-run searches); per-link repairs: arm_chain_patch.py <link>."),
    S("arm_wiring_verify", "2d", "cad/arm_wiring_verify.py", ["arm_wiring_verify.py"], deps=["arm_chain"], est=15, manual=True, group="arm_wiring",
      out=["stl/electronics/arm_wiring_verify.json", "stl/electronics/arm_links.json"]),
    S("arm_clips", "2d", "cad/arm_clips.py", ["arm_clips.py"], deps=["arm_wiring_verify"], est=10, manual=True, group="arm_wiring",
      out=["stl/electronics/clips.json", "stl/electronics/clips/clip_S1_A_base.stl"]),
    S("arm_clips_check", "2d", "cad/arm_clips_check.py", ["arm_clips_check.py"], deps=["arm_clips"], est=5, manual=True, group="arm_wiring", out=["stl/electronics/clips_check.json"]),
    S("arm_cable_pose", "2d", "cad/arm_cable_pose.py", ["arm_cable_pose.py"], deps=["arm_wiring_verify"], est=5, manual=True, group="arm_wiring",
      out=["stl/electronics/cable_pose_park.json", "stl/electronics/cable_pose_reach.json", "stl/electronics/arm_cables_park.stl"]),
    S("arm_electrical", "2d", "cad/arm_electrical.py", ["arm_electrical.py", "arm_wire_spec.py"], deps=["arm_chain"], est=0.1, manual=True, group="arm_wiring", out=["stl/electronics/arm_electrical.json"]),
    S("render_sections", "2d", "cad/render_sections.py", ["render_sections.py"], deps=["arm_clips", "arm_cable_pose", "arm_hub"] + ARM_STAGES, est=2, manual=True, group="arm_wiring",
      out=["renders/sec_long.png", "renders/sec_cross.png"]),
    S("render_arm_wiring", "2d", "cad/render_arm_wiring.py", ["render_arm_wiring.py"], deps=["arm_cable_pose", "arm_clips"], est=1, manual=True, group="arm_wiring",
      out=["renders/arm_wiring_park.png", "renders/arm_wiring_reach.png"]),
    S("render_atlas", "4", "cad/render_atlas.py", ["render_atlas.py"], deps=["muscle_atlas"], est=1, out=["renders/emg_atlas.png", "renders/emg_atlas_back.png"]),
    S("atlas_spec", "4", "cad/atlas_spec.py", ["atlas_spec.py"], deps=["muscle_atlas"], est=0.2, out=["specs/SPEC_emg_atlas.md"]),
    S("render_harness", "4", "cad/render_harness.py", ["render_harness.py"], deps=["wearer", "clasp", "connector", "chassis", "scapular_frame", "muscle_atlas"], est=1,
      out=["renders/harness_atlas.png"], note="rev c addition, not listed in section 9."),
    # ---- 5 faceted shoulder
    S("cowl", "5", "cad/shoulder_cowl.py", ["shoulder_cowl.py", "facet.py"], deps=["shoulder_stage", "ball_stops", "chassis"], est=3, est_src="doc",
      out=[SHD + "cowl/cowl_R.stl", SHD + "cowl/head_dome_R.stl", SHD + "cowl/cowl_R_a_posX.stl"]),
    S("pauldron", "5", "cad/pauldron.py", ["pauldron.py", "facet.py"], deps=["cowl", "clasp", "chassis", "scapular_frame"], est=2, est_src="doc",
      out=[SHD + "pauldron/pauldron_R.stl", SHD + "pauldron/pauldron.json", SHD + "pauldron/pauldron_core_R.stl"], note="needs: pip install shapely"),
    # ---- 6 verification
    S("verify_stops", "6", "cad/verify_stops.py", ["verify_stops.py"] + SWEEP, deps=["shoulder_stage", "ball_stops"], est=2,
      out=[SHD + "stops/stops_verify.json"]),
    S("verify_shoulder", "6", "cad/verify_shoulder.py", ["verify_shoulder.py"] + SWEEP, deps=["shoulder_stage", "ball_stops", "cowl", "pauldron", "chassis", "clasp", "connector", "scapular_frame", "wearer"],
      est=3, est_src="doc", out=[SHD + "verify_shoulder.json"]),
    S("verify_pauldron", "6", "cad/verify_pauldron.py", ["verify_pauldron.py"] + KIN, deps=["pauldron", "clasp", "connector", "scapular_frame", "chassis", "wearer"] + ARM_STAGES + ["cowl"],
      est=3, out=[SHD + "pauldron/verify_pauldron.json"]),
    S("verify_claw", "6", "cad/verify_claw.py", ["verify_claw.py", "claw_asm.py", "claw_synth.py"], deps=["claw_asm"], est=1, est_src="doc", iso=True,
      out=["stl/distal/claw/verify_claw.json"], note="Section 9 lists verify_distal first, but verify_distal reads verify_claw.json, so verify_claw runs first."),
    S("verify_distal", "6", "cad/verify_distal.py", ["verify_distal.py"] + KIN, deps=ARM_STAGES + ["cowl", "verify_claw"], est=4,
      out=["stl/distal/verify_distal.json"], note="writes forearm_hand_mass_g; harness_loads / body_loads / safety_limits / power_budget read it"),
    S("sweep_claw_wrist", "6", "cad/sweep_claw_wrist.py", ["sweep_claw_wrist.py", "claw_asm.py"], deps=["claw_asm", "distal_arm"], est=2, est_src="doc",
      out=["stl/distal/claw/sweep_claw_wrist.json"]),
    S("verify_assembly", "6", "cad/verify_assembly.py", ["verify_assembly.py"] + KIN, deps=ARM_STAGES + ["cowl", "pauldron", "chassis"], est=4,
      out=[SHD + "verify_assembly.json"]),
    S("harness_loads", "6", "cad/harness_loads.py", ["harness_loads.py", "biomimetic_shoulder.py"],
      deps=["verify_distal", "sensor_pods", "clasp", "connector", "chassis", "scapular_frame", "pauldron", "upper_arm_elbow", "cowl", "shoulder_stage"],
      est=0.2, out=[SHD + "harness_loads.json"],
      note="2-D sagittal gate (14.7 kPa). Reads clasp_sites (contoured area_mm2), clasp_pads.json if present, scapular_frame RAIL zone, pods.json, verify_distal.json."),
    S("verify_harness", "6", "cad/verify_harness.py", ["verify_harness.py", "bodyprobe.py", "sweep_shoulder.py"], deps=["harness_loads", "clasp", "connector", "scapular_frame", "chassis", "wearer"],
      est=3, out=[SHD + "verify_harness.json"], note="rev d: also reads pad_foam.npz (contoured foam gaps, plate print slopes, AC relief) and strap_axilla.json (strap clearances)."),
    # 6 (rev d): 3-D skin-load model.  Design case MU 0.4, E_FOAM_KPA 200, NRAND 600 (defaults).  Output name: body_loads_mu<MU>[_E<E>][<TAG>].json
    S("body_loads", "6", "cad/body_loads.py", BL_SCRIPTS, deps=BL_DEPS, est=10,
      out=[SHD + "body_loads_mu0.4.json"],
      note="design case (MU 0.4, E 200 kPa). Env: MU, E_FOAM_KPA, NRAND, PRELOAD_MM, PAD_MODEL (auto|legacy|raster), PAD_CELL, SKIRT_ETA, STRAP_T, RAIL_CFG, "
           "FOAM_GRADE, TAG. Docstring says it writes body_loads.json + renders/body_loads.png: it writes only the JSON (render_body_loads.py draws the PNG). "
           "Reads keepout_map.npz (random poses, head sweep), strap_axilla.json, pad_foam.npz, scapular_frame.json, emg_atlas.json landmarks, verify_distal.json."),
    S("body_loads_mu0.2", "6", "cad/body_loads.py", BL_SCRIPTS, deps=BL_DEPS, env="MU=0.2", est=10, manual=True, group="sensitivity",
      out=[SHD + "body_loads_mu0.2.json"], note="friction sensitivity (rev d re-ran 0.2 / 0.4 / 0.6; stall limits swing ~2x with mu)."),
    S("body_loads_mu0.6", "6", "cad/body_loads.py", BL_SCRIPTS, deps=BL_DEPS, env="MU=0.6", est=10, manual=True, group="sensitivity",
      out=[SHD + "body_loads_mu0.6.json"]),
    S("render_body_loads", "6", "cad/render_body_loads.py", ["render_body_loads.py"], deps=["body_loads", "chassis", "clasp", "wearer"], est=1,
      out=["renders/body_loads.png"], note="argv[1] = result JSON (default body_loads_mu0.4.json); also reads strap_axilla.json, clasp_cradle.json."),
    S("body_mitigations", "6", "cad/body_mitigations.py", ["body_mitigations.py"] + BL_SCRIPTS, deps=["body_loads"] + BL_DEPS, est=45, manual=True, group="freefall",
      out=[SHD + "body_mitigations.json"],
      note="'long' (section 9). Imports body_loads 4x (E 200/100/400 + study 2), bisection per drop pose: runtime is a guess. Does not read body_loads' JSON; "
           "the dep only orders it after the design run. Its limiting-pose M_allow (22.6 N m) is the free-fall gate limit."),
    S("render_body_mitigations", "6", "cad/render_body_mitigations.py", ["render_body_mitigations.py"], deps=["body_mitigations"], est=1, manual=True, group="freefall",
      out=["renders/body_mitigations.png"]),
    # rev d harness design-study drivers (manual): pad_compare is where the F2 15.7 kPa single-cell and F4 'AC 8 mm' numbers come from.
    S("pad_compare", "6", "cad/pad_compare.py", ["pad_compare.py"] + BL_SCRIPTS, deps=["body_loads"] + BL_DEPS, env="TAG=design", iso=True, est=5, manual=True, group="harness_study",
      out=[SHD + "pad_compare_design.json"], note="env TAG (output suffix), NRAND (300), PAD_MODEL, SITES_JSON, PAD_ROLL, SKIRT_ETA, AC_FILL, MU. TAG=design is the gate run."),
    S("rail_eval", "6", "cad/rail_eval.py", ["rail_eval.py"] + BL_SCRIPTS, deps=["body_loads"] + BL_DEPS, est=3, manual=True, group="harness_study", out=[],
      note="candidate evaluator: prints one 'RES {json}' line, writes no file (freshness = input hash; result only in logs/pipeline/rail_eval.log). Env RAIL_CFG, NRAND (100)."),
    S("sweep_clasp_diff", "6", "cad/sweep_clasp_diff.py", ["sweep_clasp_diff.py"] + KIN, deps=ARM_STAGES + ["cowl", "connector"], iso=True, est=5, manual=True, group="harness_study",
      needs=[SHD + "bl_base/clasp_frame_R.stl"], out=["stl/sweep_clasp_diff.json"],
      note="clasp-only differential vs a SAVED baseline clasp (copy stl/shoulder/clasp_frame_R.stl to stl/shoulder/bl_base/ before changing the clasp). "
           "0 flips => sweep_full / --restricted can be skipped."),
    S("sweep_clasp_diff_restricted", "6", "cad/sweep_clasp_diff.py --restricted", ["sweep_clasp_diff.py"] + KIN, deps=ARM_STAGES + ["cowl", "connector"], iso=True, est=5,
      manual=True, group="harness_study", needs=[SHD + "bl_base/clasp_frame_R.stl"], out=["stl/sweep_clasp_diff_restricted.json"]),
    S("sweep_full", "6", "cad/sweep_full.py", ["sweep_full.py"] + KIN, deps=ARM_STAGES + ["cowl", "pauldron", "wearer", "scapular_frame", "chassis", "clasp", "connector"], est=8, est_src="doc",
      out=["stl/sweep_full.json", "stl/elbow_limits_15mm.json"]),
    S("sweep_full_restricted", "6", "cad/sweep_full.py --restricted", ["sweep_full.py"] + KIN, deps=ARM_STAGES + ["cowl", "pauldron", "wearer", "scapular_frame", "chassis", "clasp", "connector"],
      est=8, est_src="doc", out=["stl/sweep_full_restricted.json", "stl/elbow_limits_15mm_restricted.json"], note="same script as sweep_full, restricted wrist tier"),
    # ---- 6b S2 gravity balancer I1b (in design; NOT in the default chain).  Outputs stl/balancer/.
    S("balancer_calc", "6b", "cad/balancer_calc.py", ["balancer_calc.py"] + BL_SCRIPTS, deps=["body_loads"] + BL_DEPS, iso=True, est=2, manual=True, group="balancer",
      soft=[BAL + "verify_balancer.json"], out=[BAL + "balancer_calc.json"],
      note="report mode (argv 'opt' = spring search, not staged). Tg() imports body_loads. Soft input: verify_balancer.json arm-side mass (EXTRA_ARM); after "
           "verify_balancer changes the mass, run --stage balancer_calc once more (two-pass; the graph cannot express the loop)."),
    S("balancer", "6b", "cad/balancer.py", ["balancer.py", "balancer_calc.py"], deps=["balancer_calc", "shoulder_stage", "ball_stops", "distal_stops"], iso=True, est=3,
      manual=True, group="balancer",
      out=[BAL + "balancer_parts.json", BAL + "bal_anchor_R.stl", BAL + "bal_collar_R.stl", BAL + "bal_cap_R.stl", BAL + "s1s2_yoke_bal_R.stl",
           BAL + "tpu_s2_pos_bump_R.stl", BAL + "tpu_s2_neg_bump_R.stl"],
      note="reads s1s2_yoke_R.stl, s3_lid_R.stl, stops/s2_pad_{pos,neg}_R.stl, stl/distal/stops_M2/*. Writes *_L.stl mirrors too."),
    S("plate_pocket", "6b", "cad/plate_pocket.py", ["plate_pocket.py"], deps=["clasp", "chassis"], iso=True, est=5, manual=True, group="balancer",
      out=[BAL + "plate_pocket.json", BAL + "chassis_plate_pocketed_R.stl", "?plate_pocket_stress.png"],
      note="needs: pip install scikit-fem. FE of baseline vs pocketed 6061 plate; the stress PNG lands at the repo root."),
    S("verify_balancer", "6b", "cad/verify_balancer.py", ["verify_balancer.py", "balancer_calc.py"] + KIN,
      deps=["balancer"] + ARM_STAGES + ["cowl", "pauldron", "wearer", "scapular_frame", "chassis", "connector"], iso=True, est=10, manual=True, group="balancer",
      out=[BAL + "verify_balancer.json"], note="argv 'quick' = 6 x 8 grid instead of 14 x 24."),
    S("balancer_freefall", "6b", "cad/balancer_freefall.py", ["balancer_freefall.py", "balancer_calc.py", "body_mitigations.py"] + BL_SCRIPTS,
      deps=["balancer_calc", "verify_balancer", "body_loads"] + BL_DEPS, iso=True, est=30, manual=True, group="balancer",
      out=[BAL + "balancer_freefall.json"],
      note="argv E_b M_plateau [E+,E-,M+,M-] (defaults 2.0 J / 20 N m; as-built 0.79,1.21,16.6,17.0). BAL_MASS=1 -> balancer_freefall_massed.json "
           "(also needs plate_pocket.json; not staged). verify_balancer dep = arm-side mass via balancer_calc.EXTRA_ARM. Runtime is a guess."),
    S("balancer_loads", "6b", "cad/balancer_loads.py", ["balancer_loads.py", "harness_loads.py", "biomimetic_shoulder.py"],
      deps=["verify_balancer", "plate_pocket", "harness_loads", "clasp", "scapular_frame"], iso=True, est=0.5, manual=True, group="balancer",
      out=[BAL + "balancer_loads.json"]),
    S("balancer_struct", "6b", "cad/balancer_struct.py", ["balancer_struct.py", "balancer.py", "balancer_calc.py"], deps=["balancer_calc"], est=0.2, manual=True, group="balancer",
      out=[BAL + "balancer_struct.json"], note="pure hand calc on the balancer_calc / balancer constants (imports balancer.py, reads no file)."),
    S("render_balancer", "6b", "cad/render_balancer.py", ["render_balancer.py", "balancer_calc.py"] + KIN,
      deps=["balancer", "plate_pocket"] + ARM_STAGES + ["cowl", "chassis", "wearer"], est=3, manual=True, group="balancer",
      out=["renders/balancer_poses.png", "renders/balancer_closeup.png"]),
    # ---- 7 analysis
    S("biomimetic_shoulder", "7", "cad/biomimetic_shoulder.py", ["biomimetic_shoulder.py"], est=0.5, out=["?stl/biomimetic_shoulder.json"],
      note="also imported as a library by harness_loads.py, body_loads.py, balancer_loads.py and safety_limits.py"),
    S("safety_limits", "7", "cad/safety_limits.py", ["safety_limits.py", "biomimetic_shoulder.py"], deps=["verify_distal", "verify_claw", "clasp"], est=0.5,
      out=["stl/control/safety_limits.json"]),
    S("control_map", "7", "cad/control_map.py", ["control_map.py", "biomimetic_shoulder.py"], deps=["keepout_map", "safety_limits", "muscle_atlas"], est=2,
      out=["?stl/control/control_map.json"], note="needs keepout_map.npz; re-run after any HOMOLOGY / CHANNELS change"),
    S("power_budget", "7", "cad/power_budget.py", ["power_budget.py"], deps=["verify_distal"], est=0.1, out=["stl/power_budget.json"]),
    S("print_table", "7", "cad/print_table.py", ["print_table.py"], deps=GEOM, est=1, out=["stl/print_table.json", "specs/PRINT_TABLE.md"],
      note="globs the STL tree: depends on every geometry stage. Section 10 says the claw rows of PRINT_TABLE.md are stale since 14h; balancer parts not listed."),
    # ---- 7b optional controls / electronics analyses (manual). Scripts live in other workspaces until promoted to cad/: /home/claude/tc
    #      (thermal_gate, ads_afe_calc, afe_overlap: stdout only, hard-coded inputs) and /home/claude/td/cad (emg_calib, gravity_fit_sim,
    #      cond_exact_vs_triangle: write <root>/results/*.json).  Absent scripts are listed as 'absent' and never planned.
    S("thermal_gate", "7b", "cad/thermal_gate.py", ["thermal_gate.py"], est=0.1, manual=True, group="analysis", optional=True, out=[],
      note="lumped thermal model of AK70 / XM540 / STS3215 (vendor + ASSUMED Rc, C, Rw, 35 C ambient); stdout only."),
    S("ads_afe_calc", "7b", "cad/ads_afe_calc.py", ["ads_afe_calc.py"], est=0.1, manual=True, group="analysis", optional=True, out=[],
      note="ADS1299 gain 4 vs 8 trade (CHnSET item 0g); stdout only."),
    S("afe_overlap", "7b", "cad/afe_overlap.py", ["afe_overlap.py"], deps=["sensor_pods"], est=0.1, manual=True, group="analysis", optional=True, out=[],
      note="AFE pod vs battery box overlap (item 0h); hard-coded box sizes from sensor_pods.py, stdout only; the dep only flags it after a pod change."),
    S("emg_calib", "7b", "cad/emg_calib.py", ["emg_calib.py", "control_map.py"], deps=["muscle_atlas", "control_map"], est=3, manual=True, group="analysis", optional=True,
      out=["?results/calib_sim_nrep1.json"], note="env NMC (60), NREP (1) -> results/calib_sim_nrep<NREP>.json. Pair centres are hard-coded from emg_atlas rev 14b."),
    S("gravity_fit_sim", "7b", "cad/gravity_fit_sim.py", ["gravity_fit_sim.py", "emg_calib.py"], deps=["emg_calib"], est=0.5, manual=True, group="analysis", optional=True,
      out=["?results/gravity_fit_sim.json"]),
    S("cond_exact_vs_triangle", "7b", "cad/cond_exact_vs_triangle.py", ["cond_exact_vs_triangle.py", "control_map.py", "biomimetic_shoulder.py"], deps=["control_map"], est=0.2,
      manual=True, group="analysis", optional=True, out=["?results/cond_exact_vs_triangle.json"],
      note="conservatism of the triangle-inequality tip limiter. Inserts work/stub/cad on sys.path; in cad/ it falls back to the real control_map."),
    # ---- 8 firmware
    S("gen_tables", "8", "firmware/gen_tables.py", ["firmware/gen_tables.py"], deps=["keepout_map", "sweep_full", "sweep_full_restricted"], est=0.2,
      out=["firmware/third_shoulder/keepout_table.h", "firmware/third_shoulder/elbow_limits.h"]),
    S("host_test", "8", "cd firmware && g++ -std=c++17 -O1 -I. -Ihost_test/shim host_test/host_test.cpp -o ../logs/ht && ../logs/ht",
      ["firmware/host_test/host_test.cpp", "firmware/third_shoulder/control.h", "firmware/third_shoulder/protocols.h", "firmware/third_shoulder/third_shoulder.ino",
       "firmware/host_test/shim/Arduino.h"], deps=["gen_tables"], est=1, out=[],
      note="build + run (env NRUNS, DBG). rev d added torque caps, stall detector, category-1 E-stop + hard cut (45 safety checks). No file outputs: "
           "freshness is the input hash only; record its counts with 'python3 cad/gates.py record ...' (GATES_PATCHES.md section 15)."),
    # ---- 9 renders
    S("render_full", "9", "cad/render_full.py", ["render_full.py"] + KIN, deps=ARM_STAGES + ["cowl", "pauldron", "clasp", "connector", "chassis", "scapular_frame", "wearer"], est=3,
      out=["renders/full_poses.png", "?renders/full_park.png"]),
    S("render_shoulder_closeup", "9", "cad/render_shoulder_closeup.py", ["render_shoulder_closeup.py"], deps=["cowl", "pauldron", "wearer"], est=1, out=["?renders/shoulder_closeup.png"]),
    S("render_elbow", "9", "cad/render_elbow.py", ["render_elbow.py"], deps=["upper_arm_elbow", "distal_arm"], est=1, out=["?renders/elbow.png"]),
    S("claw_render", "9", "cad/claw_render.py", ["claw_render.py", "claw_asm.py"], deps=["claw_asm"], est=1, out=["?renders/claw.png"]),
    S("render_claw_c3", "9", "cad/render_claw_c3.py", ["render_claw_c3.py", "claw_asm.py"], deps=["claw_asm"], est=1,
      out=["renders/claw_c3_G100.png", "renders/claw_c3_G50.png", "renders/claw_c3_G0.png"]),
]
BYNAME = {s["name"]: s for s in STAGES}
GROUPS = sorted({s["group"] for s in STAGES if s["manual"]})


def _validate():
    seen = set()
    for s in STAGES:
        assert s["name"] not in seen, "duplicate stage " + s["name"]
        for d in s["deps"]:
            assert d in seen, "%s depends on %s which is not defined earlier (not a topological order)" % (s["name"], d)
        if not s["manual"]:
            bad = [d for d in s["deps"] if BYNAME_[d]["manual"] and d != "claw_synth"]
            assert not bad, "default-chain stage %s depends on manual stage(s) %s" % (s["name"], bad)
        seen.add(s["name"])
        BYNAME_[s["name"]] = s


BYNAME_ = {}
_validate()


def chain_of(s):
    return "manual:" + s["group"] if s["manual"] else "default"


def script_path(root, name):
    return os.path.join(root, name) if "/" in name else os.path.join(root, "cad", name)


def is_absent(root, s):
    """optional stage whose (first) script is not in this tree"""
    return s["optional"] and not os.path.exists(script_path(root, s["scripts"][0]))


def downstream(names):
    """names plus every stage that (transitively) depends on them."""
    out = set(names)
    for s in STAGES:
        if any(d in out for d in s["deps"]):
            out.add(s["name"])
    return out


def upstream(names):
    out = set(names)
    for s in reversed(STAGES):
        if s["name"] in out:
            out.update(s["deps"])
    return out


# ----------------------------------------------------------------------------------------------------------------------------------------------
# hashing
# ----------------------------------------------------------------------------------------------------------------------------------------------
_fh = {}


def fhash(p):
    st = os.stat(p)
    k = (p, st.st_mtime_ns, st.st_size)
    if k not in _fh:
        h = hashlib.sha256()
        with open(p, "rb") as f:
            for blk in iter(lambda: f.read(1 << 20), b""):
                h.update(blk)
        _fh[k] = h.hexdigest()
    return _fh[k]


def inputs_hash(root, s):
    h = hashlib.sha256(s["cmd"].encode())
    for sc in s["scripts"]:
        p = script_path(root, sc)
        h.update(b"S:" + sc.encode() + (fhash(p).encode() if os.path.exists(p) else b"MISSING"))
    for pat in s["data"]:
        for p in sorted(glob.glob(os.path.join(root, pat))):
            h.update(b"D:" + os.path.relpath(p, root).encode() + fhash(p).encode())
    return h.hexdigest()


def out_files(root, s):
    """(existing files, missing required patterns)"""
    files, missing = [], []
    for pat in s["out"]:
        opt = pat.startswith("?"); pat = pat.lstrip("?")
        hits = [p for p in sorted(glob.glob(os.path.join(root, pat))) if os.path.isfile(p)]
        files += hits
        if not hits and not opt:
            missing.append(pat)
    return files, missing


def out_digest(root, s):
    files, _ = out_files(root, s)
    h = hashlib.sha256()
    for p in files:
        h.update(os.path.relpath(p, root).encode() + fhash(p).encode())
    return h.hexdigest() if files else "none"


# ----------------------------------------------------------------------------------------------------------------------------------------------
# state + evaluation
# ----------------------------------------------------------------------------------------------------------------------------------------------
def state_path(root):
    return os.path.join(root, ".pipeline_state.json")


def load_state(root):
    try:
        with open(state_path(root)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"schema": 1, "stages": {}}


def save_state(root, st):
    tmp = state_path(root) + ".tmp"
    with open(tmp, "w") as f:
        json.dump(st, f, indent=1, sort_keys=True)
    os.replace(tmp, state_path(root))


def evaluate(root, s, st, pending, forced, dry):
    """Return (stale: bool, reasons: [str]).  pending = names that will run/ran in this invocation (dry: assumed to change their outputs)."""
    rec = st["stages"].get(s["name"])
    why = []
    if s["name"] in forced:
        why.append("forced")
    if not rec:
        why.append("never run (no record)")
    else:
        if rec.get("inputs") != inputs_hash(root, s):
            why.append("scripts/data changed")
        for d in s["deps"]:
            if d in pending and dry:
                why.append("upstream %s will re-run" % d)
            elif d in pending:      # real run: compare actual output digest with what we saw last time
                if rec.get("upstream", {}).get(d) != out_digest(root, BYNAME[d]):
                    why.append("upstream %s outputs changed" % d)
            else:
                if rec.get("upstream", {}).get(d) != out_digest(root, BYNAME[d]):
                    why.append("upstream %s outputs differ from last run" % d)
        if rec.get("returncode", 0) != 0:
            why.append("last run failed")
    _, missing = out_files(root, s)
    if missing:
        why.append("missing outputs: " + ", ".join(missing[:3]) + (" ..." if len(missing) > 3 else ""))
    return bool(why), why


def schedulable(s, forced, with_manual, pending):
    """default-chain stages always; manual ones only when asked for, or pulled by a MANUAL upstream that runs in this invocation"""
    if not s["manual"]:
        return True
    return with_manual or s["name"] in forced or any(d in pending and BYNAME[d]["manual"] for d in s["deps"])


def plan(root, st, targets=None, forced=(), with_manual=False, dry=True):
    """Ordered list of (stage, reasons) that would run.  targets: restrict to these names (already closed under whatever the caller wants)."""
    pending, out = set(), []
    forced = set(forced)
    for s in STAGES:
        if targets is not None and s["name"] not in targets:
            continue
        if is_absent(root, s) or not schedulable(s, forced, with_manual, pending):
            continue                        # manual stage nobody asked for: consumers just see its current outputs
        stale, why = evaluate(root, s, st, pending, forced, dry)
        if stale:
            pending.add(s["name"]); out.append((s, why))
    return out


def status_of(root, s, st):
    if is_absent(root, s):
        return "absent", ["script %s not in cad/ (optional stage)" % s["scripts"][0]]
    stale, why = evaluate(root, s, st, set(), set(), True)
    rec = st["stages"].get(s["name"])
    if not rec:
        return "never-run", why
    return ("STALE" if stale else "fresh"), why


# ----------------------------------------------------------------------------------------------------------------------------------------------
# commands
# ----------------------------------------------------------------------------------------------------------------------------------------------
def fmt_min(m):
    return "%.0f s" % (m * 60) if m < 1 else "%.0f min" % m if m == int(m) else "%.1f min" % m


def cmd_list(root, st, as_json):
    rows = []
    for s in STAGES:
        stt, why = status_of(root, s, st)
        rows.append(dict(name=s["name"], step=s["step"], chain=chain_of(s), est_min=s["est"], est_src=s["est_src"], deps=s["deps"], status=stt, why=why, cmd=s["cmd"],
                         out=s["out"], soft=s["soft"], needs=s["needs"], manual=s["manual"], optional=s["optional"], note=s["note"]))
    if as_json:
        print(json.dumps(rows, indent=1)); return
    print("%-28s %-4s %8s  %-22s %-11s %s" % ("stage", "step", "est", "chain", "status", "depends on"))
    for r in rows:
        print("%-28s %-4s %8s  %-22s %-11s %s" % (r["name"], r["step"], fmt_min(r["est_min"]) + ("*" if r["est_src"] == "guess" else ""), r["chain"], r["status"],
                                                  ",".join(r["deps"]) or "-"))
    print("\n* = estimate not quoted in DESIGN_STATE section 9.  step 1*/2*/4* = placed differently from section 9 (see notes: run_all.py --list --json).")
    dflt = [r for r in rows if not r["manual"]]; man = [r for r in rows if r["manual"]]
    print("default chain: %d stages, %s if everything ran.  manual: %d stages in groups %s (%s), %d absent."
          % (len(dflt), fmt_min(sum(r["est_min"] for r in dflt)), len(man), ", ".join(GROUPS), fmt_min(sum(r["est_min"] for r in man if r["status"] != "absent")),
             sum(r["status"] == "absent" for r in rows)))
    soft = [r for r in rows if r["soft"]]
    if soft:
        print("soft (read-back, not hashed) inputs: " + "; ".join("%s <- %s" % (r["name"], ", ".join(r["soft"])) for r in soft))


def cmd_plan(root, p, as_json, header="PLAN"):
    tot = sum(s["est"] for s, _ in p)
    if as_json:
        print(json.dumps([dict(stage=s["name"], chain=chain_of(s), est_min=s["est"], cmd=s["cmd"], reasons=w) for s, w in p], indent=1)); return
    if not p:
        print("%s: nothing to do, all stages fresh." % header); return
    print("%s: %d stage(s), est. %s" % (header, len(p), fmt_min(tot)))
    for i, (s, w) in enumerate(p, 1):
        print("%2d. %-28s %-8s %-18s %s" % (i, s["name"], fmt_min(s["est"]), chain_of(s), "; ".join(w)))
        print("      $ " + s["cmd"])
        miss = [n for n in s["needs"] if not os.path.exists(os.path.join(root, n))]
        if miss:
            print("      ! will fail: missing external input(s) " + ", ".join(miss))
    print("(dry-run is an upper bound: a downstream stage is skipped in a real run if its upstream regenerated byte-identical outputs)")


def run_stage(root, s, st, run_id):
    missing_need = [n for n in s["needs"] if not os.path.exists(os.path.join(root, n))]
    if missing_need:
        print("   cannot run: missing external input(s): " + ", ".join(missing_need)); return 127
    ins = inputs_hash(root, s)
    ups = {d: out_digest(root, BYNAME[d]) for d in s["deps"]}
    key = hashlib.sha256((ins + json.dumps(ups, sort_keys=True)).encode()).hexdigest()[:16]
    env = dict(os.environ, TS_STAGE_KEY=key, TS_RUN_ID=run_id, TS_STAGE=s["name"])
    logd = os.path.join(root, "logs", "pipeline"); os.makedirs(logd, exist_ok=True)
    lp = os.path.join(logd, s["name"] + ".log")
    t0 = time.time()
    with open(lp, "w") as lf:
        rc = subprocess.call(s["cmd"], shell=True, cwd=root, env=env, stdout=lf, stderr=subprocess.STDOUT)
    dt = time.time() - t0
    _, missing = out_files(root, s)
    if rc == 0 and missing:
        print("   exited 0 but required outputs are missing: %s" % missing); rc = 3
    if rc != 0:
        with open(lp) as lf:
            tail = lf.read().splitlines()[-15:]
        print("   FAILED rc=%d (%.0f s), last log lines (%s):" % (rc, dt, os.path.relpath(lp, root)))
        print("\n".join("   | " + l for l in tail))
    st["stages"][s["name"]] = dict(inputs=ins, upstream={d: out_digest(root, BYNAME[d]) for d in s["deps"]}, out_digest=out_digest(root, s), key=key,
                                   returncode=rc, seconds=round(dt, 1), ran_at=time.strftime("%Y-%m-%dT%H:%M:%S"), run_id=run_id)
    save_state(root, st)
    return rc


def cmd_run(root, st, p_targets, forced, with_manual, keep_going):
    run_id = time.strftime("%Y%m%dT%H%M%S"); failed, ran = set(), []
    first = plan(root, st, p_targets, forced, with_manual, dry=True)
    cmd_plan(root, first, False, "RUNNING")
    todo = [s for s, _ in first]
    for s in todo:
        bad = [d for d in s["deps"] if d in failed]
        if bad:
            print("-- skip %-26s (upstream failed: %s)" % (s["name"], ",".join(bad))); failed.add(s["name"]); continue
        stale, why = evaluate(root, s, st, set(ran), forced, dry=False)   # early cutoff: identical regenerated outputs do not wake downstream
        if not stale:
            print("-- skip %-26s (upstream outputs unchanged)" % s["name"]); continue
        print(">> %-28s %s  [%s]" % (s["name"], fmt_min(s["est"]), "; ".join(why)))
        rc = run_stage(root, s, st, run_id)
        if rc != 0:
            failed.add(s["name"])
            if not keep_going:
                print("stopping at first failure (use --keep-going to continue independent stages)"); break
        else:
            ran.append(s["name"])
    print("done: ran %d, failed %d%s" % (len(ran), len(failed), (" (" + ", ".join(sorted(failed)) + ")") if failed else ""))
    return 1 if failed else 0


def cmd_adopt(root, st, names):
    n = 0
    for s in STAGES:
        if names and s["name"] not in names:
            continue
        if is_absent(root, s):
            continue
        _, missing = out_files(root, s)
        if missing:
            print("skip %-26s missing outputs %s" % (s["name"], missing[:2])); continue
        st["stages"][s["name"]] = dict(inputs=inputs_hash(root, s), upstream={d: out_digest(root, BYNAME[d]) for d in s["deps"]}, out_digest=out_digest(root, s),
                                       key="adopted", returncode=0, seconds=0, ran_at=time.strftime("%Y-%m-%dT%H:%M:%S"), run_id="adopt")
        n += 1
    save_state(root, st); print("adopted %d stage(s) as up to date (no scripts were run)" % n)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog="see the module docstring (head of this file) for the staleness rules")
    ap.add_argument("--list", action="store_true"); ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--stage", action="append", default=[], help="force this stage (repeatable); downstream stale stages follow")
    ap.add_argument("--group", action="append", default=[], help="force every stage of a manual group (%s)" % ", ".join(GROUPS))
    ap.add_argument("--with-upstream", action="store_true"); ap.add_argument("--no-downstream", action="store_true")
    ap.add_argument("--force", action="store_true"); ap.add_argument("--with-manual", action="store_true"); ap.add_argument("--keep-going", action="store_true")
    ap.add_argument("--adopt", nargs="*", default=None, metavar="NAME"); ap.add_argument("--forget", action="append", default=[])
    ap.add_argument("--json", action="store_true"); ap.add_argument("--root", default=ROOT)
    a = ap.parse_args(argv); root = os.path.abspath(a.root); st = load_state(root)
    for g in a.group:
        if g not in GROUPS:
            ap.error("unknown group %r; groups: %s" % (g, ", ".join(GROUPS)))
        a.stage += [s["name"] for s in STAGES if s["manual"] and s["group"] == g and not is_absent(root, s)]
    for n in a.stage + a.forget + (a.adopt or []):
        if n not in BYNAME:
            ap.error("unknown stage %r; try --list" % n)
    if a.forget:
        for n in a.forget:
            st["stages"].pop(n, None)
        save_state(root, st); print("forgot", a.forget)
    if a.adopt is not None:
        return cmd_adopt(root, st, set(a.adopt))
    if a.list:
        return cmd_list(root, st, a.json)
    forced = set(a.stage)
    if a.force:
        forced |= {s["name"] for s in STAGES if not s["manual"] or a.with_manual}
    targets = None
    if a.stage:
        targets = set(a.stage) if a.no_downstream else downstream(a.stage)
        if a.with_upstream:
            targets |= upstream(a.stage)
    if a.dry_run:
        return cmd_plan(root, plan(root, st, targets, forced, a.with_manual, dry=True), a.json)
    return cmd_run(root, st, targets, forced, a.with_manual, a.keep_going)


if __name__ == "__main__":
    sys.exit(main() or 0)
