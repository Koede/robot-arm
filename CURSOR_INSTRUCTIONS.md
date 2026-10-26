# Third Shoulder: heavy jobs on a local machine (instructions for Cursor)

You are running pre-built CAD-as-code scripts for a wearable robot-arm project. Do not redesign anything. Run the jobs below, report results, and do not edit any file in `cad/` or `stl/` unless a command fails because of a missing dependency or path. Read `CLAUDE.md` and `README.md` first for the conventions (mm and degrees, right arm generated, never edit `stl/` by hand, never pipe a script's output through `head`).

## 0. Get the repo
`git clone https://github.com/Koede/robot-arm` then `git checkout claude/friendly-euler-59spol` (the branch with the latest work and `run_all_local.py`). Everything is in the repo: `cad/` scripts, `stl/` inputs and outputs, `external/makehuman_base/base.obj` (the body mesh). Run every command from the repo root.

## 1. Run everything (one command)
```
python run_all_local.py --setup
```
`--setup` creates `.venv` and installs the dependencies (Python 3.10 to 3.13; if `python-fcl` fails to install, report the full error and stop). The script then:
1. rebuilds the wearer and harness meshes (about 15 min, one core), then the SDF cache (about 5 min). The order is harness first, then SDF, because the cache reads the cowl, pauldron, wing and plate meshes;
2. runs 9 jobs in parallel, one core each: `wire_search_combined.py W2` and `W3` (400 candidates, 30 combined-pose bases), `sweep_full.py` and `--restricted`, the three collision-method checks (`collision_frontier`, `collision_separable`, `collision_screw`), `verify_balancer.py`, `sweep_harness_worst.py` (full and `--restricted`), and a `wrist_parallel_search.py` re-search.

Useful flags: `--skip-rebuild` (reuse the meshes already in `stl/`), `--jobs N` (workers, default all cores). It prints `ALL_DONE: name=ok|FAILED ...` at the end. Logs: `logs/local_*.log`; summary and per-job seconds: `logs/local_driver.log`.

If a job fails, send the full traceback and the last 30 lines of its log. Do not retry with changed parameters.

## 2. What to send back
Zip and return: `results/`, `logs/`, `stl/electronics/*.json`, `stl/sweep_full*.json`, `stl/sweep_harness_worst*.json`, `stl/balancer/verify_balancer.json`, `stl/wrist_parallel/kin_local.json`, and the console tail of any job that failed. Also report wall-clock time per job (from `logs/local_driver.log`), the core count, and the OS.

## 3. Optional extras (one at a time, from `cad/`, only if asked)
```
python -u balancer_freefall.py     # about 30 min; the script is not in the repo yet, report that and skip if it is missing
```
More jobs are listed in `docs/LOCAL_JOBS.md`.

## Rules
- Never delete or overwrite files in `stl/` by hand; scripts regenerate them.
- Do not commit or push anything unless the user asks.
- Do not change parameters, limits or geometry. Design decisions are the user's call.
