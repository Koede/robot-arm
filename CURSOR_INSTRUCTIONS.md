# Third Shoulder: heavy jobs on a local machine (instructions for Cursor)

You are running pre-built CAD-as-code scripts for a wearable robot-arm project. Do not redesign anything. Run the jobs below, report results, and do not edit any file in `cad/` unless a command fails because of a missing dependency or path.

## Layout
Repo root contains `cad/` (scripts), `stl/` (inputs and outputs), `external/makehuman_base/base.obj` (CC0 body mesh), `run_local.sh`, `harness_chain.sh`. Run every command from the repo root unless stated. Units are mm and degrees.

## 0. Unpack
Unzip ALL five archives (`ts1_code.zip` ... `ts5_misc.zip`) into the same empty folder; they merge into one repo (`cad/`, `stl/`, `external/`, scripts). Left-arm (`*_L.stl`), the SDF cache and the harness outputs are not included: the SDF cache is rebuilt in step 2, the harness in step 2b.

## 1. Setup (once)
```
python3 -m venv .venv && source .venv/bin/activate
pip install cadquery trimesh manifold3d scikit-image rtree python-fcl scipy fast-simplification shapely scikit-fem matplotlib numpy
python3 -c "import cadquery, trimesh, fcl, skimage, shapely; print('deps ok')"
```
Python 3.10 to 3.13. If `python-fcl` fails to build, report the error and stop.

## 2. Rebuild the SDF cache (about 5 min)
```
cd cad && python3 arm_wiring_lib.py --rebuild && cd ..
```

## 2b. Rebuild the wearer and harness meshes (about 15 min, one core)
```
./harness_chain.sh
```
It must finish with no traceback. Check that these exist: `stl/shoulder/proxies/wearer_makehuman.stl`, `stl/shoulder/clasp_frame_R.stl`, `stl/shoulder/chassis_plate_R.stl`, `stl/shoulder/scapular/wing_R.stl`, `stl/shoulder/cowl/cowl_R.stl`, `stl/shoulder/pauldron/pauldron_R.stl`. Do not pipe any script's output through `head` (it kills the script before it writes its JSON).

## 3. Heavy jobs (parallel, uses all cores)
```
./run_local.sh
```
This runs four jobs at once:
- `wire_search_combined.py W2 400 2 30` and `... W3 400 2 30` (cable loop re-search against 30 combined poses). Writes `stl/electronics/wire_search_W2_CB.json` and `..._W3_CB.json`.
- `sweep_full.py` and `sweep_full.py --restricted` (full collision sweeps against the wearer and harness). Write `stl/sweep_full.json`, `stl/sweep_full_restricted.json`, `stl/elbow_limits_15mm*.json`.
The script prints `ALL_DONE` and copies the results into `results/`.

## 4. Optional extras if time allows (run one at a time, from `cad/`)
```
python3 -u collision_screw.py            # certified collision tracer; compare to collision_frontier.npz
python3 -u collision_separable.py        # separable tables
python3 -u verify_balancer.py            # about 10 min
python3 -u balancer_freefall.py          # about 30 min
```

## 5. What to send back
Zip and return: `results/`, `logs/`, `stl/electronics/*.json`, `stl/sweep_full*.json`, and the console tail of any job that failed. Also report wall-clock time per job, core count, and OS.

## Rules
- Never delete or overwrite files in `stl/` by hand; scripts regenerate them.
- If a job crashes, send the full traceback and the last 30 log lines. Do not retry with changed parameters.
- Do not commit or push anything.
