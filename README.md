# robot-arm

Third Shoulder is a wearable robot arm, described as CAD-as-code. Units are millimetres and degrees. The right arm is the generated set; left-arm meshes are mirrors the scripts write when a stage builds both sides.

## Layout

- `cad/` — build and check scripts
- `stl/` — part meshes and JSON results
- `external/makehuman_base/base.obj` — CC0 MakeHuman body mesh
- `harness_chain.sh` — rebuild the wearer and the shoulder harness, including the faceted pauldron
- `run_local.sh` — cable-loop search and full collision sweeps

The pauldron is the faceted shoulder socket from `cad/pauldron.py`. The lateral side is a swept deltoid shell, not a flat shelf: the crown sits 12 mm off the deltoid, the rim curls inboard, and the outer edge stays near y = 85 instead of running out to y = 98. Bore, plate feet, and the acromion pad are unchanged. `harness_chain.sh` (or `cad/pauldron.py` on its own, after the harness meshes exist) writes `stl/shoulder/pauldron/`:

- `pauldron_R.stl` — assembled shell used by the sweeps
- `pauldron_core_R.stl`, `pauldron_arm_lat_R.stl`, `pauldron_arm_med_R.stl` — print split for a 256 mm build volume, joined by half-lap splices
- `pauldron.json` — extents, mass, splice frames, and clearance carve

## Run locally

Python 3.10–3.13.

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install cadquery trimesh manifold3d scikit-image rtree python-fcl scipy fast-simplification shapely scikit-fem matplotlib numpy
python3 -c "import cadquery, trimesh, fcl, skimage, shapely; print('deps ok')"
```

`cad/wearer_makehuman.py` reads the body from `/home/claude/external/makehuman_base/base.obj`. Point that path at the mesh in this repo before the harness (the script does not take the path from `harness_chain.sh`):

```bash
sudo mkdir -p /home/claude/external/makehuman_base
sudo ln -sfn "$PWD/external/makehuman_base/base.obj" /home/claude/external/makehuman_base/base.obj
```

Then, from the repo root:

```bash
cd cad && python3 arm_wiring_lib.py --rebuild && cd ..
./harness_chain.sh
./run_local.sh
```

`harness_chain.sh` must finish with no traceback. `run_local.sh` prints `ALL_DONE` and copies sweep and wire-search JSON into `results/`. Do not pipe a script’s output through `head`; that stops it before it writes JSON.
