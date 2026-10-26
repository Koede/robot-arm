#!/usr/bin/env python3
"""Third Shoulder: run the heavy jobs in parallel on this machine. From the repo root:
   python run_all_local.py [--setup] [--skip-rebuild] [--jobs N]
     --setup         create .venv and install the dependencies (done automatically if .venv is missing)
     --skip-rebuild  skip the wearer / harness / SDF rebuild (use the meshes and caches already in stl/)
     --jobs N        parallel workers (default: all cores)
Order matters: harness meshes first, THEN the SDF cache (the cache reads the cowl, pauldron, wing and plate meshes).
Logs: logs/local_*.log, summary in logs/local_driver.log, copies of the key results in results/."""
import argparse, os, shutil, subprocess, sys, time, venv
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path.cwd()
assert (ROOT / "cad").is_dir(), "run this from the repo root (the folder containing cad/ and stl/)"
ap = argparse.ArgumentParser()
ap.add_argument("--setup", action="store_true")
ap.add_argument("--skip-rebuild", action="store_true")
ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
args = ap.parse_args()

(ROOT / "logs").mkdir(exist_ok=True); (ROOT / "results").mkdir(exist_ok=True)
DRIVER = ROOT / "logs" / "local_driver.log"
def say(msg):
    line = f"[{time.strftime('%Y-%m-%dT%H:%M:%S')}] {msg}"; print(line, flush=True)
    with open(DRIVER, "a") as f: f.write(line + "\n")

PY = str(ROOT / ".venv" / ("Scripts/python.exe" if os.name == "nt" else "bin/python"))
if args.setup or not Path(PY).exists():
    say("creating .venv and installing dependencies (Python 3.10-3.13 needed)")
    venv.EnvBuilder(with_pip=True).create(ROOT / ".venv")
    subprocess.run([PY, "-m", "pip", "install", "-q", "--upgrade", "pip"], check=True)
    subprocess.run([PY, "-m", "pip", "install", "-q", "cadquery", "trimesh", "manifold3d", "scikit-image", "rtree",
                    "python-fcl", "scipy", "fast-simplification", "shapely", "scikit-fem", "matplotlib", "numpy"], check=True)
if subprocess.run([PY, "-c", "import cadquery, trimesh, fcl, skimage, shapely"]).returncode != 0:
    sys.exit("dependencies missing (python-fcl is the usual culprit); run with --setup")

env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")   # one core per job so N jobs scale
OBJ = str(ROOT / "external" / "makehuman_base" / "base.obj")

def run(cmd, cwd, log):
    with open(ROOT / "logs" / log, "w") as f:
        return subprocess.run(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT, env=env).returncode

if not args.skip_rebuild:
    say("stage 1: wearer + harness (about 15 min, one core)")
    steps = [["cad/wearer_makehuman.py", "--back_to_p", "108", "--obj", OBJ]] + \
            [["cad/" + s.split()[0]] + s.split()[1:] for s in
             ["muscle_atlas.py --rules", "clasp_shoulder.py --facets_only", "organic_connector.py", "muscle_atlas.py",
              "chassis_axilla.py", "scapular_frame.py", "shoulder_cowl.py", "pauldron.py"]]
    for i, st in enumerate(steps):
        if run([PY, "-u"] + st, ROOT, f"local_1_harness_{i}.log") != 0:
            sys.exit(f"harness step failed: {' '.join(st)} (see logs/local_1_harness_{i}.log)")
    say("stage 2: SDF cache (about 5 min)")
    if run([PY, "-u", "arm_wiring_lib.py", "--rebuild"], ROOT / "cad", "local_2_sdf.log") != 0:
        sys.exit("SDF rebuild failed (see logs/local_2_sdf.log)")

# name -> commands run in order in cad/ (a chain stops at its first failure); longest first
JOBS = [
    ("wire_W2", [["wire_search_combined.py", "W2", "400", "2", "30"]]),
    ("wire_W3", [["wire_search_combined.py", "W3", "400", "2", "30"]]),
    ("sweep_full", [["sweep_full.py"]]),
    ("sweep_full_restricted", [["sweep_full.py", "--restricted"]]),
    ("collision", [["collision_frontier.py"], ["collision_separable.py"], ["collision_screw.py"]]),
    ("balancer", [["verify_balancer.py"]]),
    ("worst_full", [["sweep_harness_worst.py", "15"]]),
    ("worst_restricted", [["sweep_harness_worst.py", "--restricted", "15"]]),
    ("wrist_search", [["wrist_parallel_search.py", "--seed", "3", "--out", "kin_local.json"]]),
]
def job(item):
    name, chain = item; t0 = time.time(); rc = 0
    for c in chain:
        rc = run([PY, "-u"] + c, ROOT / "cad", f"local_{name}.log")
        if rc: break
    say(f"job {name} exit {rc} ({time.time() - t0:.0f} s)")
    return name, rc

say(f"stage 3: {len(JOBS)} jobs on {args.jobs} workers")
with ThreadPoolExecutor(max_workers=args.jobs) as ex: results = list(ex.map(job, JOBS))
for pat in ("stl/electronics/wire_search_*_CB.json", "stl/sweep_full*.json", "stl/sweep_harness_worst*.json", "stl/elbow_limits_15mm*.json",
            "stl/balancer/verify_balancer.json", "stl/wrist_parallel/kin_local.json"):
    for p in ROOT.glob(pat): shutil.copy(p, ROOT / "results" / p.name)
say("ALL_DONE: " + ", ".join(f"{n}={'ok' if rc == 0 else 'FAILED'}" for n, rc in results))
