# Jobs for a many-core local machine

Heavy runs that are slow on the 4-core cloud VM. Run from the repo root in an activated venv (setup in `README.md`; the body mesh must be linked at `/home/claude/external/makehuman_base/base.obj`, see `CLAUDE.md`). Do not pipe a script's output through `head`. Do not edit `stl/` by hand; scripts regenerate it. Timings are from `CURSOR_INSTRUCTIONS.md` and the logs, not re-measured.

## 0. Rebuild order (once per geometry change)
```
cd cad && python3 -u arm_wiring_lib.py --rebuild && cd ..    # SDF cache, ~5 min
./harness_chain.sh                                           # wearer + harness, ~15 min, one core
```

## 1. Sweeps and cable-loop search (all cores)
```
./run_local.sh      # wire_search_combined W2/W3 (400 candidates, 30 bases) + sweep_full.py + sweep_full.py --restricted; prints ALL_DONE
```
The restricted sweep alone took 1601 s on 2 cores. The W2/W3 searches did not finish on the VM (75/400 and 8/400 candidates in ~30 min), so give them the full machine.

## 2. Collision method validation (one at a time, from `cad/`)
```
python3 -u collision_frontier.py       # reference grid, ~6 min on 2 cores
python3 -u collision_separable.py      # must reproduce the contact mask exactly
python3 -u collision_screw.py          # certified fallback; compare with collision_frontier.npz
```

## 3. Design-study searches (each ~9 min per run on one core; run in parallel on separate cores)
```
python3 -u wrist_parallel_search.py    # parallel wrist (branch worktree-agent-ab4e0da4b2db08c19): add a hard forearm-axis exclusion, or restrict W2 to -50..40
python3 -u wrist_parallel_check.py     # clearance and roll check of the result (do NOT pipe through head)
```
Agent branches with the studies: `worktree-agent-ab4e0da4b2db08c19` (parallel wrist), `worktree-agent-a5686402ac2f58a49` (differential wrist), `worktree-agent-a0ccef773ea89ff91` (four-bar elbow), `worktree-agent-aaf01cfdf9e87a085` (parallel shoulder), `worktree-agent-acc63c4dd1ece6073` (connectors). Each has its report in `docs/`.

## 4. Optional extras
```
python3 -u verify_balancer.py          # ~10 min
python3 -u balancer_freefall.py        # ~30 min (script not in the repo yet)
```

## What to send back
`results/`, `logs/`, `stl/electronics/*.json`, `stl/sweep_full*.json`, and the console tail of any failed job, plus wall-clock time per job and the core count.
