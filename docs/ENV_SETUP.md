# Environment bootstrap (read this first; do not rediscover it each session)
**Repo:** `github.com/Koede/robot-arm` (add with add_repo, access push; ONE shallow clone: `git clone --depth 1 https://github.com/Koede/robot-arm /home/claude/robot-arm`, 10 min timeout, then register_repo_root). Work on a `claude/...` branch, never main.
**Python deps (one command):** `pip install --break-system-packages cadquery trimesh manifold3d scikit-image rtree python-fcl scipy shapely mapbox-earcut fast-simplification networkx`
**Wearer mesh:** `curl -sSL -o external/makehuman_base/base.obj https://raw.githubusercontent.com/makehumancommunity/makehuman/master/makehuman/data/3dobjs/base.obj` then `python3 -I cad/wearer_makehuman.py --back_to_p 108` (the repo script; the concept study used a condensed copy with identical output).
**Rendering:** Chromium + Playwright are preinstalled; `cd render_tools && npm i three`; `cad/j1r2/render_views.py` (three.js, headless, SwiftShader) + `cad/j1r2/compose.py` (sheets). Render one view per browser launch for heavy meshes.
**Machine:** 2 cores. A whole-arm sweep (170,100 poses) takes about 9-10 min; shoulder-level sweep about 9 s. Do not run more than 2 compute jobs at once.
**J1 rev 2 workflow:** `cad/j1r2/j1r2_params.py` (frames, ranges) -> `j1r2_geom.py` build() -> `j1r2_kin.py` -> `j1r2_sweep.py` -> `j1r2_integrate.py` / `j1r2_opt.py` (pad scan) -> `j1r2_real.py` (swap proxies for the repo's real wing / clasp / distal / HAND rev B STLs). Library: `clib.py`.
**Project doc store (claude.ai Project "Robot Arm")** holds the original DESIGN_STATE.md, firmware headers (`firmware/third_shoulder/*.h`, `gen_tables.py`) and specs that are not in this repo.
**Subagents:** forks inherit the full context and are expensive; use at most 2-3 at once and only for separable heavy work.

**Work root (2026-11-02):** `clib.ROOT` reads `J1R2_ROOT`. Make a work root with `stl/proxies -> <repo>/stl/shoulder/proxies`, `stl/j1r2 -> <repo>/stl/j1r2`, `render -> <repo>/render_tools`, `j1r2 -> <repo>/cad/j1r2` (symlinks), then `export J1R2_ROOT=<work root>`. The mount study (`cad/j1r2/mj_*.py`) renders with `cad/render_three.py`.
