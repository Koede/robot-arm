#!/bin/bash
# usage: J1R2_ROOT=.. bash mj11_possweep.sh  -> stl/j1r2_mounts/mj11_possweep.jsonl  (one real-parts sweep per pad position, one process each because mj_frame reads J1_PAD at import)
OUT=/home/claude/robot-arm/stl/j1r2_mounts/mj11_possweep.jsonl; : > $OUT
for z in -140 -165 -190; do for y in -60 -45 -30 -15; do
  J1_PAD="$y,$z,13" MJ11_CLASP_PSI=180 python3 -I - >> $OUT 2>/dev/null <<PY
import sys, json; sys.path.insert(0, '.')
import mj11_sweep as W, mj_concepts3 as C3, mj_frame as M, numpy as np
try:
    parts = C3.mj11_geo(None); r, _ = W.run(parts); Y,Z,sk,inn,pl = C3.seat_surfaces(); m = np.hypot(Y-C3.Y1, Z-C3.Z1) <= 66; t = (pl-inn)[m]
    r['dish_mm'] = [round(float(t.min()),1), round(float(t.mean()),1), round(float(t.max()),1)]; r['mass_g'] = round(sum(abs(p['mesh'].volume)/1000*({'PACF':1.15,'PETGCF':1.3}.get(p['mat'],1.2))*p.get('fill',1.0) if p.get('mass_g') is None else p['mass_g'] for p in parts.values()),0)
    r['worst'] = r['worst'][:4]; print(json.dumps(r, default=float))
except Exception as e: print(json.dumps(dict(pad=[$y,$z,13], error=repr(e)[:200])))
PY
done; done
echo done
