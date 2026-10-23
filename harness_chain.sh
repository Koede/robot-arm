#!/usr/bin/env bash
# Rebuild wearer + harness stages (about 15 min). Run from repo root.
set -e
python3 cad/wearer_makehuman.py --back_to_p 108
for s in "muscle_atlas.py --rules" "clasp_shoulder.py --facets_only" "organic_connector.py" "muscle_atlas.py" "chassis_axilla.py" "scapular_frame.py" "shoulder_cowl.py" "pauldron.py"; do
  echo "== $s"; python3 cad/$s > logs_$(echo $s | cut -d. -f1).log 2>&1
done
