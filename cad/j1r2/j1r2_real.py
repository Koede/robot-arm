"""Real-geometry layer for the J1 rev 2 sweep: loads the repo's verified STLs (wearer-fit wing, spine bridge, clasp, distal arm, HAND rev B) from /home/claude/robot-arm.
Distal groups are built in the baseline home frame (arm along +z from P) and translated to the J1 arm base O0, exactly like Kin.proxies()."""
import os, sys, json, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clib as L
REPO = "/home/claude/robot-arm"; R = os.path.join(REPO, "stl")
def ld(p):
    m = trimesh.load(os.path.join(R, p)); return m
def cat(ms): return trimesh.util.concatenate([m for m in ms if m is not None])
def stops(g):
    d = os.path.join(R, "distal", "stops_" + g); return [trimesh.load(os.path.join(d, f)) for f in sorted(os.listdir(d)) if f.endswith(".stl")] if os.path.isdir(d) else []
def distal_groups(O0, hand="hand_open_posed"):
    """-> {M3, ME, MW1, MW2, MW3} meshes in the J1 home frame (translated by O0)."""
    G = dict(
        M3=[ld("upperarm/ua_lat_R.stl"), ld("upperarm/ua_med_R.stl"), ld("upperarm/proxies/xm_elbow.stl")] + stops("M3"),
        ME=[ld("upperarm/fork_drive_R.stl"), ld("upperarm/fork_idler_R.stl"), ld("upperarm/fork_bridge_R.stl"), ld("distal/forearm_root_a_R.stl"), ld("distal/forearm_root_b_R.stl"), ld("distal/proxies/sts_w1_R.stl"), ld("distal/w1_pinion_R.stl")] + stops("ME"),
        MW1=[ld("distal/forearm_dist_R.stl"), ld("distal/proxies/sts_w2_R.stl")] + stops("MW1"),
        MW2=[ld("distal/w2_yoke_R.stl"), ld("distal/proxies/sts_w3_R.stl")] + stops("MW2"),
        MW3=[ld(f"hand/poses/{hand}.stl")] + stops("MW3"))
    out = {}
    for g, ms in G.items():
        m = cat(ms); m.apply_translation(np.asarray(O0, float)); out[g] = m
    return out
def fixed_real():
    """real fixed bodies (robot frame): wing (upper+lower), spine bridge, clasp frame, clasp facets (designed skin contact, excluded at park)."""
    return dict(wing=cat([ld("shoulder/scapular/wing_R.stl")]), spine_bridge=ld("shoulder/scapular/spine_bridge.stl"), clasp_frame=ld("shoulder/clasp_frame_R.stl"), clasp_facets=ld("shoulder/clasp_facets_R.stl"))
def seats_real():
    s = json.load(open(os.path.join(R, "shoulder", "clasp_sites.json"))); return {k: np.array(v["p"]) for k, v in s.items() if "p" in v}
if __name__ == "__main__":
    import j1r2_params as P, j1r2_kin as K
    D = P.derive(10.0, -30.0, 22.0); kin = K.Kin(D); g = distal_groups(kin.O0)
    for k, m in g.items(): print(k, m.bounds.round(0).tolist(), len(m.faces))
    f = fixed_real()
    for k, m in f.items(): print(k, m.bounds.round(0).tolist())
    print(seats_real())
