"""Clock-spring bay check: is there an annular bay around a joint axis, free of every solid at every angle of the joint, in which a spiral of cable can live?
The spiral is anchored at one end on the fixed side (group A) and at the other on the rotating side (group B); as B turns by dq the spiral winds / unwinds.
The bay is the set of cylindrical-coordinate cells (R, h) about the axis, at ALL azimuths, that is free of group A solids (static) and of group B solids swept over the joint's hard range.
Output: free (R, h) map -> largest rectangle R0..R1 x h0..h1 -> number of turns of a bundle of OD d at pitch d + 1 and the cable length.
Usage: python3 arm_bay.py S1"""
import sys, os, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L, arm_wire_spec as S, arm_kin as ak, rom_limits as RL
from trimesh.transformations import rotation_matrix as rot
OUT = os.path.join(HERE, "..", "stl", "electronics")
# joint axis in the zero-pose robot frame (point, direction, radial reference direction, rotation sign from arm_kin)
AXIS = {"S1": dict(p=(0, 0, 0), d=(0, 0, 1), e1=(1, 0, 0), sign=-1, hmin=-70.0, hmax=-45.0, Rmin=44.0, Rmax=95.0),     # frames: R1 = Rz(-S1)
        "S2": dict(p=(0, 0, 0), d=(0, 1, 0), e1=(1, 0, 0), sign=-1, hmin=-12.0, hmax=22.0, Rmin=36.0, Rmax=95.0),      # R2 = R1 Ry(-S2)
        "S3": dict(p=(0, 0, 0), d=(0, 0, 1), e1=(1, 0, 0), sign=+1, hmin=108.0, hmax=150.0, Rmin=18.0, Rmax=80.0)}      # R3 = R2 Rz(+S3)

def frame_for(joint, ang):
    q = dict(S1=0, S2=0, S3=0, E=0, W1=0, W2=0, W3=0, G=50); q[joint] = ang
    return ak.frames(q)

def bay_map(joint, base=None, dphi=3.0, dR=1.0, dh=1.0, c=0.8, rstep=6.0):
    a = AXIS[joint]; gA, gB = S.CROSS[joint]["frm"], S.CROSS[joint]["to"]
    d = np.array(a["d"], float); e1 = np.array(a["e1"], float); e2 = np.cross(d, e1)
    Rs = np.arange(a["Rmin"], a["Rmax"] + 1e-9, dR); hs = np.arange(a["hmin"], a["hmax"] + 1e-9, dh); ph = np.radians(np.arange(0, 360, dphi))
    RR, HH, PP = np.meshgrid(Rs, hs, ph, indexing="ij")
    pts = np.array(a["p"], float) + HH[..., None] * d + RR[..., None] * (np.cos(PP)[..., None] * e1 + np.sin(PP)[..., None] * e2)
    P = pts.reshape(-1, 3); lo, hi = RL.HARD[joint]; angs = np.arange(lo, hi + 1e-9, rstep)
    # other groups (not A, B) are fixed relative to A or B depending on the chain: evaluate every group at every sampled pose of the joint, keep the worst per cell
    worst = np.full(len(P), 1e9)
    for ang in angs:
        F = frame_for(joint, ang)
        for g in L.GROUPS:
            Pl = L.to_local(F[g], P)
            try: v = L.sample_sdf(g, Pl)
            except ValueError: continue
            worst = np.minimum(worst, v)
    # the hub body (inner cylinder, rotating with B) is allowed inside R < Rmin: handled by Rmin; everything else must be free
    free = (worst.reshape(RR.shape) >= c).all(axis=2)          # all azimuths
    return Rs, hs, free, worst.reshape(RR.shape)

def largest_rect(free):
    best = (0, None); nR, nH = free.shape
    for i0 in range(nR):
        for j0 in range(nH):
            if not free[i0, j0]: continue
            i1 = i0
            while i1 + 1 < nR and free[i1 + 1, j0]: i1 += 1
            for j1 in range(j0, nH):
                col = free[i0:i1 + 1, j1]
                while len(col) and not col.all():            # shrink the radial extent
                    i1 -= 1; col = free[i0:i1 + 1, j1]
                if len(col) == 0: break
                area = (i1 - i0 + 1) * (j1 - j0 + 1)
                if area > best[0]: best = (area, (i0, i1, j0, j1))
    return best

if __name__ == "__main__":
    j = sys.argv[1]; Rs, hs, free, worst = bay_map(j)
    area, rect = largest_rect(free)
    print(joint := j, "free cells", int(free.sum()), "of", free.size)
    if rect is None: print("no free bay"); sys.exit()
    i0, i1, j0, j1 = rect; R0, R1, h0, h1 = Rs[i0], Rs[i1], hs[j0], hs[j1]; od = S.bundle(j, "ultra", 2 if j == "S1" else 1)["od_mm"]
    pitch = od + 1.0; turns = (R1 - R0 - od) / pitch + 1; Lcap = sum(2 * np.pi * (R0 + od / 2 + k * pitch) for k in range(int(turns)))
    print("largest free rectangle: R %.0f..%.0f mm, h %.0f..%.0f mm (height %.0f); bundle OD %.1f -> %.1f radial turns, %.0f mm of cable" % (R0, R1, h0, h1, h1 - h0, od, turns, Lcap))
    json.dump(dict(joint=j, R=[float(R0), float(R1)], h=[float(h0), float(h1)], od=od, turns=float(turns), cable_len_cap_mm=float(Lcap)), open(os.path.join(OUT, "bay_%s.json" % j), "w"), indent=1)
