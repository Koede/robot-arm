"""Segmented chassis fairing: one skin per moving link group, offset T from the link solids, clipped so it stays >= 3 mm from every other group's swept skin (K_g >= T+3 over the legal poses).
shell_g = {sdf_g <= T and K_g >= T+3} minus {sdf_g <= T - w}  (open skin; the clipped edge is the joint gap).  Cable exposure = loop arc length outside every cavity {sdf_h <= T - w, K_h >= T+3}, per pose.
Usage: python3 fairing.py T [w]   -> stl/fairing/fair_<g>_R.stl, stl/fairing/fairing_T<T>.json"""
import os, sys, json, time, numpy as np, trimesh
from scipy import ndimage
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L, arm_keepout as K, arm_kin as ak, rom_limits as RL, arm_wiring_verify as V
OUT = os.path.join(HERE, "..", "stl", "fairing"); os.makedirs(OUT, exist_ok=True); EL = os.path.join(HERE, "..", "stl", "electronics")
MOV = ["M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"]; ORDER = ["S1", "S2", "S3", "E", "W1", "W2", "W3"]; RHO = 1.27e-3
CH = json.load(open(os.path.join(EL, "arm_chain_loops.json")))
PITCH = 2.0
def kfield(g, T, FR):
    """K_g and sdf_g on a PITCH grid, evaluated only in the band 0 < sdf <= T+3 (elsewhere K = +inf placeholder)"""
    sdf, lo, p = L.sdf_of(g); sub = sdf[::2, ::2, ::2]; lo2 = lo; ax = [lo2[i] + np.arange(sub.shape[i]) * 2 * p for i in range(3)]
    band = (sub <= T + 3.0)
    idx = np.argwhere(band); P = np.c_[ax[0][idx[:, 0]], ax[1][idx[:, 1]], ax[2][idx[:, 2]]]
    Kv = np.full(sub.shape, 1e3, np.float32); Kv[band] = K.keepout(g, P, FR).astype(np.float32)
    return sub, Kv, lo2, 2 * p
def sample(arr, lo, p, P): return ndimage.map_coordinates(arr, ((np.asarray(P) - lo) / p).T, order=1, mode="nearest", cval=1e3)
def loops_at(q):
    F = ak.frames(q); out = {}
    for j in ORDER:
        c = CH["joints"][j]; r = CH["od"][j] / 2; gA, gB = CH["frm"][j], CH["to"][j]; x = None
        for t in np.linspace(0, 1, 7):
            qq = dict(q); qq[j] = float((1 - t) * 0.5 * sum(RL.SOFT[j]) + t * q[j]); Fq = ak.frames(qq)
            Pa = (Fq[gA] @ np.r_[c["A"], 1])[:3]; Pb = (Fq[gB] @ np.r_[c["B"], 1])[:3]; ta = Fq[gA][:3, :3] @ np.array(c["tA"]); tb = Fq[gB][:3, :3] @ np.array(c["tB"])
            x, info = L.rod_relax(Pa, ta, Pb, tb, c["L"], Fq, r, 0.8, x0=x)
        out[j] = (x, r)
    return F, out
if __name__ == "__main__":
    T = float(sys.argv[1]); w = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5; t0 = time.time()
    FR, n = V.legal_poses(K.frames_list(K.pose_list(70, 11)), 3.0); print(n, "legal poses for K", flush=True)
    fields = {g: kfield(g, T, FR) for g in MOV}; print("K fields %.0fs" % (time.time() - t0), flush=True)
    rec = {}; gm = L.build_group_meshes()
    for g in MOV:
        sub, Kv, lo, p = fields[g]; outer = (sub <= T) & (Kv >= T + 3.0); cav = sub <= T - w; skin = outer & ~cav
        # sdf values here are voxel-centre distances; marching cubes on a smooth field
        f_out = np.maximum(sub - T, (T + 3.0) - np.minimum(Kv, T + 20)).astype(np.float32); f_cav = (sub - (T - w)).astype(np.float32)
        from skimage.measure import marching_cubes
        try:
            vo, fo, _, _ = marching_cubes(np.pad(f_out, 1, constant_values=50), 0.0, spacing=(p,) * 3); vo += lo - p
            vc, fc, _, _ = marching_cubes(np.pad(f_cav, 1, constant_values=50), 0.0, spacing=(p,) * 3); vc += lo - p
            mo = trimesh.Trimesh(vo, fo); mc = trimesh.Trimesh(vc, fc); mo.fix_normals(); mc.fix_normals()
            sk = trimesh.boolean.difference([mo, mc], engine="manifold")
            sk.export(os.path.join(OUT, "fair_%s_R.stl" % g)); rec[g] = dict(vol_cm3=round(float(sk.volume) / 1000, 2), mass_g=round(float(sk.volume) * RHO, 1), bbox=np.round(sk.extents, 0).tolist(), watertight=bool(sk.is_watertight))
        except Exception as e: rec[g] = dict(err=str(e)[:100])
        print(g, rec[g], "%.0fs" % (time.time() - t0), flush=True)
    # exposure of the cable loops
    rng = np.random.default_rng(3); qs = [q for q in K.pose_list(40, 21)]
    expo = {j: [] for j in ORDER}
    for q in qs:
        F, lp = loops_at(q)
        for j in ORDER:
            x, r = lp[j]; seg = np.linalg.norm(np.diff(x, axis=0), axis=1); mid = 0.5 * (x[1:] + x[:-1]); cov = np.zeros(len(mid), bool)
            for h in MOV:
                sub, Kv, lo, p = fields[h]; Pl = (np.c_[mid, np.ones(len(mid))] @ np.linalg.inv(F[h]).T)[:, :3]
                s = sample(sub, lo, p, Pl); kk = sample(Kv, lo, p, Pl); cov |= (s <= T - w - r) & (kk >= T + 3.0)
            expo[j].append(float(seg[~cov].sum() / seg.sum()))
    ex = {j: dict(mean=round(float(np.mean(v)), 3), max=round(float(np.max(v)), 3)) for j, v in expo.items()}
    print("exposure", ex)
    json.dump(dict(T=T, w=w, shells=rec, exposure=ex, total_mass_g=round(sum(v.get("mass_g", 0) for v in rec.values()), 1)), open(os.path.join(OUT, "fairing_T%g.json" % T), "w"), indent=1)
