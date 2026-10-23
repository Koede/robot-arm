"""Tiny numpy ray caster against the wearer mesh (no rtree dependency)."""
import numpy as np, trimesh, os
HERE = os.path.dirname(os.path.abspath(__file__))
def load():
    m = trimesh.load(os.path.join(HERE, "..", "stl", "shoulder", "proxies", "wearer_makehuman.stl")); return m
def cast(m, o, d):
    """first hit of ray o + t d (t>0). returns (point, face normal) or None"""
    o = np.asarray(o, float); d = np.asarray(d, float); d = d / np.linalg.norm(d)
    T = m.triangles; v0 = T[:, 0]; e1 = T[:, 1] - v0; e2 = T[:, 2] - v0
    p = np.cross(d, e2); det = np.einsum("ij,ij->i", e1, p)
    ok = np.abs(det) > 1e-12; inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
    s = o - v0; u = np.einsum("ij,ij->i", s, p) * inv
    q = np.cross(s, e1); v = (q @ d) * inv; t = np.einsum("ij,ij->i", e2, q) * inv
    hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
    if not hit.any(): return None
    i = np.where(hit)[0][np.argmin(t[hit])]
    return o + t[i] * d, m.face_normals[i]
def smooth_normal(m, pt, r=14.0, min_faces=8):
    """area-weighted normal of the faces whose centres lie within r of pt; r grows until min_faces faces are included
    (the base mesh is low-poly in places, so a fixed radius can contain no face centres)"""
    c = m.triangles_center; d = np.linalg.norm(c - pt, axis=1)
    while (d < r).sum() < min_faces: r *= 1.25
    w = d < r
    fn = np.nan_to_num(m.face_normals[w]); a = np.nan_to_num(m.area_faces[w])
    n = (fn * a[:, None]).sum(0); return n / np.linalg.norm(n)
