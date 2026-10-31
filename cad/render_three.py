#!/usr/bin/env python3
"""render_three.py -- offline product-style renderer for CAD STL meshes.

Usage:  python3 render_three.py scene.json

Renders with three.js (WebGL2) inside headless Chromium via Playwright.
three.js is taken from the local npm install in cad/_three/node_modules/three
(created automatically with `npm install three` if missing; no CDN).

Scene JSON:
{
  "out_prefix": "/abs/path/prefix",     # writes <prefix>_<view>.png and <prefix>_sheet.png
  "title": "text shown on the sheet",
  "width": 1100, "height": 1100,        # per-view output size in px
  "parts": [ {"stl": "/abs/path.stl",
              "color": [r, g, b],       # 0-255
              "metal": 0.0,             # 0-1 metalness
              "rough": 0.5,             # 0-1 roughness
              "matrix": [[...4x4 row-major...]]   # optional, world mm (flat list of 16 also ok)
             } ],
  "up": [0, 0, 1],                      # world up axis
  "views": [ {"name": "palm", "dir": [1, 0, 0.15]} ]   # direction FROM target TOWARD camera
}
Optional keys (defaults in brackets):
  time_budget [45]    seconds for the whole run; frames per view adapt to it
  frames [24], min_frames [4]   progressive frames per view (subpixel AA + soft shadows + AO)
  ssaa [1]            extra supersampling factor (multiplies pixel cost)
  fill [0.8]          fraction of the frame the model fills
  crease [35]         crease angle (deg) for smooth normals
  ground [true]       contact shadow under the model (hidden for views from below)
  outline [0.55]      edge-line strength (0 = off); outline_crease [0.6], outline_crease_deg [40]
  exposure [0.9], key [2.6], key_dir [[0.7,-0.9,0.9] in (toward-camera, camera-right, up)],
  key_soft [0.05 rad], sky [0.9] (AO light), probe [0.22], hemi [0.35], fill_light [0.3], rim [1.0]
  reflections [false] PMREM reflections on all parts (always on for metal >= 0.3 or dark colours)
  max_tris [400000]   above this, sub-pixel triangles are removed by vertex clustering
  background [[247,248,250],[205,210,218]]  top/bottom gradient; sheet_max_width [4400]
  per view: "label" (sheet caption, default = name)
Camera: orthographic, auto-framed on the projected vertices of all parts.
Lighting: key light + sky light with jittered shadow maps (accumulated), SH light probe from
three's RoomEnvironment, ACES tone mapping, sRGB output; outlines from a 2x normal/depth pass.
Geometry is prepared in Python (merged vertices, creased normals, LOD, shadow proxies) and
cached in $TMPDIR/render_three_cache.
"""
import sys, os, json, time, base64, io, threading, subprocess, http.server, socketserver, hashlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
THREE_DIR = os.path.join(HERE, "_three")
THREE_PKG = os.path.join(THREE_DIR, "node_modules", "three")


def ensure_three():
    if os.path.isfile(os.path.join(THREE_PKG, "build", "three.module.js")):
        return
    os.makedirs(THREE_DIR, exist_ok=True)
    if not os.path.isfile(os.path.join(THREE_DIR, "package.json")):
        subprocess.run(["npm", "init", "-y"], cwd=THREE_DIR, check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["npm", "install", "three"], cwd=THREE_DIR, check=True)


# ---------------------------------------------------------------- geometry prep
CACHE_DIR = os.path.join(os.environ.get("TMPDIR", "/tmp"), "render_three_cache")
CACHE_VER = "v3"


def _rows_unique(a):
    """np.unique(axis=0) on an int64 2-D array, fast (void view). returns first, inverse."""
    a = np.ascontiguousarray(a)
    v = a.view(np.dtype((np.void, a.dtype.itemsize * a.shape[1]))).ravel()
    _, first, inv = np.unique(v, return_index=True, return_inverse=True)
    return first, inv.ravel()


def load_tri(path):
    """STL -> (V, F) with coincident vertices merged and degenerate faces dropped."""
    import trimesh
    m = trimesh.load(path, force="mesh", process=False)
    V = np.asarray(m.vertices, np.float64)
    F = np.asarray(m.faces, np.int64)
    if len(F) == 0:
        return V[:0], F[:0]
    scale = max(np.ptp(V, axis=0).max(), 1e-9)
    first, inv = _rows_unique(np.round(V / (scale * 1e-7)).astype(np.int64))
    F = inv[F]
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    return V[first], F


def face_count(path):
    with open(path, "rb") as f:
        head = f.read(84)
        if len(head) == 84 and os.path.getsize(path) == 84 + 50 * int.from_bytes(head[80:84], "little"):
            return int.from_bytes(head[80:84], "little")
    return len(load_tri(path)[1])


def cluster_decimate(V, F, cell):
    """Vertex clustering on a grid of size `cell` (mm): collapses detail finer than the cell,
    drops degenerate and duplicate faces. Used as a pixel-scale LOD and for shadow proxies."""
    q = np.floor(V / cell).astype(np.int64)
    first, inv = _rows_unique(q)
    n = len(first)
    cntv = np.bincount(inv, minlength=n).astype(np.float64)
    Vc = np.stack([np.bincount(inv, weights=V[:, k], minlength=n) for k in range(3)], 1) / cntv[:, None]
    F2 = inv[F]
    F2 = F2[(F2[:, 0] != F2[:, 1]) & (F2[:, 1] != F2[:, 2]) & (F2[:, 0] != F2[:, 2])]
    if len(F2):
        F2 = F2[np.sort(_rows_unique(np.sort(F2, axis=1))[0])]
    used = np.unique(F2)
    remap = np.full(n, -1, np.int64); remap[used] = np.arange(len(used))
    return Vc[used], remap[F2]


def stl_bounds(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        nt = int.from_bytes(data[80:84], "little")
        if len(data) == 84 + 50 * nt and nt > 0:
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=nt, offset=84)
            v = rec["v"].reshape(-1, 3)
            return v.min(0).astype(np.float64), v.max(0).astype(np.float64)
    V, _ = load_tri(path)
    return (V.min(0), V.max(0)) if len(V) else (np.zeros(3), np.zeros(3))


def creased_geometry(path, crease_deg, cell=0.0):
    """Faceted STL -> indexed (pos, normal, index) with smooth normals except across
    edges sharper than crease_deg (area-weighted face normals, like toCreasedNormals).
    cell > 0 first applies vertex-clustering LOD with that cell size (mm)."""
    Vm, F = load_tri(path)
    if len(F) == 0:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.float32), np.zeros(0, np.uint32)
    if cell > 0:
        Vm, F = cluster_decimate(Vm, F, cell)
        if len(F) == 0:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.float32), np.zeros(0, np.uint32)
    tri = Vm[F]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nf = fn / np.maximum(np.linalg.norm(fn, axis=1), 1e-30)[:, None]
    nF = len(F)
    cv = F.ravel()
    cf = np.repeat(np.arange(nF), 3)
    order = np.argsort(cv, kind="stable")
    fs = cf[order]
    cnt = np.bincount(cv, minlength=len(Vm))
    start = np.concatenate([[0], np.cumsum(cnt)[:-1]])
    deg = cnt[cv]
    tot = int(deg.sum())
    pc = np.repeat(np.arange(3 * nF), deg)
    off = np.arange(tot) - np.repeat(np.cumsum(deg) - deg, deg)
    pg = fs[start[cv[pc]] + off]
    ok = np.einsum("ij,ij->i", nf[cf[pc]], nf[pg]) >= np.cos(np.radians(crease_deg))
    N = np.empty((3 * nF, 3))
    for k in range(3):
        N[:, k] = np.bincount(pc, weights=fn[pg, k] * ok, minlength=3 * nF)
    N /= np.maximum(np.linalg.norm(N, axis=1), 1e-30)[:, None]
    key = np.concatenate([cv[:, None], np.round(N * 2000).astype(np.int64)], axis=1)
    first2, inv2 = _rows_unique(key)
    return Vm[cv[first2]].astype(np.float32), N[first2].astype(np.float32), inv2.astype(np.uint32)


def _prep_one(args):
    path, crease, cell = args
    st = os.stat(path)
    h = hashlib.sha1(f"{CACHE_VER}|{os.path.abspath(path)}|{st.st_size}|{st.st_mtime_ns}|{crease}|{cell:.6g}".encode()).hexdigest()
    fn = os.path.join(CACHE_DIR, h + ".npz")
    if os.path.isfile(fn):
        try:
            z = np.load(fn)
            return fn, z["p"], z["n"], z["i"]
        except Exception:
            pass
    p, n, i = creased_geometry(path, crease, cell)
    os.makedirs(CACHE_DIR, exist_ok=True)
    tmp = fn + f".{os.getpid()}.tmp.npz"
    np.savez(tmp, p=p, n=n, i=i)
    os.replace(tmp, fn)
    return fn, p, n, i


def _blob(P, Nn, I, M):
    if M is not None:
        M = np.asarray(flat_matrix(M), np.float64).reshape(4, 4)
        P = (P.astype(np.float64) @ M[:3, :3].T + M[:3, 3]).astype(np.float32)
        Nn = Nn.astype(np.float64) @ np.linalg.inv(M[:3, :3])      # inverse-transpose
        Nn = (Nn / np.maximum(np.linalg.norm(Nn, axis=1), 1e-30)[:, None]).astype(np.float32)
        if np.linalg.det(M[:3, :3]) < 0:                           # mirror: flip winding
            I = I.reshape(-1, 3)[:, ::-1].ravel().copy()
    return np.array([len(P), len(I)], np.uint32).tobytes() + P.tobytes() + Nn.tobytes() + I.astype(np.uint32).tobytes()


def prepare_parts(parts, crease, max_tris, px_count):
    """Display geometry and shadow-proxy geometry per part.
    Display: if the scene has more than max_tris source triangles, vertex-cluster at half
    a screen pixel (invisible) to drop sub-pixel triangles. Shadow proxies: always clustered
    at ~1/700 of the scene size (about a shadow-map texel)."""
    uniq = sorted(set(p["stl"] for p in parts))
    nf = {u: face_count(u) for u in uniq}
    bnd = {u: stl_bounds(u) for u in uniq}
    total = sum(nf[p["stl"]] for p in parts)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for p in parts:
        b0, b1 = bnd[p["stl"]]
        C = np.array([[x, y, z] for x in (b0[0], b1[0]) for y in (b0[1], b1[1]) for z in (b0[2], b1[2])])
        M = p.get("matrix")
        if M is not None:
            M = np.asarray(flat_matrix(M)).reshape(4, 4); C = C @ M[:3, :3].T + M[:3, 3]
        lo = np.minimum(lo, C.min(0)); hi = np.maximum(hi, C.max(0))
    diag = float(np.linalg.norm(hi - lo)) or 1.0
    # matrices are assumed rigid (or near): a cell in part-local mm equals a cell in world mm
    cell_disp = (diag / px_count * 0.3) if total > max_tris else 0.0
    cell_shad = diag / 700.0
    jobs = sorted(set((u, crease, c) for u in uniq for c in (cell_disp, cell_shad)))
    if len(jobs) > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=min(len(jobs), os.cpu_count() or 2)) as ex:
            res = dict(zip(jobs, ex.map(_prep_one, jobs)))
    else:
        res = {jobs[0]: _prep_one(jobs[0])}
    disp, shad, nd, ns = [], [], 0, 0
    for p in parts:
        _, P, Nn, I = res[(p["stl"], crease, cell_disp)]
        disp.append(_blob(P, Nn, I, p.get("matrix"))); nd += len(I) // 3
        _, P, Nn, I = res[(p["stl"], crease, cell_shad)]
        shad.append(_blob(P, Nn, I, p.get("matrix"))); ns += len(I) // 3
    return disp, shad, dict(total=total, disp=nd, shad=ns, cell_disp=cell_disp, cell_shad=cell_shad)


PAGE = r"""<!DOCTYPE html><html><head><meta charset="utf-8">
<style>html,body{margin:0;background:transparent;overflow:hidden}</style>
<script type="importmap">{"imports":{"three":"/three/build/three.module.js","three/addons/":"/three/examples/jsm/"}}</script>
</head><body>
<script type="module">
import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { LightProbeGenerator } from 'three/addons/lights/LightProbeGenerator.js';

window.__log = [];
const log = (s) => window.__log.push(s);

window.setup = async function (cfg) {
  const t0 = performance.now();
  const W = cfg.W, H = cfg.H;
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(1);
  renderer.setSize(W, H);
  renderer.setClearColor(0x000000, 0);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = cfg.exposure;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = !cfg.dbgNoShadow;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  document.body.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const pmrem = new THREE.PMREMGenerator(renderer);
  // Diffuse studio light from a RoomEnvironment baked into an SH light probe (cheap per pixel).
  // Full PMREM reflections only on metallic parts (or on all parts if cfg.reflections).
  const room = new RoomEnvironment();
  const envTex = pmrem.fromScene(room, 0.04).texture;
  const cubeRT = new THREE.WebGLCubeRenderTarget(64, { type: THREE.HalfFloatType });
  new THREE.CubeCamera(0.1, 100, cubeRT).update(renderer, room);
  const probe = await LightProbeGenerator.fromCubeRenderTarget(renderer, cubeRT);
  probe.intensity = cfg.probe;
  scene.add(probe);

  const up = new THREE.Vector3(...cfg.up).normalize();
  const meshes = [];
  const shadowProxyMat = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, shadowSide: THREE.BackSide });
  const bbox = new THREE.Box3();
  let ntri = 0;
  for (let i = 0; i < cfg.parts.length; i++) {
    const p = cfg.parts[i];
    const parse = (buf) => {
      if (buf.byteLength < 8) return null;
      const [nv, ni] = new Uint32Array(buf, 0, 2);
      if (nv === 0) return null;
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buf, 8, nv * 3), 3));
      g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(buf, 8 + nv * 12, nv * 3), 3));
      g.setIndex(new THREE.BufferAttribute(new Uint32Array(buf, 8 + nv * 24, ni), 1));
      g.computeBoundingBox(); g.computeBoundingSphere();
      return g;
    };
    const [b1, b2] = await Promise.all([fetch('/geo/' + i).then(r => r.arrayBuffer()), fetch('/shd/' + i).then(r => r.arrayBuffer())]);
    const g = parse(b1);
    if (!g) continue;
    const gs = parse(b2) || g;
    // shadow proxy: only on layer 1, which only the shadow cameras see
    const proxy = new THREE.Mesh(gs, shadowProxyMat);
    proxy.layers.set(1); proxy.castShadow = true;
    scene.add(proxy);
    ntri += g.index.count / 3;
    const c = new THREE.Color().setRGB(p.color[0] / 255, p.color[1] / 255, p.color[2] / 255, THREE.SRGBColorSpace);
    const mat = new THREE.MeshStandardMaterial({
      color: c, metalness: p.metal, roughness: p.rough,
      side: THREE.DoubleSide, shadowSide: THREE.BackSide,
    });
    const lum = 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b;   // dark parts need env specular to show form
    if (cfg.reflections || p.metal >= 0.3 || lum < 0.12) { mat.envMap = envTex; mat.envMapIntensity = cfg.envIntensity; }
    const mesh = new THREE.Mesh(g, mat);
    mesh.castShadow = false; mesh.receiveShadow = true;
    scene.add(mesh); meshes.push(mesh);
    bbox.union(g.boundingBox);
  }
  const center = bbox.getCenter(new THREE.Vector3());
  const radius = bbox.getSize(new THREE.Vector3()).length() / 2;
  log(`loaded ${cfg.parts.length} parts, ${ntri} tris in ${(performance.now() - t0).toFixed(0)} ms`);

  // sample vertices for tight framing (cap ~1.5M samples)
  const sample = [];
  const totalV = meshes.reduce((a, m) => a + m.geometry.attributes.position.count, 0);
  const stride = Math.max(1, Math.floor(totalV / 1.5e6));
  for (const m of meshes) {
    const a = m.geometry.attributes.position.array;
    for (let k = 0; k < a.length; k += 3 * stride) sample.push(a[k], a[k + 1], a[k + 2]);
  }
  const S = new Float32Array(sample);

  // lights
  const hemi = new THREE.HemisphereLight(0xffffff, 0xb8bcc4, cfg.hemi);
  hemi.position.copy(up);
  scene.add(hemi);
  const key = new THREE.DirectionalLight(0xfff6ec, cfg.key);
  key.castShadow = true;
  key.shadow.mapSize.set(cfg.shadowRes, cfg.shadowRes);
  key.shadow.radius = 3;
  key.shadow.bias = -0.001; key.shadow.normalBias = radius * cfg.normalBias;
  // sky light: random upper-hemisphere direction per frame -> ambient occlusion by accumulation
  const sky = new THREE.DirectionalLight(0xffffff, cfg.sky);
  sky.castShadow = cfg.sky > 0;
  sky.shadow.mapSize.set(cfg.shadowRes / 2, cfg.shadowRes / 2);
  sky.shadow.radius = 4; sky.shadow.bias = -0.002; sky.shadow.normalBias = radius * cfg.normalBias * 2;
  sky.shadow.camera.layers.set(1);
  scene.add(sky); scene.add(sky.target);
  key.shadow.camera.layers.set(1);
  scene.add(key); scene.add(key.target);
  const fill = new THREE.DirectionalLight(0xe8f0ff, cfg.fill); scene.add(fill); scene.add(fill.target);
  const rim = new THREE.DirectionalLight(0xffffff, cfg.rim); scene.add(rim); scene.add(rim.target);

  // contact shadow on a ground plane under the model (along -up): the model is rendered
  // once from below with a depth material, blurred at two scales, and used as an alpha map.
  let minUp = Infinity, maxUp = -Infinity;
  for (let k = 0; k < S.length; k += 3) {
    const d = S[k] * up.x + S[k + 1] * up.y + S[k + 2] * up.z;
    if (d < minUp) minUp = d; if (d > maxUp) maxUp = d;
  }
  const g1 = new THREE.Vector3().crossVectors(up, Math.abs(up.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0)).normalize();
  const g2 = new THREE.Vector3().crossVectors(up, g1).normalize();
  let a0 = Infinity, a1 = -Infinity, b0 = Infinity, b1 = -Infinity;
  for (let k = 0; k < S.length; k += 3) {
    const a = S[k] * g2.x + S[k + 1] * g2.y + S[k + 2] * g2.z, b = S[k] * g1.x + S[k + 1] * g1.y + S[k + 2] * g1.z;
    if (a < a0) a0 = a; if (a > a1) a1 = a; if (b < b0) b0 = b; if (b > b1) b1 = b;
  }
  const ext = Math.max(a1 - a0, b1 - b0, 1e-6), height = Math.max(maxUp - minUp, 1e-6);
  const SZ = ext * 1.6;
  const gpos = new THREE.Vector3().addScaledVector(g2, (a0 + a1) / 2).addScaledVector(g1, (b0 + b1) / 2)
    .addScaledVector(up, minUp - radius * 0.003);
  const RES = 512;
  const scam = new THREE.OrthographicCamera(-SZ / 2, SZ / 2, SZ / 2, -SZ / 2, 0, Math.max(height * cfg.contactHeight, ext * 0.08));
  scam.position.copy(gpos); scam.up.copy(g1); scam.lookAt(gpos.clone().add(up)); scam.updateProjectionMatrix();
  const rt = new THREE.WebGLRenderTarget(RES, RES);
  scene.overrideMaterial = new THREE.MeshDepthMaterial({ side: THREE.DoubleSide });
  const savedTM = renderer.toneMapping; renderer.toneMapping = THREE.NoToneMapping;
  renderer.setRenderTarget(rt); renderer.setClearColor(0x000000, 1); renderer.clear(); renderer.render(scene, scam);
  const px = new Uint8Array(RES * RES * 4); renderer.readRenderTargetPixels(rt, 0, 0, RES, RES, px);
  renderer.setRenderTarget(null); renderer.setClearColor(0x000000, 0); renderer.toneMapping = savedTM;
  scene.overrideMaterial = null; rt.dispose();
  const raw = document.createElement('canvas'); raw.width = raw.height = RES;
  const id = raw.getContext('2d').createImageData(RES, RES);
  for (let k = 0; k < RES * RES; k++) { id.data[4 * k + 3] = 255 * Math.pow(px[4 * k] / 255, cfg.contactGamma); }
  raw.getContext('2d').putImageData(id, 0, 0);
  const cs = document.createElement('canvas'); cs.width = cs.height = RES;
  const cctx = cs.getContext('2d');
  cctx.filter = `blur(${RES * 0.008}px)`; cctx.globalAlpha = cfg.contactDark; cctx.drawImage(raw, 0, 0);
  cctx.filter = `blur(${RES * 0.04}px)`; cctx.globalAlpha = cfg.softDark; cctx.drawImage(raw, 0, 0);
  const stex = new THREE.CanvasTexture(cs); stex.flipY = false;
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(SZ, SZ),
    new THREE.MeshBasicMaterial({ color: 0x000000, alphaMap: null, map: stex, transparent: true, depthWrite: false, toneMapped: false, side: THREE.DoubleSide }));
  ground.matrixAutoUpdate = false;
  ground.matrix.makeBasis(g2, g1, up.clone().negate()).setPosition(gpos);
  ground.renderOrder = -1;
  scene.add(ground);

  // ---- outline pass: view normals + view depth at 2x into a float target, edges found on the CPU
  const gbufMat = new THREE.ShaderMaterial({
    side: THREE.DoubleSide,
    vertexShader: `varying vec3 vN; varying float vZ;
      void main(){ vec4 mv = modelViewMatrix * vec4(position,1.0); vZ = -mv.z;
        vN = normalize(normalMatrix * normal); gl_Position = projectionMatrix * mv; }`,
    fragmentShader: `varying vec3 vN; varying float vZ;
      void main(){ vec3 n = normalize(vN); if (!gl_FrontFacing) n = -n; gl_FragColor = vec4(n, vZ); }`,
  });
  const drawOutlines = (ctx, pxWorld) => {
    const t0 = performance.now();
    const OW = W * 2, OH = H * 2;
    const grt = new THREE.WebGLRenderTarget(OW, OH, { type: THREE.FloatType, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
    const gv = ground.visible; ground.visible = false;
    const sm = renderer.shadowMap.autoUpdate; renderer.shadowMap.autoUpdate = false;
    scene.overrideMaterial = gbufMat;
    renderer.setRenderTarget(grt); renderer.setClearColor(0x000000, 0); renderer.clear(); renderer.render(scene, cam);
    const G = new Float32Array(OW * OH * 4); renderer.readRenderTargetPixels(grt, 0, 0, OW, OH, G);
    renderer.setRenderTarget(null); scene.overrideMaterial = null; ground.visible = gv; renderer.shadowMap.autoUpdate = sm;
    grt.dispose();
    const px = pxWorld / 2;                     // world size of one 2x pixel
    const zThr = px * cfg.outlineDepth;          // second-difference threshold (planes give 0)
    const nThr = 1 - Math.cos(cfg.outlineCrease * Math.PI / 180);
    const ec = document.createElement('canvas'); ec.width = OW; ec.height = OH;
    const ectx = ec.getContext('2d'); const img = ectx.createImageData(OW, OH); const D = img.data;
    const BIG = 1e9;
    const zAt = (i) => G[4 * i + 3] > 0 ? G[4 * i + 3] : BIG;
    for (let y = 1; y < OH - 1; y++) {
      for (let x = 1; x < OW - 1; x++) {
        const i = y * OW + x;
        const z = zAt(i), zl = zAt(i - 1), zr = zAt(i + 1), zd = zAt(i - OW), zu = zAt(i + OW);
        let e = 0;
        if (z === BIG) {
          if (zl !== BIG || zr !== BIG || zd !== BIG || zu !== BIG) e = cfg.outline;   // silhouette (outside)
        } else {
          if (zl === BIG || zr === BIG || zd === BIG || zu === BIG) e = cfg.outline;
          else {
            const lx = Math.abs(zl + zr - 2 * z), ly = Math.abs(zd + zu - 2 * z);
            if (Math.max(lx, ly) > zThr) e = cfg.outline;
            else {
              const nx = G[4 * i], ny = G[4 * i + 1], nz = G[4 * i + 2];
              let dmin = 1;
              for (const j of [i - 1, i + 1, i - OW, i + OW]) {
                const d = nx * G[4 * j] + ny * G[4 * j + 1] + nz * G[4 * j + 2]; if (d < dmin) dmin = d;
              }
              if (1 - dmin > nThr) e = cfg.outline * cfg.outlineCreaseStrength;
            }
          }
        }
        if (e > 0) { const o = 4 * ((OH - 1 - y) * OW + x); D[o] = cfg.outlineColor[0]; D[o + 1] = cfg.outlineColor[1]; D[o + 2] = cfg.outlineColor[2]; D[o + 3] = 255 * e; }
      }
    }
    ectx.putImageData(img, 0, 0);
    ctx.globalAlpha = 1; ctx.imageSmoothingEnabled = true; ctx.imageSmoothingQuality = 'high';
    ctx.drawImage(ec, 0, 0, W, H);
    log(`  outlines ${(performance.now() - t0).toFixed(0)} ms`);
  };
  const prepass = new THREE.MeshBasicMaterial({ colorWrite: false, side: THREE.DoubleSide });
  const cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, radius * 40);
  const aspect = W / H;

  window.renderView = function (dirArr, budgetMs) {
    const t1 = performance.now();
    const dir = new THREE.Vector3(...dirArr).normalize();
    // camera basis: forward = -dir, up projected
    let camUp = up.clone().addScaledVector(dir, -up.dot(dir));
    if (camUp.length() < 1e-3) {   // looking along up: use a perpendicular
      camUp = Math.abs(up.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
      camUp.addScaledVector(dir, -camUp.dot(dir));
    }
    camUp.normalize();
    const right = new THREE.Vector3().crossVectors(camUp, dir).normalize();
    // tight projected extents
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let k = 0; k < S.length; k += 3) {
      const px = S[k] - center.x, py = S[k + 1] - center.y, pz = S[k + 2] - center.z;
      const u = px * right.x + py * right.y + pz * right.z;
      const v = px * camUp.x + py * camUp.y + pz * camUp.z;
      if (u < x0) x0 = u; if (u > x1) x1 = u; if (v < y0) y0 = v; if (v > y1) y1 = v;
    }
    const cu = (x0 + x1) / 2, cv = (y0 + y1) / 2;
    let hw = (x1 - x0) / 2 / cfg.fillFrac, hh = (y1 - y0) / 2 / cfg.fillFrac;
    if (hw / hh > aspect) hh = hw / aspect; else hw = hh * aspect;
    const tgt = center.clone().addScaledVector(right, cu).addScaledVector(camUp, cv);
    cam.left = -hw; cam.right = hw; cam.top = hh; cam.bottom = -hh;
    cam.up.copy(camUp);
    cam.position.copy(tgt).addScaledVector(dir, radius * 10);
    cam.near = radius * 10 - radius * 3; cam.far = radius * 10 + radius * 30;
    cam.lookAt(tgt);
    cam.updateProjectionMatrix();

    // lights follow the camera: key upper-left-front, fill right, rim behind-above
    const basis = (a, b, c) => new THREE.Vector3().addScaledVector(dir, a).addScaledVector(right, b).addScaledVector(up, c).normalize();
    const aim = (L, d) => {
      L.position.copy(center).addScaledVector(d, radius * 4); L.target.position.copy(center); L.target.updateMatrixWorld();
    };
    const fitShadow = (L) => {
      const sc = L.shadow.camera, r = radius * 1.15;
      sc.left = -r; sc.right = r; sc.top = r; sc.bottom = -r; sc.near = radius * 1.5; sc.far = radius * 7;
      sc.updateProjectionMatrix();
    };
    fitShadow(key); fitShadow(sky);
    const kd0 = basis(cfg.keyDir[0], cfg.keyDir[1], cfg.keyDir[2]);
    const kU = new THREE.Vector3().crossVectors(kd0, up).normalize();
    const kV = new THREE.Vector3().crossVectors(kU, kd0).normalize();
    aim(fill, basis(0.8, 0.9, -0.1));
    aim(rim, basis(-1.0, 0.3, 0.8));
    ground.visible = cfg.ground && !cfg.dbgNoGround && dir.dot(up) > 0.02;

    const N = cfg.frames;  // max frames; stop early when the time budget runs out
    const budget = (budgetMs === null || budgetMs === undefined) ? 1e12 : budgetMs;
    const out = document.createElement('canvas'); out.width = W; out.height = H;
    const ctx = out.getContext('2d');
    ctx.clearRect(0, 0, W, H);
    const pxX = (cam.right - cam.left) / W, pxY = (cam.top - cam.bottom) / H;
    const base = { l: cam.left, r: cam.right, t: cam.top, b: cam.bottom };
    // low-discrepancy sequences
    const halton = (i, b) => { let f = 1, r = 0; while (i > 0) { f /= b; r += f * (i % b); i = Math.floor(i / b); } return r; };
    let done = 0, tFrames = 0;
    for (let k = 0; k < N; k++) {
      const est = done > 1 ? tFrames / (done - 1) : (window.__tFrame || 0);
      const minF = Math.max(1, Math.min(cfg.minFrames, window.__tFrame ? Math.floor(budget / window.__tFrame) : cfg.minFrames));
      if (k >= minF && (performance.now() - t1) + est > budget) break;
      const jx = N > 1 ? halton(k + 1, 2) - 0.5 : 0, jy = N > 1 ? halton(k + 1, 3) - 0.5 : 0;
      cam.left = base.l + jx * pxX; cam.right = base.r + jx * pxX;
      cam.top = base.t + jy * pxY; cam.bottom = base.b + jy * pxY;
      cam.updateProjectionMatrix();
      // key: jitter inside a disc (area light, ~cfg.keySoft rad)
      const a = 2 * Math.PI * halton(k + 1, 5), rr = cfg.keySoft * Math.sqrt(halton(k + 1, 7));
      aim(key, kd0.clone().addScaledVector(kU, rr * Math.cos(a)).addScaledVector(kV, rr * Math.sin(a)).normalize());
      // sky: directions inside a cone around up (tilted toward the camera); averaged over frames -> soft AO
      const u1 = (halton(k + 1, 11) + 0.37) % 1, u2 = (halton(k + 1, 13) + 0.61) % 1;
      const ca = 1 - u1 * (1 - Math.cos(cfg.skyCone)), sa = Math.sqrt(1 - ca * ca), sp = 2 * Math.PI * u2;
      const sAxis = up.clone().addScaledVector(dir, 0.45).normalize();
      const e1 = new THREE.Vector3().crossVectors(sAxis, right).normalize();
      const e2 = new THREE.Vector3().crossVectors(sAxis, e1);
      const sd = sAxis.clone().multiplyScalar(ca).addScaledVector(e1, sa * Math.cos(sp)).addScaledVector(e2, sa * Math.sin(sp)).normalize();
      aim(sky, sd);
      const ta = performance.now();
      // depth pre-pass (kills overdraw of the expensive PBR shading), then the lit pass
      renderer.autoClear = false; renderer.clear();
      const gv = ground.visible; ground.visible = false;
      renderer.shadowMap.autoUpdate = false; scene.overrideMaterial = prepass;
      if (cfg.prepass) renderer.render(scene, cam);
      scene.overrideMaterial = null; ground.visible = gv;
      renderer.shadowMap.needsUpdate = true;
      renderer.render(scene, cam);
      const gl = renderer.getContext(); gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, new Uint8Array(4));
      const tb = performance.now();
      ctx.globalAlpha = 1 / (k + 1);
      ctx.drawImage(renderer.domElement, 0, 0);
      ctx.getImageData(0, 0, 1, 1);
      if (k > 0) tFrames += performance.now() - ta;   // frame 0 carries shader compilation
      done++;
      if (cfg.profile) log(`  frame ${k}: render ${(tb - ta).toFixed(0)} ms, composite ${(performance.now() - tb).toFixed(0)} ms`);
    }
    if (done > 1) window.__tFrame = tFrames / (done - 1);
    cam.left = base.l; cam.right = base.r; cam.top = base.t; cam.bottom = base.b;
    cam.updateProjectionMatrix();
    if (cfg.outline > 0) drawOutlines(ctx, pxX);
    const t2r = performance.now();
    const url = out.toDataURL('image/png');
    log(`view: ${done} frames in ${(t2r - t1).toFixed(0)} ms, encode ${(performance.now() - t2r).toFixed(0)} ms`);
    return url;
  };
  return true;
};
window.__ready = true;
</script></body></html>
"""


class Handler(http.server.SimpleHTTPRequestHandler):
    blobs = []
    shadow_blobs = []

    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            data = PAGE.encode()
            return self._send(data, "text/html")
        if path.startswith("/geo/"):
            return self._send(self.blobs[int(path[5:])], "application/octet-stream")
        if path.startswith("/shd/"):
            b = self.shadow_blobs[int(path[5:])]
            return self._send(b if b is not None else b"", "application/octet-stream")
        if path.startswith("/three/"):
            fp = os.path.normpath(os.path.join(THREE_PKG, path[len("/three/"):]))
            if fp.startswith(THREE_PKG) and os.path.isfile(fp):
                with open(fp, "rb") as f:
                    return self._send(f.read(), "text/javascript" if fp.endswith(".js") else "application/octet-stream")
        self.send_error(404)

    def _send(self, data, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def flat_matrix(m):
    if m is None:
        return None
    if len(m) == 4 and hasattr(m[0], "__len__"):
        return [float(v) for row in m for v in row]
    assert len(m) == 16, "matrix must be 4x4 or 16 values (row-major)"
    return [float(v) for v in m]


def gradient_bg(w, h, top, bot):
    col = Image.new("RGB", (1, 256))
    for y in range(256):
        t = y / 255.0
        col.putpixel((0, y), tuple(int(round(top[i] * (1 - t) + bot[i] * t)) for i in range(3)))
    return col.resize((w, h), Image.BICUBIC)


def font(size, bold=False):
    names = (["DejaVuSans-Bold.ttf", "LiberationSans-Bold.ttf"] if bold else ["DejaVuSans.ttf", "LiberationSans-Regular.ttf"])
    for n in names:
        for d in ("/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/truetype/liberation", "/usr/share/fonts/TTF", ""):
            p = os.path.join(d, n) if d else n
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                pass
    return ImageFont.load_default()


def main(scene_path):
    T0 = time.time()
    cfg = json.load(open(scene_path))
    ensure_three()
    W, H = int(cfg.get("width", 1100)), int(cfg.get("height", 1100))
    ss = int(cfg.get("ssaa", 1))
    parts = cfg["parts"]
    for p in parts:
        if not os.path.isfile(p["stl"]):
            raise FileNotFoundError(p["stl"])
    t = time.time()
    Handler.blobs, Handler.shadow_blobs, st = prepare_parts(
        parts, float(cfg.get("crease", 35)), int(cfg.get("max_tris", 400000)), 0.8 * min(W, H) * ss)
    print(f"  geometry prep {time.time() - t:.1f} s  (tris: source {st['total']}, display {st['disp']}, shadow proxy {st['shad']})", flush=True)
    js_cfg = dict(
        W=W * ss, H=H * ss,
        up=cfg.get("up", [0, 0, 1]),
        crease=float(cfg.get("crease", 35)),
        fillFrac=float(cfg.get("fill", 0.8)),
        ground=bool(cfg.get("ground", True)),
        exposure=float(cfg.get("exposure", 0.9)),
        envIntensity=float(cfg.get("env_intensity", 0.6)), probe=float(cfg.get("probe", 0.22)), reflections=bool(cfg.get("reflections", False)), prepass=bool(cfg.get("prepass", False)),
        outline=float(cfg.get("outline", 0.55)), outlineCreaseStrength=float(cfg.get("outline_crease", 0.6)),
        outlineDepth=float(cfg.get("outline_depth_px", 3.0)), outlineCrease=float(cfg.get("outline_crease_deg", 40)),
        outlineColor=list(cfg.get("outline_color", [40, 44, 52])),
        hemi=float(cfg.get("hemi", 0.35)), key=float(cfg.get("key", 2.6)),
        fill=float(cfg.get("fill_light", 0.3)), rim=float(cfg.get("rim", 1.0)),
        sky=float(cfg.get("sky", 0.9)), keySoft=float(cfg.get("key_soft", 0.05)),
        skyCone=float(cfg.get("sky_cone", 50)) * np.pi / 180, keyDir=list(cfg.get("key_dir", [0.7, -0.9, 0.9])),
        frames=int(cfg.get("frames", 24)), minFrames=int(cfg.get("min_frames", 4)), profile=bool(cfg.get("profile", False)),
        dbgNoShadow=bool(cfg.get("dbg_noshadow", False)), dbgNoGround=bool(cfg.get("dbg_noground", False)),
        contactDark=float(cfg.get("contact_dark", 0.6)), softDark=float(cfg.get("soft_dark", 0.45)),
        contactHeight=float(cfg.get("contact_height", 0.3)), contactGamma=float(cfg.get("contact_gamma", 2.5)),
        shadowRes=int(cfg.get("shadow_res", 2048)), normalBias=float(cfg.get("normal_bias", 0.012)),
        parts=[dict(color=list(p.get("color", [200, 200, 205])), metal=float(p.get("metal", 0.0)),
                    rough=float(p.get("rough", 0.5))) for p in parts],
    )
    srv = socketserver.ThreadingTCPServer(("127.0.0.1", 0), Handler)
    srv.daemon_threads = True
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    bg_top, bg_bot = cfg.get("background", [[247, 248, 250], [205, 210, 218]])
    out_prefix = cfg["out_prefix"]
    os.makedirs(os.path.dirname(out_prefix) or ".", exist_ok=True)
    views = cfg.get("views") or [{"name": "iso", "dir": [1, -1, 0.8]}]
    outs = []
    from playwright.sync_api import sync_playwright
    with sync_playwright() as pw:
        browser = pw.chromium.launch(args=["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"])
        page = browser.new_page(viewport={"width": W * ss, "height": H * ss})
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
        page.goto(f"http://127.0.0.1:{port}/")
        page.wait_for_function("window.__ready === true", timeout=30000)
        try:
            page.evaluate("cfg => window.setup(cfg)", js_cfg)
        except Exception:
            print("\n".join(errors), file=sys.stderr)
            raise
        budget = float(cfg.get("time_budget", 45.0)) - (time.time() - T0)
        t_views = time.time()
        for iv, v in enumerate(views):
            left = budget - (time.time() - t_views)
            per_view = max(0.0, left / (len(views) - iv)) * 1000.0
            url = page.evaluate("a => window.renderView(a[0], a[1])", [list(v["dir"]), per_view])
            img = Image.open(io.BytesIO(base64.b64decode(url.split(",", 1)[1]))).convert("RGBA")
            if ss > 1:
                img = img.resize((W, H), Image.LANCZOS)
            bg = gradient_bg(W, H, bg_top, bg_bot).convert("RGBA")
            bg.alpha_composite(img)
            fn = f"{out_prefix}_{v['name']}.png"
            bg.convert("RGB").save(fn, optimize=False)
            outs.append((v.get("label", v["name"]), bg.convert("RGB")))
            print(fn, flush=True)
        for l in page.evaluate("window.__log"):
            print("  [three]", l)
        if errors:
            print("  [browser errors]", *errors[:5], sep="\n    ")
        browser.close()
    srv.shutdown()

    # contact sheet
    n = len(outs)
    max_w = int(cfg.get("sheet_max_width", 4400))
    pw_ = min(W, (max_w - 40 * (n + 1)) // n)
    ph = int(round(H * pw_ / W))
    pad, title_h, label_h = 40, 110, 64
    sheet = Image.new("RGB", (pad + n * (pw_ + pad), title_h + ph + label_h + pad), (255, 255, 255))
    d = ImageDraw.Draw(sheet)
    d.text((pad, 34), cfg.get("title", ""), fill=(35, 38, 45), font=font(46, bold=True))
    lf = font(32)
    for i, (label, im) in enumerate(outs):
        x = pad + i * (pw_ + pad)
        tile = im.resize((pw_, ph), Image.LANCZOS) if pw_ != W else im
        sheet.paste(tile, (x, title_h))
        d.rectangle([x - 1, title_h - 1, x + pw_, title_h + ph], outline=(214, 218, 224), width=1)
        tw = d.textlength(label, font=lf)
        d.text((x + (pw_ - tw) / 2, title_h + ph + 14), label, fill=(70, 74, 82), font=lf)
    fn = f"{out_prefix}_sheet.png"
    sheet.save(fn)
    print(fn)
    print(f"total {time.time() - T0:.1f} s")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
