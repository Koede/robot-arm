"""gates.py - one place where every verify_*/sweep_* script records its gate results.

Usage in a verify script (after the number is computed, next to the print):

    import gates                                   # cad/ is already on sys.path in every verify script
    gates.begin("verify_claw.py")                  # once, near the top: drops this script's old records
    gates.record("claw.grip_SF", SF, 2.0, "-", source_script="verify_claw.py")          # op defaults to ">="
    gates.record("claw.min_gap.finger_fixed", d, 0.2, "mm", op=">=", note="designed layer gap")
    gates.record("stops.S1-@S2=25.hard_angle", a, hard, "deg", op="approx", tol=1.0)
    gates.record("render.claw_c3", "G100/50/0 checked by eye 2026-10-14h", passed=True)   # manual / text gates

Records go to  stl/gates.json  (override with env TS_GATES_PATH), written atomically under a file lock, so scripts that
run in parallel or die half-way cannot corrupt it.  Nothing here imports numpy; numpy scalars/arrays are converted.

SCHEMA (gates.json, schema 1)
{
  "schema": 1,
  "gates": {
    "<dotted.name>": {
      "name": "<dotted.name>",                 # first dotted component is the group used by gen_state_table.py (torque, claw, shoulder, ...)
      "value": 1.27,                           # number, string or small JSON-able structure
      "limit": 2.0,                            # number or null (null + passed null => "info")
      "op": ">=",                              # one of >= <= > < == approx absle   (see OPS)
      "tol": null,                             # only for op "approx"
      "unit": "-",
      "passed": false,                         # true / false / null (info only)
      "status": "fail",                        # pass | fail | info   (derived; "open" can be forced via status=)
      "source_script": "verify_distal.py",
      "stage": "verify_distal",                # TS_STAGE set by run_all.py (the stage NAME; one script can back several stages, e.g. body_loads / body_loads_mu0.2)
      "stage_key": "ab12..",                   # TS_STAGE_KEY set by run_all.py: lets the table flag gates whose inputs changed since
      "run_id": "20261014T101500",             # TS_RUN_ID set by run_all.py
      "timestamp": "2026-10-14T10:15:03",
      "note": "free text",
      "previous": {"value": 1.4, "passed": false, "timestamp": ".."}   # last record before this one: regression detection
    }
  }
}
Status words: pass / fail / info (no limit) / open (known-open item, forced by status="open", counts as not passing) /
flag (passes or is info, but carries a reviewer flag shown as **FLAG** in the table, e.g. the F2 15.7 kPa single cell above the 14.7 sustained mean).

CLI (manual gates: renders checked by eye, host-test counts, numbers quoted from a spec):
    python3 cad/gates.py --show
    python3 cad/gates.py record NAME VALUE [--limit L] [--op >=] [--unit U] [--note TEXT] [--source SCRIPT] [--status pass|fail|info|open|flag] [--tol T]
VALUE / --limit are parsed as numbers when they look like numbers, else kept as text.
"""
import json, os, sys, time, tempfile, math

try:
    import fcntl
except ImportError:      # non-posix: no locking, still atomic via os.replace
    fcntl = None

SCHEMA = 1
HERE = os.path.dirname(os.path.abspath(__file__))


def path():
    return os.environ.get("TS_GATES_PATH") or os.path.join(HERE, "..", "stl", "gates.json")


OPS = {
    ">=": lambda v, l, t: v >= l,
    "<=": lambda v, l, t: v <= l,
    ">": lambda v, l, t: v > l,
    "<": lambda v, l, t: v < l,
    "==": lambda v, l, t: v == l,
    "approx": lambda v, l, t: abs(v - l) <= (t if t is not None else 1e-6),   # |value - limit| <= tol
    "absle": lambda v, l, t: abs(v) <= l,                                      # |value| <= limit
}


def _plain(x):
    """numpy / tuple / Path -> JSON-able plain python."""
    if hasattr(x, "tolist"):
        x = x.tolist()
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return None if math.isnan(x) else ("inf" if x > 0 else "-inf")
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, set)):
        return [_plain(v) for v in x]
    if isinstance(x, (int, float, str, bool)) or x is None:
        return x
    return str(x)


def _load(p):
    try:
        with open(p) as f:
            d = json.load(f)
        if d.get("schema") == SCHEMA and isinstance(d.get("gates"), dict):
            return d
    except (OSError, ValueError):
        pass
    return {"schema": SCHEMA, "gates": {}}


class _Lock:
    def __init__(self, p):
        self.lp = p + ".lock"; self.f = None

    def __enter__(self):
        os.makedirs(os.path.dirname(self.lp), exist_ok=True)
        if fcntl:
            self.f = open(self.lp, "w"); fcntl.flock(self.f, fcntl.LOCK_EX)
        return self

    def __exit__(self, *a):
        if self.f:
            fcntl.flock(self.f, fcntl.LOCK_UN); self.f.close()


def _write(p, d):
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(p), prefix=".gates.", suffix=".tmp")
    with os.fdopen(fd, "w") as f:
        json.dump(d, f, indent=1, sort_keys=True)
    os.replace(tmp, p)


def _src(source_script):
    if source_script:
        return os.path.basename(source_script)
    main = sys.modules.get("__main__")
    return os.path.basename(getattr(main, "__file__", "") or "interactive")


def begin(source_script=None):
    """Remove every record written by this script earlier, so gates that no longer exist do not linger. Call once at the top."""
    s = _src(source_script); p = path()
    with _Lock(p):
        d = _load(p)
        keep = {k: v for k, v in d["gates"].items() if v.get("source_script") != s}
        # remember the old ones so record() can fill "previous"
        for k, v in d["gates"].items():
            if v.get("source_script") == s:
                keep.setdefault("__prev__:" + k, {"previous_only": True, "value": v.get("value"), "passed": v.get("passed"), "timestamp": v.get("timestamp")})
        d["gates"] = keep; _write(p, d)


def record(name, value, limit=None, unit="", passed=None, source_script=None, *, op=">=", tol=None, note="", status=None):
    """Record one gate. Returns the pass flag (True / False / None for info) so the caller can still print or exit on it.
    passed=None and a numeric limit  -> computed from op;  passed given -> used as is;  no limit and no passed -> info."""
    value, limit, tol = _plain(value), _plain(limit), _plain(tol)
    if passed is None and limit is not None and isinstance(value, (int, float)) and not isinstance(value, bool) and isinstance(limit, (int, float)):
        if op not in OPS:
            raise ValueError("unknown op %r" % op)
        passed = bool(OPS[op](value, limit, tol))
    if status is None:
        status = "info" if passed is None else ("pass" if passed else "fail")
    elif status == "open":
        passed = False
    elif status == "flag" and passed is False:
        status = "fail"                         # a flag never hides a failure
    now = time.strftime("%Y-%m-%dT%H:%M:%S")
    rec = dict(name=name, value=value, limit=limit, op=op, tol=tol, unit=unit, passed=passed, status=status, source_script=_src(source_script),
               stage=os.environ.get("TS_STAGE"), stage_key=os.environ.get("TS_STAGE_KEY"), run_id=os.environ.get("TS_RUN_ID"), timestamp=now, note=note)
    p = path()
    with _Lock(p):
        d = _load(p)
        old = d["gates"].get(name) or d["gates"].pop("__prev__:" + name, None)
        if old:
            rec["previous"] = {k: old.get(k) for k in ("value", "passed", "timestamp")}
        d["gates"].pop("__prev__:" + name, None)
        d["gates"][name] = rec
        _write(p, d)
    return passed


def record_many(prefix, mapping, limit=None, unit="", source_script=None, **kw):
    """record(prefix + '.' + k, v, ...) for a dict of values sharing one limit; returns True if all passed."""
    r = [record("%s.%s" % (prefix, k), v, limit, unit, source_script=source_script, **kw) for k, v in mapping.items()]
    return all(x is not False for x in r)


def load():
    return {k: v for k, v in _load(path())["gates"].items() if not k.startswith("__prev__:")}


def _num(x):
    if x is None:
        return None
    try:
        return int(x) if x.lstrip("+-").isdigit() else float(x)
    except ValueError:
        return x


def _cli(argv):
    import argparse
    ap = argparse.ArgumentParser(prog="gates.py")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("show")
    r = sub.add_parser("record")
    r.add_argument("name"); r.add_argument("value")
    r.add_argument("--limit"); r.add_argument("--op", default=">="); r.add_argument("--unit", default=""); r.add_argument("--note", default="")
    r.add_argument("--source", default="manual"); r.add_argument("--status"); r.add_argument("--tol", type=float)
    r.add_argument("--passed", choices=("true", "false"))
    a = ap.parse_args(argv)
    if a.cmd == "record":
        p = None if a.passed is None else (a.passed == "true")
        res = record(a.name, _num(a.value), _num(a.limit), a.unit, passed=p, source_script=a.source, op=a.op, tol=a.tol, note=a.note, status=a.status)
        print("recorded %s = %s (%s)" % (a.name, a.value, load()[a.name]["status"])); return 0 if res is not False else 1
    g = load()
    for k in sorted(g):
        v = g[k]; print("%-5s %-44s %s %s %s (%s)" % (v["status"].upper(), k, v["value"], v["op"], v["limit"], v["source_script"]))
    print(len(g), "gates in", os.path.abspath(path()), "| failing:", [k for k, v in g.items() if v["status"] in ("fail", "open")])
    return 0


if __name__ == "__main__":      # python3 cad/gates.py [--show | show | record ...]
    sys.exit(_cli([a for a in sys.argv[1:] if a != "--show"] or ["show"]))
