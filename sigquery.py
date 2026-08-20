#!/usr/bin/env python3
"""Read back what sigmon stored -- from MongoDB, or from the JSONL fallback.

    ./sigquery.py signals                  # one row per emitter
    ./sigquery.py signals --min-obs 5
    ./sigquery.py observations 96.0M       # every observation of one signal
    ./sigquery.py runs

Reads the same URI/db as sigmon.  If Mongo is unreachable it falls back to the
JSONL file, so a run captured while the database was down is still queryable
with the same command.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dfcore                       # noqa: E402


def load_jsonl(path, collection):
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if d.get("_collection") == collection:
                out.append(d)
    return out


def get_source(a):
    """Return (kind, handle). Mongo if reachable, else the JSONL file."""
    try:
        import pymongo
        c = pymongo.MongoClient(a.mongo_uri, serverSelectionTimeoutMS=2000)
        c.admin.command("ping")
        return "mongo", c[a.mongo_db]
    except Exception as e:                                  # noqa: BLE001
        print(f"# MongoDB unavailable ({type(e).__name__}); "
              f"reading {a.fallback}", file=sys.stderr)
        return "jsonl", a.fallback


def _signals_from_obs(obs):
    """Per-emitter statistics recomputed from raw observations."""
    by = {}
    for o in obs:
        if o.get("bearing_deg") is None:
            continue
        by.setdefault(o["freq_hz"], []).append(o)
    rows = []
    for f, lst in by.items():
        b = [o["bearing_deg"] for o in lst]
        rows.append(dict(freq_hz=f, n_observations=len(b),
                         bearing_mean_deg=dfcore.circ_mean_deg(b),
                         bearing_std_deg=dfcore.circ_std_deg(b),
                         last_level_dbfs=lst[-1]["level_dbfs"],
                         last_snr_db=lst[-1]["snr_db"],
                         calibrated=lst[-1].get("calibrated", False)))
    return rows


def latest_run_id(kind, h):
    if kind == "mongo":
        d = h.runs.find_one({}, {"run_id": 1, "_id": 0}, sort=[("ts", -1)])
        return d.get("run_id") if d else None
    runs = load_jsonl(h, "runs")
    return runs[-1].get("run_id") if runs else None


def cmd_signals(a, kind, h):
    # Default to the most recent run only.  Pooling observations across runs
    # silently mixes different beamwidths, balance settings and gains, and the
    # resulting circular std then describes the configuration changes rather
    # than the emitter -- every signal reads "unstable" for the wrong reason.
    run = None
    if not a.all_runs:
        run = a.run or latest_run_id(kind, h)
        if run:
            print(f"# run {run[:8]} only (--all-runs to pool, but note that "
                  f"pools different settings)", file=sys.stderr)

    if kind == "mongo":
        rows = list(h.signals.find({} if run is None else {"run_id": run},
                                   {"_id": 0}))
        if run is not None and not rows:
            # signals docs are upserted per emitter and carry only the latest
            # run_id, so fall back to recomputing from this run's observations.
            rows = _signals_from_obs(list(h.observations.find(
                {"run_id": run}, {"_id": 0})))
    else:
        # The fallback appends one record per update, so recompute the
        # statistics from the observations rather than trusting whichever
        # signal row happened to be written last.
        obs = load_jsonl(h, "observations")
        if run is not None:
            obs = [o for o in obs if o.get("run_id") == run]
        rows = _signals_from_obs(obs)

    rows = [r for r in rows if r.get("n_observations", 0) >= a.min_obs]
    rows.sort(key=lambda r: -r.get("last_level_dbfs", -999))
    if not rows:
        print("no signals stored yet")
        return
    cal = any(r.get("calibrated") for r in rows)
    print(f"{'MHz':>11} {'n':>5} {'bearing':>9} {'circ std':>9} "
          f"{'dBFS':>8} {'SNR':>7}  verdict")
    for r in rows:
        sd = r.get("bearing_std_deg", float("nan"))
        verdict = ("stable" if sd < 5 else "usable" if sd < 15 else "unstable")
        print(f"{r['freq_hz']/1e6:>11.3f} {r.get('n_observations',0):>5} "
              f"{r.get('bearing_mean_deg',float('nan')):>8.1f}d {sd:>8.1f}d "
              f"{r.get('last_level_dbfs',float('nan')):>8.1f} "
              f"{r.get('last_snr_db',float('nan')):>6.1f}d  {verdict}")
    if not cal:
        print("\nbearings are RELATIVE (no calibration was in use when these")
        print("were recorded) -- comparable to each other, not to true north")


def cmd_observations(a, kind, h):
    tol = a.tolerance
    if kind == "mongo":
        q = {} if a.freq is None else {
            "freq_hz": {"$gte": a.freq - tol, "$lte": a.freq + tol}}
        rows = list(h.observations.find(q, {"_id": 0}).sort("ts", 1))
    else:
        rows = load_jsonl(h, "observations")
        if a.freq is not None:
            rows = [r for r in rows if abs(r["freq_hz"] - a.freq) <= tol]
    if not rows:
        print("no observations match")
        return
    print(f"{'ts':>26} {'MHz':>10} {'dBFS':>8} {'bearing':>9} {'conf':>6}")
    for r in rows[-a.limit:]:
        b = r.get("bearing_deg")
        bs = f"{b:>8.1f}d" if b is not None else f"{'-':>9}"
        print(f"{str(r.get('ts'))[:26]:>26} {r['freq_hz']/1e6:>10.3f} "
              f"{r['level_dbfs']:>8.1f} {bs} {r.get('confidence',0):>6.2f}")
    bl = [r["bearing_deg"] for r in rows if r.get("bearing_deg") is not None]
    if len(bl) >= 3:
        print(f"\n{len(bl)} bearings: mean {dfcore.circ_mean_deg(bl):.1f} deg, "
              f"circular std {dfcore.circ_std_deg(bl):.1f} deg")


def cmd_runs(a, kind, h):
    rows = (list(h.runs.find({}, {"_id": 0}).sort("ts", -1))
            if kind == "mongo" else load_jsonl(h, "runs"))
    if not rows:
        print("no runs stored")
        return
    for r in rows[-a.limit:]:
        print(f"{str(r.get('ts'))[:19]}  {r.get('run_id','?')[:8]}  "
              f"{r.get('start_hz',0)/1e6:.1f}-{r.get('stop_hz',0)/1e6:.1f} MHz  "
              f"ports {r.get('channels')}  "
              f"{'calibrated' if r.get('calibrated') else 'uncalibrated'}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("what", choices=["signals", "observations", "runs"])
    p.add_argument("freq", nargs="?", default=None,
                   help="frequency filter for observations, e.g. 96.0M")
    p.add_argument("--tolerance", type=float, default=100e3)
    p.add_argument("--min-obs", type=int, default=1)
    p.add_argument("--limit", type=int, default=200)
    p.add_argument("--run", default=None, help="restrict to one run_id")
    p.add_argument("--all-runs", action="store_true",
                   help="pool every run (mixes different settings -- the "
                        "resulting spread describes the config changes, "
                        "not the emitter)")
    p.add_argument("--mongo-uri", default=os.environ.get(
        "SIGMON_MONGO_URI",
        "mongodb://sigmon:sigmon@127.0.0.1:27017/?authSource=admin"))
    p.add_argument("--mongo-db", default=os.environ.get("SIGMON_MONGO_DB", "signals"))
    p.add_argument("--fallback", default="sigmon_fallback.jsonl")
    a = p.parse_args()

    if a.freq:
        s = a.freq.strip().upper()
        mult = {"K": 1e3, "M": 1e6, "G": 1e9}.get(s[-1:], 1.0)
        a.freq = float(s[:-1] if mult != 1.0 else s) * mult
    else:
        a.freq = None

    kind, h = get_source(a)
    {"signals": cmd_signals, "observations": cmd_observations,
     "runs": cmd_runs}[a.what](a, kind, h)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
