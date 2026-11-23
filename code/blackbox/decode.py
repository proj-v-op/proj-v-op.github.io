"""Decode every log in a Betaflight .bbl into CSV + a JSON summary of headers/stats."""
import sys, json, os
sys.path.insert(0, os.environ.get("ORANGEBOX_PATH", ""))
import numpy as np
from orangebox import Parser

path, outdir = sys.argv[1], sys.argv[2]
os.makedirs(outdir, exist_ok=True)
n = Parser.load(path).reader.log_count
summary = []
for i in range(1, n + 1):
    rows, err = [], None
    try:
        p = Parser.load(path, i)
        names = p.field_names
        try:
            for fr in p.frames():
                rows.append(list(fr.data))
        except Exception as e:
            err = repr(e)
        hdr = {k: (v if isinstance(v, (int, float, str, list)) else str(v)) for k, v in p.headers.items()}
    except Exception as e:
        summary.append({"log": i, "error_open": repr(e)})
        continue
    if not rows:
        summary.append({"log": i, "frames": 0, "error": err})
        continue
    w = max(len(r) for r in rows)
    def num(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return np.nan
    rows = [[num(x) for x in r] + [np.nan] * (w - len(r)) for r in rows if len(r) >= len(names) - 5]
    a = np.array(rows, dtype=float)[:, :len(names)]
    np.save(f"{outdir}/log{i}.npy", a)
    with open(f"{outdir}/log{i}_names.json", "w") as f:
        json.dump(names, f)
    with open(f"{outdir}/log{i}_headers.json", "w") as f:
        json.dump(hdr, f, indent=1)
    col = {nm: a[:, k] for k, nm in enumerate(names)}
    t = (col["time"] - col["time"][0]) / 1e6
    summary.append({
        "log": i, "frames": len(a), "dur_s": float(t[-1]), "parse_error": err,
        "vbat_first_V": float(col["vbatLatest"][0] / 100), "vbat_min_V": float(np.nanmin(col["vbatLatest"]) / 100),
        "thr_max": float(np.nanmax(col["rcCommand[3]"])), "thr_mean": float(np.nanmean(col["rcCommand[3]"])),
        "motor_max": float(np.nanmax(col["motor[0]"])),
        "amp_max_A": float(np.nanmax(col["amperageLatest"]) / 100),
    })
    print(summary[-1], flush=True)
with open(f"{outdir}/summary.json", "w") as f:
    json.dump(summary, f, indent=1)
