import json, sys, os
import numpy as np
from scipy import signal

W = sys.argv[1]
OUT = sys.argv[2]
os.makedirs(OUT, exist_ok=True)
for i in range(1, 8):
    a = np.load(f"{W}/log{i}.npy")
    names = json.load(open(f"{W}/log{i}_names.json"))
    step = 8 if i == 7 else 1
    b = a[::step]
    t = (b[:, names.index("time")] - a[0, names.index("time")]) / 1e6
    hdr = "t_s," + ",".join(names)
    np.savetxt(f"{OUT}/log{i}{'_250Hz' if step > 1 else '_2kHz'}.csv", np.column_stack([t, b]),
               delimiter=",", header=hdr, comments="", fmt="%.6g")
    if i == 4:
        x = np.nan_to_num(a[:, names.index("debug[0]")])
        f, p = signal.welch(x - x.mean(), fs=2004, nperseg=2048)
        m = f > 30
        print("log4 roll unfiltered peak Hz", f[m][np.argmax(p[m])])
for i in range(1, 8):
    h = json.load(open(f"{W}/log{i}_headers.json"))
    with open(f"{OUT}/log{i}_settings.txt", "w") as fh:
        for k, v in h.items():
            fh.write(f"{k} = {v}\n")
print(os.popen(f"du -sh {OUT}/*").read())
