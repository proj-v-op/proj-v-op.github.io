"""Motor balance + steady I-term evidence figure for log 7."""
import json, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

W, OUT = sys.argv[1], sys.argv[2]
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
plt.rcParams.update({"figure.dpi": 130, "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "#e6e5e0", "legend.frameon": False,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})
n = json.load(open(f"{W}/log7_names.json")); a = np.load(f"{W}/log7.npy")
c = lambda k: a[:, n.index(k)]
t = (c("time") - c("time")[0]) / 1e6
m = c("rcCommand[3]") > 1100
mot = [np.nanmean(c(f"motor[{i}]")[m]) for i in range(4)]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw={"width_ratios": [1, 1.6]})
labels = ["M1\nrear right", "M2\nfront right", "M3\nrear left", "M4\nfront left"]
cols = [C[1], C[0], C[1], C[0]]
bars = ax1.bar(labels, mot, color=cols, width=0.6)
for b, v in zip(bars, mot):
    ax1.text(b.get_x() + b.get_width() / 2, v + 10, f"{v:.0f}", ha="center", fontsize=8.5)
ax1.set_ylabel("Average motor output while flying")
ax1.set_ylim(0, max(mot) * 1.15)
ax1.set_title("Rear motors (orange) run ~100 higher than front (blue)")
ax1.grid(axis="x", visible=False)

k = int(2004 * 1.0)
win = np.ones(k) / k
names = ["Roll", "Pitch", "Yaw"]
for j in range(3):
    x = np.nan_to_num(c(f"axisI[{j}]"))
    ax2.plot(t[k - 1:], np.convolve(x, win, "valid"), color=C[j], lw=1.4, label=f"{names[j]} I-term (1 s average)")
ax2.axhline(0, color="#8a8984", lw=0.8)
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("I-term")
ax2.set_title("Pitch I-term sits ~27 above zero the whole flight")
ax2.legend(loc="lower center", ncol=3)
fig.tight_layout()
fig.savefig(f"{OUT}/log7_08_motor_balance.png", bbox_inches="tight", facecolor=fig.get_facecolor())
