"""Make graphs + stats from decoded Betaflight logs (npy from export.py)."""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal

W = sys.argv[1]
OUT = sys.argv[2]
os.makedirs(OUT, exist_ok=True)

C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # validated categorical slots 1-4
AX = ["Roll", "Pitch", "Yaw"]
plt.rcParams.update({
    "figure.dpi": 130, "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
    "axes.edgecolor": "#8a8984", "axes.labelcolor": "#0b0b0b", "xtick.color": "#52514e",
    "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
    "lines.linewidth": 1.0, "legend.frameon": False, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
})


def load(i):
    a = np.load(f"{W}/log{i}.npy")
    names = json.load(open(f"{W}/log{i}_names.json"))
    hdr = json.load(open(f"{W}/log{i}_headers.json"))
    d = {n: a[:, k] for k, n in enumerate(names)}
    d["t"] = (d["time"] - d["time"][0]) / 1e6
    return d, names, hdr


def save(fig, name):
    fig.savefig(f"{OUT}/{name}", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def overview(d, i):
    t = d["t"]
    fig, axs = plt.subplots(4, 1, figsize=(11, 8.5), sharex=True)
    axs[0].plot(t, (d["rcCommand[3]"] - 1000) / 10, color=C[0])
    axs[0].set_ylabel("Throttle stick (%)")
    axs[0].set_title(f"Log {i} overview")
    for m in range(4):
        axs[1].plot(t, d[f"motor[{m}]"], color=C[m], lw=0.6, label=f"Motor {m + 1}")
    axs[1].set_ylabel("Motor output\n(DShot 0-2047)")
    axs[1].legend(ncol=4, loc="upper right")
    axs[2].plot(t, d["vbatLatest"] / 100, color=C[0])
    axs[2].set_ylabel("Battery (V)")
    axs[3].plot(t, d["amperageLatest"] / 100, color=C[1])
    axs[3].set_ylabel("Current (A)")
    axs[3].set_xlabel("Time (s)")
    save(fig, f"log{i}_01_overview.png")


def tracking(d, i, t0=None, span=None):
    t = d["t"]
    fig, axs = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True)
    sel = slice(None) if t0 is None else (t >= t0) & (t <= t0 + span)
    for k in range(3):
        axs[k].plot(t[sel], d[f"gyroADC[{k}]"][sel], color=C[0], lw=0.7, label="Gyro (measured)")
        axs[k].plot(t[sel], d[f"setpoint[{k}]"][sel], color=C[1], lw=0.9, label="Setpoint (commanded)")
        axs[k].set_ylabel(f"{AX[k]} rate (deg/s)")
    axs[0].legend(ncol=2, loc="upper right")
    ttl = "full flight" if t0 is None else f"{t0:.0f}-{t0 + span:.0f} s"
    axs[0].set_title(f"Log {i}: setpoint vs gyro, {ttl}")
    axs[2].set_xlabel("Time (s)")
    save(fig, f"log{i}_02_tracking_{'full' if t0 is None else 'zoom'}.png")


def pid_terms(d, i, t0, span):
    t = d["t"]
    sel = (t >= t0) & (t <= t0 + span)
    fig, axs = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True)
    for k in range(3):
        terms = [("P", f"axisP[{k}]"), ("I", f"axisI[{k}]"), ("D", f"axisD[{k}]"), ("F", f"axisF[{k}]")]
        for j, (lab, key) in enumerate(terms):
            if key in d:
                axs[k].plot(t[sel], d[key][sel], color=C[j], lw=0.7, label=lab)
        axs[k].set_ylabel(f"{AX[k]} PID output")
    axs[0].legend(ncol=4, loc="upper right")
    axs[0].set_title(f"Log {i}: P / I / D / Feedforward terms, {t0:.0f}-{t0 + span:.0f} s")
    axs[2].set_xlabel("Time (s)")
    save(fig, f"log{i}_03_pid_terms_zoom.png")


def psd(x, fs):
    x = np.nan_to_num(x - np.nanmean(x))
    f, p = signal.welch(x, fs=fs, nperseg=2048)
    return f, 10 * np.log10(p + 1e-12)


def spectra(d, i, fs):
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.8), sharey=True)
    moving = d["rcCommand[3]"] > 1100
    out = {}
    for k in range(3):
        f, praw = psd(d[f"debug[{k}]"][moving], fs)
        f, pfilt = psd(d[f"gyroADC[{k}]"][moving], fs)
        axs[k].plot(f, praw, color=C[1], label="Unfiltered gyro (debug)")
        axs[k].plot(f, pfilt, color=C[0], label="Filtered gyro")
        axs[k].set_title(AX[k])
        axs[k].set_xlabel("Frequency (Hz)")
        axs[k].set_xlim(0, fs / 2)
        pk = f[np.argmax(praw * (f > 50) + (-999) * (f <= 50))]
        out[AX[k]] = float(pk)
    axs[0].set_ylabel("Power (dB)")
    axs[0].legend(loc="upper right")
    fig.suptitle(f"Log {i}: gyro noise spectrum while flying (throttle > 10%)", y=1.02)
    save(fig, f"log{i}_04_noise_spectrum.png")
    return out


def spectrogram(d, i, fs, key, label, tag):
    thr = (d["rcCommand[3]"] - 1000) / 10
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.8), sharey=True)
    bins = np.arange(0, 101, 5)
    for k in range(3):
        x = np.nan_to_num(d[f"{key}[{k}]"])
        nper = 512
        f, tt, S = signal.spectrogram(x, fs=fs, nperseg=nper, noverlap=nper // 2)
        idx = np.clip((tt * fs).astype(int), 0, len(thr) - 1)
        th = thr[idx]
        M = np.full((len(bins) - 1, len(f)), np.nan)
        for b in range(len(bins) - 1):
            m = (th >= bins[b]) & (th < bins[b + 1])
            if m.sum():
                M[b] = 10 * np.log10(S[:, m].mean(axis=1) + 1e-12)
        im = axs[k].imshow(M.T, origin="lower", aspect="auto", cmap="Blues",
                           extent=[0, 100, 0, fs / 2], vmin=np.nanpercentile(M, 5), vmax=np.nanpercentile(M, 99.5))
        axs[k].set_title(AX[k]); axs[k].set_xlabel("Throttle (%)"); axs[k].grid(False)
    axs[0].set_ylabel("Frequency (Hz)")
    fig.colorbar(im, ax=axs, label="Power (dB)", shrink=0.9)
    fig.suptitle(f"Log {i}: {label} noise vs throttle", y=1.02)
    save(fig, f"log{i}_05_noise_vs_throttle_{tag}.png")


def step_response(d, i, fs):
    """Wiener deconvolution step response (PIDtoolbox / Plasmatree method)."""
    win = int(fs * 1.0)
    L = int(fs * 0.5)
    fig, ax = plt.subplots(figsize=(8, 4))
    res = {}
    for k in range(3):
        sp, gy = np.nan_to_num(d[f"setpoint[{k}]"]), np.nan_to_num(d[f"gyroADC[{k}]"])
        thr = d["rcCommand[3]"]
        resp = []
        for s in range(0, len(sp) - win, win // 2):
            a, b = sp[s:s + win], gy[s:s + win]
            if np.max(np.abs(a)) < 50 or np.mean(thr[s:s + win]) < 1080:
                continue
            h = np.hanning(win)
            A, B = np.fft.rfft(a * h), np.fft.rfft(b * h)
            H = B * np.conj(A) / (A * np.conj(A) + 1e-4 * np.max(np.abs(A)) ** 2)
            imp = np.fft.irfft(H, n=win)[:L]
            st = np.cumsum(imp)
            if 0.5 < st[-L // 4:].mean() < 1.5:
                resp.append(st)
        if not resp:
            continue
        r = np.mean(resp, axis=0)
        tt = np.arange(L) / fs * 1000
        ax.plot(tt, r, color=C[k], lw=1.6, label=f"{AX[k]} ({len(resp)} windows)")
        peak = float(r.max())
        rise = float(tt[np.argmax(r >= 0.5)])
        res[AX[k]] = {"peak": peak, "overshoot_pct": (peak - 1) * 100, "t50_ms": rise, "windows": len(resp)}
    ax.axhline(1, color="#8a8984", lw=0.8, ls="--")
    ax.set_xlim(0, 300); ax.set_xlabel("Time (ms)"); ax.set_ylabel("Normalized response")
    ax.set_title(f"Log {i}: estimated step response")
    ax.legend(loc="lower right")
    save(fig, f"log{i}_06_step_response.png")
    return res


def motor_hist(d, i):
    fig, ax = plt.subplots(figsize=(8, 3.6))
    moving = d["rcCommand[3]"] > 1100
    for m in range(4):
        ax.hist(d[f"motor[{m}]"][moving], bins=80, range=(0, 2048), histtype="step", lw=1.4, color=C[m], label=f"Motor {m + 1}")
    ax.set_xlabel("Motor output (DShot 0-2047)"); ax.set_ylabel("Samples")
    ax.set_title(f"Log {i}: motor output distribution while flying")
    ax.legend(ncol=4)
    save(fig, f"log{i}_07_motor_distribution.png")


def stats(d, fs):
    moving = d["rcCommand[3]"] > 1100
    s = {}
    for k in range(3):
        e = d[f"setpoint[{k}]"][moving] - d[f"gyroADC[{k}]"][moving]
        s[AX[k]] = {
            "track_err_rms_dps": float(np.sqrt(np.nanmean(e ** 2))),
            "gyro_max_dps": float(np.nanmax(np.abs(d[f"gyroADC[{k}]"]))),
            "setpoint_max_dps": float(np.nanmax(np.abs(d[f"setpoint[{k}]"]))),
        }
    s["motor_mean"] = [float(np.nanmean(d[f"motor[{m}]"][moving])) for m in range(4)]
    s["motor_sat_pct"] = [float(100 * np.mean(d[f"motor[{m}]"][moving] >= 2000)) for m in range(4)]
    s["vbat_rest_V"] = float(d["vbatLatest"][0] / 100)
    s["vbat_min_V"] = float(np.nanmin(d["vbatLatest"]) / 100)
    s["vbat_end_V"] = float(d["vbatLatest"][-200:].mean() / 100)
    s["amp_max_A"] = float(np.nanmax(d["amperageLatest"]) / 100)
    s["amp_mean_flying_A"] = float(np.nanmean(d["amperageLatest"][moving]) / 100)
    s["mah_used"] = float(np.trapezoid(d["amperageLatest"] / 100, d["t"]) / 3.6)
    s["airtime_s"] = float(moving.sum() / fs)
    s["thr_mean_flying_pct"] = float(np.nanmean((d["rcCommand[3]"][moving] - 1000) / 10))
    s["thr_end_pct"] = float((d["rcCommand[3]"][-200:].mean() - 1000) / 10)
    return s


def busiest(d, span):
    x = np.abs(d["setpoint[0]"]) + np.abs(d["setpoint[1]"])
    fs = 1 / np.median(np.diff(d["t"]))
    n = int(span * fs)
    c = np.convolve(x, np.ones(n), "valid")
    return float(d["t"][int(np.argmax(c))])


report = {}
for i in [7, 4]:
    d, names, hdr = load(i)
    fs = 1 / np.median(np.diff(d["t"]))
    overview(d, i)
    tracking(d, i)
    t0 = busiest(d, 4.0) if i == 7 else max(0.0, busiest(d, 4.0))
    tracking(d, i, t0, 4.0)
    pid_terms(d, i, t0, 4.0)
    r = {"fs_hz": float(fs), "zoom_t0": t0, "stats": stats(d, fs)}
    if i == 7:
        r["noise_peak_hz"] = spectra(d, i, fs)
        spectrogram(d, i, fs, "debug", "unfiltered gyro", "unfiltered")
        spectrogram(d, i, fs, "gyroADC", "filtered gyro", "filtered")
        r["step"] = step_response(d, i, fs)
        motor_hist(d, i)
    report[i] = r
    print(i, json.dumps(r, indent=1), flush=True)
json.dump(report, open(f"{OUT}/report.json", "w"), indent=1)
