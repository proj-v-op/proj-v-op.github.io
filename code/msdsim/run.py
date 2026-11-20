"""Simulate, print modal table, run checks, save plots.   python -m msdsim.run"""
import sys, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp, cumulative_simpson
from .params import Params
from .model import Model, quat_to_R
from .modal import modes


def simulate(model, y0, T=12.0, n=12001, drives=True, rtol=1e-10, atol=1e-12):
    t = np.linspace(0, T, n)
    sol = solve_ivp(lambda tt, y: model.rhs(tt, y, drives), (0, T), y0, method="DOP853",
                    t_eval=t, rtol=rtol, atol=atol)
    assert sol.success, sol.message
    return sol.t, sol.y.T


def diagnostics(model, t, Y, drives=True):
    """String tensions, energy residual from  dE/dt = P_in - P_diss."""
    out = {k: np.zeros(len(t)) for k in ("TL", "TR", "Pin", "Pd", "E")}
    for i, (ti, y) in enumerate(zip(t, Y)):
        r1, q, v1, wb, r2, v2, th, thd, zM, zMd = model.unpack(y)
        R = quat_to_R(q)
        L = model.loads(ti, r1, R, v1, R @ wb, r2, v2, th, thd, zM, zMd, drives)
        out["TL"][i], out["TR"][i], out["Pin"][i], out["Pd"][i] = L["TL"], L["TR"], L["Pin"], L["Pdiss"]
        out["E"][i] = model.energy(y, ti, drives)
    W = cumulative_simpson(out["Pin"] - out["Pd"], x=t, initial=0)
    out["resid"] = out["E"] - out["E"][0] - W
    return out


def main(T=12.0):
    P = Params(); m = Model(P)
    x, y0, r = m.equilibrium()
    print(f"static equilibrium residual |f| = {r:.2e}")
    md = modes(m)
    print("\nundamped f [Hz]:", np.round(md["f_undamped"], 3))
    print("damped   f [Hz]:", np.round(md["f_damped"], 3))
    print("damping ratio  :", np.round(md["zeta"], 4))
    print(f"K asymmetry (relative): {md['K_asym']:.2e}")

    t, Y = simulate(m, y0, T)
    d = diagnostics(m, t, Y)
    print(f"\nstrings taut for all t: min TL = {d['TL'].min():.2f} N, min TR = {d['TR'].min():.2f} N")
    print(f"energy balance: max|E - E0 - int(Pin-Pd)| / max|E-E0| = {np.abs(d['resid']).max() / np.abs(d['E'] - d['E'][0]).max():.2e}")

    fig, ax = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    ax[0, 0].plot(t, 1e3 * (Y[:, 0:3] - y0[0:3])); ax[0, 0].set_ylabel("B1 position [mm]"); ax[0, 0].legend(["x", "y", "z"])
    ax[0, 1].plot(t, 1e3 * (Y[:, 13:16] - y0[13:16])); ax[0, 1].set_ylabel("m2 position [mm]"); ax[0, 1].legend(["x", "y", "z"])
    ax[1, 0].plot(t, np.degrees(Y[:, 19] - y0[19])); ax[1, 0].set_ylabel("pulley angle [deg]")
    ax[1, 1].plot(t, 1e3 * (Y[:, 21] - y0[21])); ax[1, 1].set_ylabel("counterweight z [mm]")
    ax[2, 0].plot(t, d["TL"], label="T_L"); ax[2, 0].plot(t, d["TR"], label="T_R"); ax[2, 0].set_ylabel("tension [N]"); ax[2, 0].legend()
    ax[2, 1].plot(t, d["resid"]); ax[2, 1].set_ylabel("energy residual [J]")
    for a in ax[2]: a.set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig("response.png", dpi=130)
    print("saved response.png")


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 12.0)
