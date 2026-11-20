"""Independent checks.   python -m msdsim.verify

1. static equilibrium residual ~ 0
2. stiffness matrix symmetric (conservative system) and positive definite (stable)
3. undamped nonlinear ring-down: FFT peaks match the linearised eigenfrequencies
4. undamped, undriven nonlinear run conserves total energy
"""
import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks
from .params import Params
from .model import Model, quat_to_R
from .modal import modes


def undamped_params():
    p = Params()
    z = lambda t: (t[0], t[1], 0.)
    return Params(k1x=z(p.k1x), k1y=z(p.k1y), k1z=z(p.k1z), k12=z(p.k12), k2=z(p.k2), k2y=z(p.k2y), c_theta=0., cM=0.)


def main():
    m = Model(undamped_params())
    x, y0, r = m.equilibrium()
    md = modes(m); f = md["f_undamped"]
    ok = True
    print(f"1. equilibrium residual: {r:.1e}                       {'PASS' if r < 1e-8 else 'FAIL'}")
    pd = np.all(np.linalg.eigvalsh(0.5 * (md['K'] + md['K'].T)) > 0)
    print(f"2. K asymmetry {md['K_asym']:.1e}, positive definite: {pd}       {'PASS' if md['K_asym'] < 1e-6 and pd else 'FAIL'}")

    rng = np.random.default_rng(0)
    y = y0.copy(); y[7:10] += 1e-4 * rng.standard_normal(3); y[10:13] += 1e-3 * rng.standard_normal(3)
    y[16:19] += 1e-4 * rng.standard_normal(3); y[20] += 1e-3; y[22] += 1e-4
    T, n = 20.0, 8001
    t = np.linspace(0, T, n)
    sol = solve_ivp(lambda tt, yy: m.rhs(tt, yy, False), (0, T), y, method="DOP853", t_eval=t, rtol=1e-9, atol=1e-12)
    Y = sol.y.T
    sig = np.c_[Y[:, 0:3], Y[:, 13:16], Y[:, 19], Y[:, 21], Y[:, 10:13]]
    sig = (sig - sig.mean(0)) * np.hanning(n)[:, None]
    S = np.abs(np.fft.rfft(sig, axis=0, n=4 * n)).sum(1)
    fr = np.fft.rfftfreq(4 * n, T / (n - 1))
    pk, _ = find_peaks(S, height=S.max() * 1e-3)
    pf = fr[pk]
    print("3. mode  linear[Hz]  nearest FFT peak[Hz]  err[%]")
    worst = 0
    for fi in f:
        j = np.argmin(abs(pf - fi)); e = abs(pf[j] - fi) / fi * 100; worst = max(worst, e)
        print(f"        {fi:8.3f}    {pf[j]:8.3f}           {e:5.2f}")
    print(f"   worst error {worst:.2f}%                                  {'PASS' if worst < 1.0 else 'FAIL'}")

    E = np.array([m.energy(yy, 0.0, False) for yy in Y])
    drift = np.ptp(E) / max(abs(E - E.min()).max(), 1e-30)
    print(f"4. energy drift (undamped, undriven): {np.ptp(E):.1e} J      {'PASS' if np.ptp(E) < 1e-8 else 'FAIL'}")


if __name__ == "__main__":
    main()
