"""Linearisation about static equilibrium:  M q'' + C q' + K q = f(t).

Coordinates dq = [dr1, dphi (world small rotation), dr2, dtheta, dzM].
K and C are obtained by central differences of the exact nonlinear loads, so every
geometric effect (pretension, oblique springs, off-centre moments) is included.
"""
import numpy as np
from scipy.linalg import eigh, eig
from .model import Model, exp_so3


def linearise(model: Model, h=1e-6):
    P = model.P
    x, _, _ = model.equilibrium()
    r1e, phie, r2e, the, zMe = x[0:3], x[3:6], x[6:9], x[9], x[10]
    Re = exp_so3(phie)

    def Q(dq, dqd):
        L = model.loads(0, r1e + dq[0:3], exp_so3(dq[3:6]) @ Re, dqd[0:3], dqd[3:6],
                        r2e + dq[6:9], dqd[6:9], the + dq[9], dqd[9], zMe + dq[10], dqd[10], drives=False)
        return np.r_[L["F1"], L["T1"], L["F2"], L["Tp"], L["FM"]]

    n = 11; K = np.zeros((n, n)); C = np.zeros((n, n)); z = np.zeros(n)
    for j in range(n):
        e = np.zeros(n); e[j] = h
        K[:, j] = -(Q(e, z) - Q(-e, z)) / (2 * h)
        C[:, j] = -(Q(z, e) - Q(z, -e)) / (2 * h)
    Jw = Re @ model.J @ Re.T
    M = np.zeros((n, n))
    M[0:3, 0:3] = P.m1 * np.eye(3); M[3:6, 3:6] = Jw
    M[6:9, 6:9] = P.m2 * np.eye(3); M[9, 9] = P.Ip; M[10, 10] = P.M
    return M, C, K


def modes(model: Model):
    """Undamped frequencies [Hz] + shapes, damped eigenvalues, and K asymmetry (a check)."""
    M, C, K = linearise(model)
    asym = np.linalg.norm(K - K.T) / np.linalg.norm(K)
    Ks = 0.5 * (K + K.T)
    w2, phi = eigh(Ks, M)
    f = np.sqrt(np.abs(w2)) / (2 * np.pi)
    n = M.shape[0]
    A = np.block([[np.zeros((n, n)), np.eye(n)], [-np.linalg.solve(M, K), -np.linalg.solve(M, C)]])
    lam = eig(A, right=False)
    lam = lam[lam.imag > 1e-9]
    lam = lam[np.argsort(lam.imag)]
    return dict(f_undamped=f, shapes=phi, f_damped=lam.imag / (2 * np.pi),
                zeta=-lam.real / np.abs(lam), K_asym=asym, M=M, C=C, K=K)
