"""Equations of motion for the 3D coupled system (11 DOF, 23 states).

Generalised coordinates   : r1 (3), orientation of B1, r2 (3), theta, zM
State vector y (23)       : [r1(3), q(4), v1(3), w_body(3), r2(3), v2(3), th, thd, zM, zMd]

Every spring/damper is one `Element` between two points; its force on point a is
    F_a = [ k d + kappa d^3 + c Ldot ] n,   d = L - L0,   n = (b-a)/L,   Ldot = n.(v_b - v_a)
and -F_a acts on point b.  Strings and drives are in `loads`.
"""
import numpy as np
from scipy.optimize import root
from scipy.spatial.transform import Rotation
from .params import Params


# ----------------------------------------------------------------- rotation helpers
def skew(a):
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def exp_so3(phi):
    t = np.linalg.norm(phi)
    if t < 1e-12:
        return np.eye(3) + skew(phi)
    K = skew(phi / t)
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


def quat_to_R(q):
    q = q / np.linalg.norm(q); w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def R_to_quat(R):
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def quat_mul(a, b):
    return np.array([a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                     a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                     a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                     a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])


# ----------------------------------------------------------------- model
class Model:
    def __init__(self, P: Params = None):
        self.P = P = P or Params()
        g = P.geom
        self.J = P.J1
        self.nF = np.asarray(P.nF, float) / np.linalg.norm(P.nF)
        # element table: (name, (k, kappa, c), a=(kind, local), b=(kind, local))
        spec = [
            ("x",  P.k1x, ("B1", g["rho_x"]),  ("Gx", g["anc_x"])),   # moving base wall
            ("y",  P.k1y, ("B1", g["rho_y"]),  ("G",  g["anc_y"])),
            ("z",  P.k1z, ("B1", g["rho_z"]),  ("G",  g["anc_z"])),
            ("x2", P.k1x, ("B1", g["rho_x2"]), ("Gx", g["anc_x2"])),   # mirrored partners
            ("y2", P.k1y, ("B1", g["rho_y2"]), ("G",  g["anc_y2"])),
            ("z2", P.k1z, ("B1", g["rho_z2"]), ("G",  g["anc_z2"])),
            ("12", P.k12, ("B1", g["rho_12"]), ("m2", np.zeros(3))),
            ("2",  P.k2,  ("m2", np.zeros(3)), ("G",  g["anc_2"])),
            ("2y", P.k2y, ("m2", np.zeros(3)), ("G",  g["anc_2y"])),
        ]
        self.elements = []
        for name, (k, kap, c), a, b in spec:
            pa = g["p2"] if a[0] == "m2" else a[1]
            pb = g["p2"] if b[0] == "m2" else b[1]
            L0 = np.linalg.norm(pb - pa)          # rest length = drawn length
            self.elements.append(dict(name=name, k=k, kappa=kap, c=c, a=a, b=b, L0=L0))
        self.r2_0 = g["p2"].copy()
        self.zA0 = g["rho_A"][2]
        self.zM0 = g["pM"][2]
        self.zMrest = g["pM"][2]

    # ------------------------------------------------------------ drives
    def base(self, t, drives=True):
        P = self.P
        if not drives: return 0.0, 0.0
        return P.U * np.sin(P.Wb * t), P.U * P.Wb * np.cos(P.Wb * t)

    # ------------------------------------------------------------ loads
    def _pt(self, spec, t, drives, r1, R, v1, w, r2, v2):
        kind, loc = spec
        if kind == "B1":
            rr = R @ loc
            return r1 + rr, v1 + np.cross(w, rr), rr
        if kind == "m2":
            return r2, v2, None
        u, ud = self.base(t, drives) if kind == "Gx" else (0.0, 0.0)
        return loc + np.array([u, 0, 0]), np.array([ud, 0, 0]), None

    def loads(self, t, r1, R, v1, w, r2, v2, th, thd, zM, zMd, drives=True):
        """Forces/torques (world frame) on each body, plus diagnostics."""
        P = self.P; g = P.geom
        F1 = np.array([0, 0, -P.m1 * P.g]); T1 = np.zeros(3)
        F2 = np.array([0, 0, -P.m2 * P.g])
        Pin_anchor = 0.0; Pdiss = 0.0; Epot = 0.0; fel = {}
        for e in self.elements:
            pa, va, ra = self._pt(e["a"], t, drives, r1, R, v1, w, r2, v2)
            pb, vb, rb = self._pt(e["b"], t, drives, r1, R, v1, w, r2, v2)
            d = pb - pa; L = np.linalg.norm(d); n = d / L
            dl = L - e["L0"]; Ld = n @ (vb - va)
            f = e["k"] * dl + e["kappa"] * dl ** 3 + e["c"] * Ld
            Fa = f * n
            for (kind, _), pt_rr, Fv in ((e["a"], ra, Fa), (e["b"], rb, -Fa)):
                if kind == "B1": F1 = F1 + Fv; T1 = T1 + np.cross(pt_rr, Fv)
                elif kind == "m2": F2 = F2 + Fv
            if e["b"][0] == "Gx": Pin_anchor += Fa @ vb          # work by the moving wall
            Pdiss += e["c"] * Ld ** 2
            Epot += 0.5 * e["k"] * dl ** 2 + 0.25 * e["kappa"] * dl ** 4
            fel[e["name"]] = f
        # --- strings over the pulley (taut-only), small-angle: strings stay vertical
        rA = R @ g["rho_A"]; zA = r1[2] + rA[2]
        eL = P.Rp * th - (zA - self.zA0) + P.eL0
        eR = -P.Rp * th - (zM - self.zM0) + P.eR0
        TL = P.ks * max(0.0, eL); TR = P.ks * max(0.0, eR)
        F1 = F1 + np.array([0, 0, TL]); T1 = T1 + np.cross(rA, [0, 0, TL])
        Epot += 0.5 * P.ks * (max(0, eL) ** 2 + max(0, eR) ** 2)
        # --- rotor unbalance on B1 (rotates in the body x-z plane)
        Fu = R @ (P.mue * P.Wr ** 2 * np.array([np.cos(P.Wr * t), 0, np.sin(P.Wr * t)])) if drives else np.zeros(3)
        ru = R @ g["rho_u"]
        F1 = F1 + Fu; T1 = T1 + np.cross(ru, Fu)
        vu = v1 + np.cross(w, ru)
        # --- harmonic force on m2
        Fh = P.F2 * np.sin(P.W2 * t) * self.nF if drives else np.zeros(3)
        F2 = F2 + Fh
        # --- pulley and counterweight
        tau = P.tau0 * np.sin(P.Wp * t) if drives else 0.0
        Tp = P.Rp * (TR - TL) - P.c_theta * thd - P.k_theta * th + tau
        FM = -P.M * P.g + TR - P.cM * zMd - P.kM * (zM - self.zMrest)
        Epot += 0.5 * P.k_theta * th ** 2 + 0.5 * P.kM * (zM - self.zMrest) ** 2
        Pdiss += P.c_theta * thd ** 2 + P.cM * zMd ** 2
        Pin = Pin_anchor + Fu @ vu + Fh @ v2 + tau * thd
        return dict(F1=F1, T1=T1, F2=F2, Tp=Tp, FM=FM, TL=TL, TR=TR, Pin=Pin, Pdiss=Pdiss, Epot=Epot, f=fel)

    # ------------------------------------------------------------ dynamics
    def unpack(self, y):
        r1, q, v1, wb = y[0:3], y[3:7], y[7:10], y[10:13]
        r2, v2 = y[13:16], y[16:19]
        th, thd, zM, zMd = y[19:23]
        return r1, q, v1, wb, r2, v2, th, thd, zM, zMd

    def rhs(self, t, y, drives=True):
        P = self.P
        r1, q, v1, wb, r2, v2, th, thd, zM, zMd = self.unpack(y)
        R = quat_to_R(q); w = R @ wb
        L = self.loads(t, r1, R, v1, w, r2, v2, th, thd, zM, zMd, drives)
        a1 = L["F1"] / P.m1
        wbd = np.linalg.solve(self.J, R.T @ L["T1"] - np.cross(wb, self.J @ wb))
        qd = 0.5 * quat_mul(q, np.r_[0, wb])
        return np.r_[v1, qd, a1, wbd, v2, L["F2"] / P.m2, thd, L["Tp"] / P.Ip, zMd, L["FM"] / P.M]

    def energy(self, y, t=0.0, drives=True):
        P = self.P
        r1, q, v1, wb, r2, v2, th, thd, zM, zMd = self.unpack(y)
        R = quat_to_R(q); w = R @ wb
        L = self.loads(t, r1, R, v1, w, r2, v2, th, thd, zM, zMd, drives)   # Epot needs the wall position u(t)
        T = 0.5 * P.m1 * v1 @ v1 + 0.5 * wb @ self.J @ wb + 0.5 * P.m2 * v2 @ v2 + 0.5 * P.Ip * thd ** 2 + 0.5 * P.M * zMd ** 2
        V = L["Epot"] + P.g * (P.m1 * r1[2] + P.m2 * r2[2] + P.M * zM)
        return T + V

    # ------------------------------------------------------------ statics
    def equilibrium(self, relax_time=60.0):
        """Static equilibrium (drives off). Returns (x, y0, |residual|), x = [r1, phi, r2, th, zM].

        Damped relaxation of the full dynamics finds the *stable* branch (the one the real system
        settles into); a Levenberg-Marquardt polish then drives the residual to round-off.
        """
        from scipy.integrate import solve_ivp

        def res(x):
            r1, phi, r2, th, zM = x[0:3], x[3:6], x[6:9], x[9], x[10]
            L = self.loads(0, r1, exp_so3(phi), np.zeros(3), np.zeros(3), r2, np.zeros(3), th, 0, zM, 0, drives=False)
            return np.r_[L["F1"], L["T1"], L["F2"], L["Tp"], L["FM"]]

        y = np.r_[np.zeros(3), 1, 0, 0, 0, np.zeros(6), self.r2_0, np.zeros(3), 0, 0, self.zM0, 0]
        sol = solve_ivp(lambda t, yy: self.rhs(t, yy, False), (0, relax_time), y, method="LSODA", rtol=1e-9, atol=1e-11)
        y = sol.y[:, -1]
        r1, q, _, _, r2, _, th, _, zM, _ = self.unpack(y)
        phi = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_rotvec()
        x = root(res, np.r_[r1, phi, r2, th, zM], method="lm", tol=1e-14).x
        r = np.linalg.norm(res(x))
        assert r < 1e-8, f"equilibrium not found (|f|={r:.2e})"
        y0 = np.r_[x[0:3], R_to_quat(exp_so3(x[3:6])), np.zeros(6), x[6:9], np.zeros(3), x[9], 0, x[10], 0]
        return x, y0, r
