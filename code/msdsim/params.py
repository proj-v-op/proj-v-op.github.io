"""Physical parameters and geometry for the coupled 3D system (SI units).

Geometry is taken from the system diagram: diagram units * S -> metres,
with the world origin at B1's centre of mass in the initial configuration.
All numbers live here so the model code stays symbolic.
"""
from dataclasses import dataclass
import numpy as np

S = 0.25                       # metres per diagram unit
_C1 = np.array([1.0, 0, 0])    # B1 centre in diagram coordinates


def d2m(p):
    """Diagram coordinates -> metres, origin at B1 COM."""
    return (np.array(p, float) - _C1) * S


@dataclass
class Params:
    g: float = 9.81
    # --- bodies
    m1: float = 2.0
    dims1: tuple = (2.4 * S, 1.4 * S, 0.6 * S)   # B1 box edge lengths (x, y, z)
    m2: float = 0.8
    Ip: float = 0.010            # pulley inertia
    Rp: float = 0.8 * S          # pulley radius
    M: float = 1.0               # counterweight
    # --- ground elements on B1: (k, kappa, c)
    k1x: tuple = (800., 0., 5.)
    k1y: tuple = (600., 0., 4.)
    k1z: tuple = (1500., 0., 6.)
    k12: tuple = (500., 0., 3.)
    k2: tuple = (400., 2.0e6, 3.)    # cubic hardening on m2's floor element
    k2y: tuple = (300., 0., 2.)      # m2 to back wall (3rd spring: makes m2 statically stable)
    # --- strings (taut-only), pulley bearing, counterweight guide
    ks: float = 3000.
    eL0: float = 0.004               # string pre-stretch (keeps strings taut)
    eR0: float = 0.004
    k_theta: float = 0.5
    c_theta: float = 0.01
    kM: float = 300.
    cM: float = 3.0
    # --- drives
    U: float = 0.010;   Wb: float = 2 * np.pi * 2.5     # base motion  u = U sin(Wb t)
    mue: float = 2e-3;  Wr: float = 2 * np.pi * 4.0     # rotor unbalance m_u e, speed
    F2: float = 1.5;    W2: float = 2 * np.pi * 3.3     # harmonic force on m2
    tau0: float = 0.05; Wp: float = 2 * np.pi * 1.7     # pulley motor torque
    nF: tuple = (0.6, -0.6, 0.5)                        # direction of F2 (normalised in model)

    # ---- geometry (metres, B1-COM origin) ----
    @property
    def geom(self):
        return dict(
            rho_x=d2m([-.2, -.6, 0]), anc_x=d2m([-3.7, -.6, 0]),
            rho_y=d2m([.4, .7, 0]),    anc_y=d2m([.4, 3.0, 0]),
            rho_z=d2m([-.1, .65, -.3]), anc_z=d2m([-.1, .65, -3.0]),
            rho_x2=d2m([-.2, .6, 0]), anc_x2=d2m([-3.7, .6, 0]),
            rho_y2=d2m([1.6, .7, 0]), anc_y2=d2m([1.6, 3.0, 0]),
            rho_z2=d2m([1.9, -.65, -.3]), anc_z2=d2m([1.9, -.65, -3.0]),
            rho_12=d2m([1.8, -.7, -.15]), p2=d2m([3.4, -2.3, -1.4]),
            anc_2=d2m([4.7, -1.3, -3.0]), anc_2y=d2m([3.4, 3.0, -1.4]),
            rho_A=d2m([2.2, 0, .3]),   rho_u=d2m([.2, 0, .85]),
            pM=d2m([3.8, 0, 1.15]),
        )

    @property
    def J1(self):
        a, b, c = self.dims1
        return np.diag([self.m1 / 12 * (b * b + c * c),
                        self.m1 / 12 * (a * a + c * c),
                        self.m1 / 12 * (a * a + b * b)])
