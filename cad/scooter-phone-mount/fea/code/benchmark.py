"""Check the P2-tet solver (same element, same nu) against Euler-Bernoulli.
Cantilever L x b x h, clamped at x=0.
  tip deflection under self-weight q = rho*g*b*h:  d = q L^4 / (8 E I)
  first bending frequency:                       f1 = (1.8751^2 / 2pi) sqrt(E I / (rho A L^4))
"""
import numpy as np, scipy.sparse.linalg as sla, pypardiso, sys
sys.path.insert(0, "fea")
from skfem import MeshTet, Basis, ElementVectorH1, ElementTetP2, asm, BilinearForm, LinearForm, condense, solve
from skfem.models.elasticity import linear_elasticity, lame_parameters
from skfem.helpers import dot
E, NU, RHO, g = 1.14e6, 0.49, 1180.0, 9.81
L, b, h = 0.10, 0.010, 0.010
m = MeshTet.init_tensor(np.linspace(0, L, 41), np.linspace(0, b, 5), np.linspace(0, h, 5))
ib = Basis(m, ElementVectorH1(ElementTetP2()))
lam, mu = lame_parameters(E, NU); K = asm(linear_elasticity(lam, mu), ib)
@BilinearForm
def mass(u, v, w): return RHO*dot(u, v)
@LinearForm
def grav(v, w): return -RHO*g*v[2]
M = asm(mass, ib); F = asm(grav, ib)
D = ib.get_dofs(lambda x: x[0] < 1e-9).all()
u = solve(*condense(K, F, D=D))
tip = ib.doflocs[0] > L - 1e-9; dz = u[np.where(tip)[0][2::3]] if False else u[ib.nodal_dofs[2][m.p[0] > L-1e-9]].mean()
I = b*h**3/12; q = RHO*g*b*h
d_th = q*L**4/(8*E*I)
Kc, Mc, _, I_ = condense(K, M, D=D)
vals = sla.eigsh(Kc, k=1, M=Mc, sigma=0)[0]
f_fe = np.sqrt(vals[0])/(2*np.pi); f_th = 1.8751**2/(2*np.pi)*np.sqrt(E*I/(RHO*b*h*L**4))
print(f"tip deflection  FE {-dz*1e3:.3f} mm   theory {d_th*1e3:.3f} mm   diff {abs(-dz-d_th)/d_th*100:.1f}%")
print(f"first frequency FE {f_fe:.2f} Hz   theory {f_th:.2f} Hz   diff {abs(f_fe-f_th)/f_th*100:.1f}%")
