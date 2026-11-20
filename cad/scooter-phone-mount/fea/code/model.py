"""Linear FEA of the v6 silicone phone net on a scooter bar.

Units: SI (m, kg, Pa). Quadratic tets (P2) on a linear-geometry mesh.

Model
- Net: linear elastic, isotropic, nearly incompressible silicone.
- Bar: the strap clamps the hub to the bar, so every surface node on the hub
  underside (|x|<12 mm, |y|<25 mm, z<6 mm) is fixed.
- Strap tail/tabs/keeper (|x|<8 mm, |y|>44 mm) are removed: in use they wrap
  under the bar, not stick out flat.
- Phone: a rigid body (6 DOF) bonded to every node on the top face (z = 10 mm),
  i.e. no slip between case and silicone. Valid while the stretch preload keeps
  the phone pressed on the net.
"""
import numpy as np, scipy.sparse as sp
from skfem import MeshTet, Basis, ElementVectorH1, ElementTetP2, asm
from skfem.models.elasticity import linear_elasticity, lame_parameters
from skfem.helpers import dot

RHO   = 1180.0        # kg/m^3 (Mold Star 16/30 SG 1.18)
NU    = 0.49
M_PH  = 0.220         # kg, iPhone 16 (~170 g) + case (~50 g), assumed
PH    = (0.077, 0.152, 0.011)   # phone+case size, m
G     = 9.81

def build(meshfile, E):
    d = np.load(meshfile); X = d["X"]*1e-3; T = d["T"]
    c = X[T].mean(1)
    keep = ~((np.abs(c[:,0]) < 8e-3) & (np.abs(c[:,1]) > 44e-3))
    T = T[keep]
    # keep only the largest connected piece (cutting the strap can leave slivers)
    from scipy.sparse.csgraph import connected_components
    n = len(X); r = np.repeat(np.arange(len(T)), 4)
    A = sp.csr_matrix((np.ones(r.size), (T.ravel(), r)), shape=(n, len(T)))
    nc, lab = connected_components(A @ A.T, directed=False)
    big = np.bincount(lab[T[:, 0]]).argmax(); T = T[lab[T[:, 0]] == big]
    used = np.unique(T); remap = -np.ones(len(X), int); remap[used] = np.arange(len(used))
    m = MeshTet(X[used].T.copy(), remap[T].T.copy())
    e = ElementVectorH1(ElementTetP2()); ib = Basis(m, e, intorder=4)
    lam, mu = lame_parameters(E, NU)
    K = asm(linear_elasticity(lam, mu), ib)
    from skfem import BilinearForm
    @BilinearForm
    def mass(u, v, w): return RHO*dot(u, v)
    Mm = asm(mass, ib)
    return m, ib, K, Mm

def boundary_sets(m, ib):
    """DOF index arrays: fixed (bar), top (phone) per component."""
    loc = ib.doflocs                       # (3, ndof)
    bnd = np.zeros(ib.N, bool)
    bnd[ib.get_dofs().flatten()] = True    # all boundary-facet DOFs
    x, y, z = loc
    fixed = bnd & (np.abs(x) < 12.05e-3) & (np.abs(y) < 25.05e-3) & (z < 5.99e-3)
    top   = bnd & (z > 9.99e-3) & ~fixed
    return np.where(fixed)[0], np.where(top)[0]

def phone_rigid(m, ib, K, Mm, fixed, top):
    """Reduce: u = P q, q = [free DOFs, phone (ux,uy,uz,rx,ry,rz)]."""
    N = ib.N; loc = ib.doflocs
    free = np.setdiff1d(np.arange(N), np.concatenate([fixed, top]))
    rc = np.array([0.0, loc[1][top].mean(), 10e-3 + PH[2]/2])   # phone CG
    comp = np.arange(N) % 3                 # skfem interleaves x,y,z per node
    rows, cols, vals = [], [], []
    rows += list(free); cols += list(range(len(free))); vals += [1.0]*len(free)
    nf = len(free)
    for i in top:
        r = loc[:, i] - rc; k = comp[i]
        rows.append(i); cols.append(nf+k); vals.append(1.0)          # translation
        # rotation: (theta x r)_k
        th = {0: [(1, r[2]), (2, -r[1])], 1: [(0, -r[2]), (2, r[0])], 2: [(0, r[1]), (1, -r[0])]}[k]
        for j, a in th:
            rows.append(i); cols.append(nf+3+j); vals.append(a)
    P = sp.csr_matrix((vals, (rows, cols)), shape=(N, nf+6))
    Kr = (P.T @ K @ P).tocsc(); Mr = (P.T @ Mm @ P).tolil()
    a, b, c = PH; Ip = M_PH/12*np.array([b*b+c*c, a*a+c*c, a*a+b*b])
    for k in range(3): Mr[nf+k, nf+k] += M_PH; Mr[nf+3+k, nf+3+k] += Ip[k]
    return P, Kr, Mr.tocsc(), nf, rc

def check_dof_order(ib):
    """Assert skfem's vector DOF ordering is x,y,z interleaved (used above)."""
    nd = ib.nodal_dofs; assert (nd[1] == nd[0]+1).all() and (nd[2] == nd[0]+2).all()
