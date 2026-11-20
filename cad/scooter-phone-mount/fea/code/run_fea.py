import sys, json, time, numpy as np, scipy.sparse as sp, scipy.sparse.linalg as sla, pypardiso
sys.path.insert(0, "fea"); from model import *
E = float(sys.argv[2]); meshf = sys.argv[1]; tag = sys.argv[3]
t0 = time.time()
m, ib, K, Mm = build(meshf, E); check_dof_order(ib)
fixed, top = boundary_sets(m, ib)
P, Kr, Mr, nf, rc = phone_rigid(m, ib, K, Mm, fixed, top)
print(tag, "DOF", Kr.shape[0], "fixed", len(fixed), "top", len(top), f"{time.time()-t0:.0f}s")
ex = (np.arange(ib.N)%3==0).astype(float); net_mass = float(ex @ (Mm @ ex))
# --- modal (shift-invert with pardiso)
solver = pypardiso.PyPardisoSolver()
Kc = Kr.tocsr()
op = sla.LinearOperator(Kr.shape, matvec=lambda v: solver.solve(Kc, v), dtype=float)
# generalized: K v = w2 M v  -> eigsh(A=M, M=K) not SPD-friendly; use OPinv
bt=np.random.rand(Kc.shape[0]); res_chk=np.linalg.norm(Kc@solver.solve(Kc,bt)-bt)/np.linalg.norm(bt); print('solver residual',res_chk)
vals, V = sla.eigsh(Kr, k=8, M=Mr, sigma=0.0, OPinv=op)
f = np.sqrt(np.abs(vals))/(2*np.pi)
# phone participation of each mode (which direction it moves)
ph = V[nf:nf+6, :]
# --- influence vectors (unit base translation): all DOFs of that component =1, phone translation =1
def rvec(k):
    r = np.zeros(Kr.shape[0]); comp = (np.arange(ib.N)%3)
    full = (comp==k).astype(float); r[:nf] = (P[:, :nf].T @ full) ; r[nf+k] = 1.0; return r
# --- static inertial load cases (quasi-static)
cases = {"bump_z": (2, 6.0), "brake_y": (1, 1.5), "lateral_x": (0, 1.5)}
res = {"tag": tag, "E": E, "dof": int(Kr.shape[0]), "net_mass_g": net_mass*1e3, "freqs_Hz": f.tolist(), "solver_residual": float(res_chk),
       "mode_phone": np.round(ph/np.abs(ph).max(0), 2).T.tolist(), "cases": {}}
U = {}
for name, (k, ng) in cases.items():
    F = -Mr @ rvec(k) * ng * G          # inertial body force for ng g along -k
    q = solver.solve(Kc, F)
    u = P @ q; U[name] = u
    # strains/stresses at quadrature points
    from skfem.helpers import sym_grad, trace, eye
    du = ib.interpolate(u); eps = sym_grad(du)
    lam, mu = lame_parameters(E, NU)
    e = np.array(eps)                                     # (3,3,nel,nqp)
    tr = e[0,0]+e[1,1]+e[2,2]
    s = 2*mu*e + lam*tr*np.eye(3)[:,:,None,None]
    vm = np.sqrt(0.5*((s[0,0]-s[1,1])**2+(s[1,1]-s[2,2])**2+(s[2,2]-s[0,0])**2)+3*(s[0,1]**2+s[1,2]**2+s[0,2]**2))
    ee = np.moveaxis(e, (0,1), (-2,-1)); ep = np.linalg.eigvalsh(ee)[..., -1]
    res["cases"][name] = {"g": ng, "phone_disp_mm": (q[nf:nf+3]*1e3).tolist(),
                          "phone_rot_deg": np.degrees(q[nf+3:nf+6]).tolist(),
                          "max_vm_kPa": float(vm.max()/1e3), "p99_vm_kPa": float(np.percentile(vm, 99.9)/1e3),
                          "max_principal_strain_pct": float(ep.max()*100), "p999_strain_pct": float(np.percentile(ep, 99.9)*100)}
    np.save(f"out/{tag}_{name}_vm_el.npy", vm.max(1))
# --- base-excitation transmissibility (modal, zeta assumed)
ZETA = 0.05
mn = np.einsum("ij,ij->j", V, Mr @ V)
freq = np.linspace(1, 200, 800); w = 2*np.pi*freq; wi = 2*np.pi*f
corner = np.array([PH[0]/2, PH[1]/2, PH[2]/2])          # phone corner relative to CG
trans = {}; effm = {}
for k, lab in enumerate("xyz"):
    r = rvec(k); Gam = (V.T @ (Mr @ r))/mn
    effm[lab] = float((Gam**2*mn).sum()/(r @ Mr @ r))
    # phone CG absolute accel / base accel along same axis
    phi = V[nf+k, :]
    H = 1 + (phi*Gam*w[:,None]**2/(wi**2 - w[:,None]**2 + 2j*ZETA*wi*w[:,None])).sum(1)
    trans[lab] = np.abs(H)
res["eff_mass_fraction_8modes"] = effm
np.savez(f"out/{tag}_trans.npz", freq=freq, **trans)
np.save(f"out/{tag}_V.npy", V); np.save(f"out/{tag}_modes.npy", (P @ V[:, :6])); np.save(f"out/{tag}_freqs.npy", f)
for n,u in U.items(): np.save(f"out/{tag}_{n}_u.npy", u)
json.dump(res, open(f"out/{tag}.json","w"), indent=1)
print(json.dumps({k:v for k,v in res.items() if k!="mode_phone"}, indent=1)); print(f"{time.time()-t0:.0f}s")
