import sys, json, numpy as np, pyvista as pv, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0, "fea"); from model import *
from skfem import MeshTet, Basis, ElementVectorH1, ElementTetP2
tag, meshf = sys.argv[1], sys.argv[2]
# rebuild the same reduced mesh (geometry only)
d = np.load(meshf); X = d["X"]*1e-3; T = d["T"]; c = X[T].mean(1)
keep = ~((np.abs(c[:,0]) < 8e-3) & (np.abs(c[:,1]) > 44e-3)); T = T[keep]
from scipy.sparse.csgraph import connected_components
n=len(X); r=np.repeat(np.arange(len(T)),4); A=sp.csr_matrix((np.ones(r.size),(T.ravel(),r)),shape=(n,len(T)))
nc, lab = connected_components(A@A.T, directed=False); big=np.bincount(lab[T[:,0]]).argmax(); T=T[lab[T[:,0]]==big]
used=np.unique(T); rm=-np.ones(n,int); rm[used]=np.arange(len(used)); P=X[used]; T=rm[T]
m = MeshTet(P.T.copy(), T.T.copy()); ib = Basis(m, ElementVectorH1(ElementTetP2()))
fixed, top = boundary_sets(m, ib)
grid = pv.UnstructuredGrid({pv.CellType.TETRA: T}, P*1e3)
def vert(u): return (u[ib.nodal_dofs]).T                      # (nverts,3) m
R = json.load(open(f"out/{tag}.json"))
def phone_box(q=None, s=1.0):
    b = pv.Box(bounds=(-PH[0]/2*1e3, PH[0]/2*1e3, -PH[1]/2*1e3, PH[1]/2*1e3, 10, 10+PH[2]*1e3))
    return b
pl_kw = dict(window_size=(1600,1000), off_screen=True)
# 1. mesh + boundary conditions
p = pv.Plotter(**pl_kw); p.set_background("white")
p.add_mesh(grid.extract_surface(), color="#cfe3e6", show_edges=True, edge_color="#5b7a80", line_width=0.3)
fx = np.unique(ib.nodal_dofs.T.reshape(-1,3)[np.isin(ib.nodal_dofs[0], fixed)][:,0]//3) if False else None
vfix = np.isin(ib.nodal_dofs[0], fixed); vtop = np.isin(ib.nodal_dofs[0], top)
p.add_points(P[vfix]*1e3, color="#c0392b", point_size=6, render_points_as_spheres=True, label="fixed to bar")
p.add_points(P[vtop]*1e3, color="#2e86de", point_size=3, render_points_as_spheres=True, label="tied to phone (rigid)")
p.add_text("red: fixed to the bar (hub underside)\nblue: tied to the phone (top face)", position="upper_left", color="black", font_size=13); p.view_vector((0.5,-0.8,-0.9), viewup=(0,1,0)); p.camera.zoom(1.2)
p.screenshot(f"img/fea-mesh.png"); p.close()
# 2. stress for each static case on deformed shape
for case, title in [("bump_z","6 g vertical (bump)"),("brake_y","1.5 g fore-aft (braking)"),("lateral_x","1.5 g sideways")]:
    u = np.load(f"out/{tag}_{case}_u.npy"); vm = np.load(f"out/{tag}_{case}_vm_el.npy")/1e3
    g = grid.copy(); g.cell_data["von Mises [kPa]"] = vm
    du = vert(u)*1e3; sc = 3.0/np.abs(du).max(); g.points = g.points + sc*du
    cl = np.percentile(vm, 99.5)
    p = pv.Plotter(**pl_kw); p.set_background("white")
    p.add_mesh(g, scalars="von Mises [kPa]", cmap="turbo", clim=(0, cl), smooth_shading=False,
               scalar_bar_args=dict(title="von Mises stress [kPa]", color="black", vertical=True, position_x=0.86, position_y=0.15, height=0.7))
    p.add_text(f"{title}   (deformation x{sc:.0f})", color="black", font_size=14)
    p.view_vector((0.6,-0.9,-0.8), viewup=(0,1,0)); p.camera.zoom(1.25)
    p.screenshot(f"img/fea-{case}.png"); p.close()
    # section through the hub (cut at x = 0), undeformed, same colors
    g0 = grid.copy(); g0.cell_data["vm"] = vm
    cut = g0.clip(normal="x", origin=(0, 0, 0), invert=True)
    p = pv.Plotter(**pl_kw); p.set_background("white")
    p.add_mesh(cut, scalars="vm", cmap="turbo", clim=(0, cl), show_scalar_bar=True,
               scalar_bar_args=dict(title="von Mises stress [kPa]", color="black", vertical=True, position_x=0.86, position_y=0.15, height=0.7))
    p.add_text(f"{title}: section through the hub (x = 0)", color="black", font_size=14)
    p.view_vector((1, 0, 0), viewup=(0, 0, 1)); p.camera.focal_point = (0, 0, 5); p.camera.zoom(2.6)
    p.screenshot(f"img/fea-{case}-section.png"); p.close()
# 3. first two mode shapes
modes = np.load(f"out/{tag}_modes.npy"); f = R["freqs_Hz"]
for i in range(2):
    u = modes[:, i]; du = vert(u); du = du/np.abs(du).max()*4.0
    g = grid.copy(); g.points = g.points + du; g.point_data["|u|"] = np.linalg.norm(du, axis=1)
    p = pv.Plotter(**pl_kw); p.set_background("white")
    p.add_mesh(grid.extract_surface(), color="#bbbbbb", opacity=0.25)
    p.add_mesh(g, scalars="|u|", cmap="viridis", show_scalar_bar=False)
    p.add_text(f"Mode {i+1}: {f[i]:.0f} Hz", color="black", font_size=16)
    p.view_vector((0.6,-0.9,0.9), viewup=(0,0,1)); p.camera.zoom(1.2)
    p.screenshot(f"img/fea-mode{i+1}.png"); p.close()
# 4. transmissibility
t = np.load(f"out/{tag}_trans.npz")
fig, ax = plt.subplots(figsize=(8,4.2), dpi=150)
ax.axvspan(0.5, 80, color="#f2e6c9", label="ride-comfort range for scooters (0.5-80 Hz)")
for k, lab, col in [("z","vertical","#c0392b"),("y","fore-aft","#2e86de"),("x","sideways","#27ae60")]:
    ax.semilogy(t["freq"], t[k], color=col, label=lab)
ax.axhline(1, color="k", lw=0.6); ax.set_xlim(0, 200); ax.set_xlabel("frequency [Hz]")
ax.set_ylabel("phone accel / bar accel"); ax.set_title("Vibration transmissibility (phone CG), damping ratio 0.05 assumed")
ax.legend(fontsize=8, loc="upper right"); ax.grid(alpha=0.3, which="both"); fig.tight_layout(); fig.savefig("img/fea-transmissibility.png")
print("plots done")
