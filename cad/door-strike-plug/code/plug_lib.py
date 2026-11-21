"""
Strike-plate plug, parameterized. Plug goes into -z, face at z = 0 (plate surface).
x = opening width (flat side at +x unless mirrored), y = opening height, z = depth.
"""
import math
import cadquery as cq

# estimated strike opening (from the duct tape trace, minus a bit for marker slop)
OPEN_W = 20.3 - 0.5
OPEN_H = 34.4 - 0.5
R_ROUND, R_FLAT = 8.0, 2.0
BODY_W = 19.2
SKIN = 2.0
FRONT_CH = 1.5
FACE_RECESS = 0.2

def d_profile(w, h, r_round, r_flat, depth):
    s = cq.Workplane("XY").rect(w, h).extrude(-depth)
    s = s.edges("|Z").edges("<X").fillet(r_round)
    s = s.edges("|Z").edges(">X").fillet(r_flat)
    return s

def yz_cut(points, width=60.0):
    return cq.Workplane("YZ").workplane(offset=-width / 2).polyline(points).close().extrude(width)

def profile_points(body_h, depth, foot, taper_start, rear_taper):
    """Side silhouette (y, z) used for the rocking check. Face at z = -FACE_RECESS."""
    half = body_h / 2
    rh = half - rear_taper
    p = [(half - FRONT_CH, 0), (half, -FRONT_CH), (half, -taper_start), (rh, -depth),
         (1.5, -depth), (1.5, -depth - foot), (-1.5, -depth - foot), (-1.5, -depth),
         (-rh, -depth), (-half, -taper_start), (-half, -FRONT_CH), (-(half - FRONT_CH), 0)]
    return [(y, z - FACE_RECESS) for y, z in p]

def build(squeeze=0.8, depth=10.0, foot=1.0, mirror=False, body_h=29.0, taper_start=4.5,
          rear_taper=3.0, wall=0.9, bump_z=None, bump_r=1.6, slit_root=2.8, open_w=OPEN_W):
    if bump_z is None:
        bump_z = -min(8.2, depth - 1.6)
    bump_len = (open_w + squeeze - BODY_W) / 2
    half = body_h / 2
    rear_half = half - rear_taper

    body = d_profile(BODY_W, body_h, R_ROUND, R_FLAT, depth)
    for sgn in (1, -1):   # front chamfers
        pts = [(sgn * (half - FRONT_CH - 0.5), 0.5), (sgn * half, -FRONT_CH),
               (sgn * (half + 3), -FRONT_CH), (sgn * (half + 3), 0.5)]
        body = body.cut(yz_cut(pts))
    for sgn in (1, -1):   # rear taper
        pts = [(sgn * half, -taper_start), (sgn * rear_half, -depth), (sgn * rear_half, -depth - 1),
               (sgn * (half + 3), -depth - 1), (sgn * (half + 3), -taper_start)]
        body = body.cut(yz_cut(pts))

    # hollow from the back
    cav_w = BODY_W - 2 * wall
    cav_h = 2 * (rear_half - 1.4)
    cav = d_profile(cav_w, cav_h, max(R_ROUND - wall, 1.0), 0.4, depth - SKIN).translate((0, 0, -SKIN))
    cav = cav.union(cav.translate((0, 0, -1)))
    body = body.cut(cav)

    # rear rib + foot
    outline = d_profile(BODY_W, body_h - 0.2, R_ROUND, R_FLAT, depth + 1)
    for sgn in (1, -1):
        outline = outline.cut(yz_cut([(sgn * half, -taper_start), (sgn * rear_half, -depth),
                                      (sgn * rear_half, -depth - 2), (sgn * (half + 3), -depth - 2),
                                      (sgn * (half + 3), -taper_start)]))
    rib = cq.Workplane("XY").box(2.0, cav_h + 2.0, 1.6, centered=(True, True, False)).translate((0, 0, -depth))
    body = body.union(rib.intersect(outline))
    body = body.union(cq.Workplane("XY").box(6.0, 3.0, foot + 0.4, centered=(True, True, False)).translate((0, 0, -depth - foot)))

    # slits freeing the two side tongues (side walls only)
    wall_x0 = cav_w / 2 - 0.2
    for sx in (1, -1):
        for yc in (4.5, -4.5):
            slit = cq.Workplane("XY").box(BODY_W / 2 + 1.0 - wall_x0, 1.0, depth + 0.5 - slit_root, centered=(False, True, False))
            slit = slit.translate((wall_x0 if sx == 1 else -(BODY_W / 2 + 1.0), yc, -depth - 0.5))
            body = body.cut(slit)

    # side bumps
    for sx in (1, -1):
        b = cq.Workplane("YZ").workplane(offset=sx * (BODY_W / 2 - 0.2)).center(0, bump_z).circle(bump_r).extrude(sx * (bump_len + 0.2))
        body = body.union(b)

    # press dish on the lower half of the face
    body = body.cut(cq.Workplane("XY").ellipse(6.0, 5.0).extrude(-0.35).translate((0, -8.0, 0)))
    body = body.translate((0, 0, -FACE_RECESS))
    if mirror:
        body = body.mirror("YZ")
    return body

# ---------------------------------------------------------------------------
# rocking-clearance and grip checks
# ---------------------------------------------------------------------------
def _sample(poly, step=0.2):
    pts = []
    for i in range(len(poly)):
        (y0, z0), (y1, z1) = poly[i], poly[(i + 1) % len(poly)]
        n = max(int(math.hypot(y1 - y0, z1 - z0) / step), 1)
        pts += [(y0 + (y1 - y0) * k / n, z0 + (z1 - z0) * k / n) for k in range(n)]
    return pts

def rock_check(open_h, depth=10.0, foot=1.0, body_h=29.0, taper_start=4.5, rear_taper=3.0):
    """Max rocking angle before binding on the hole (deg), top-edge lift there, lift at 12 deg, depth needed at 12 deg."""
    poly = profile_points(body_h, depth, foot, taper_start, rear_taper)
    pts = _sample(poly)
    pz = -(depth + foot + FACE_RECESS)
    def rot(deg):
        t = math.radians(deg); c, s = math.cos(t), math.sin(t)
        return [(y * c - (z - pz) * s, y * s + (z - pz) * c + pz) for y, z in pts]
    best = 0.0
    for tenth in range(0, 300):
        r = rot(tenth / 10)
        if any(z < -1e-6 and abs(y) > open_h / 2 for y, z in r):
            break
        best = tenth / 10
    top = poly[0]
    def top_lift(deg):
        t = math.radians(deg); c, s = math.cos(t), math.sin(t)
        y, z = top
        return y * s + (z - pz) * c + pz
    r12 = rot(12)
    return best, top_lift(best), top_lift(12), -min(z for y, z in r12)

def tongue(wall, depth, bump_z=None, slit_root=2.8, w=8.0, E=2000.0):
    if bump_z is None:
        bump_z = -min(8.2, depth - 1.6)
    L = abs(bump_z) - slit_root
    return E * w * wall ** 3 / (4 * L ** 3), L

def grip(squeeze, hole_delta, wall, depth, bump_z=None, slit_root=2.8, mu=0.35):
    """hole_delta = how much wider the real hole is than the estimate. Returns (grip N, tongue strain %)."""
    k, L = tongue(wall, depth, bump_z, slit_root)
    d = max((squeeze - hole_delta) / 2, 0.0)
    return 2 * mu * k * d, 1.5 * wall * d / L ** 2 * 100
