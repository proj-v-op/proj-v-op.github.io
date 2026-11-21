"""Six strike-plug configs, flipped face-down, numbered, laid out on one plate, exported as one STL."""
import sys
import cadquery as cq
from plug_lib import *

CONFIGS = [
    # id, name, kwargs
    (1, "A snug (squeeze 0.4)",      dict(squeeze=0.4)),
    (2, "B medium (squeeze 0.8)",    dict(squeeze=0.8)),
    (3, "C tight (squeeze 1.2)",     dict(squeeze=1.2)),
    (4, "B mirrored (flat side flipped)", dict(squeeze=0.8, mirror=True)),
    (5, "Deep box (13 mm body)",     dict(squeeze=0.8, depth=13.0, body_h=28.0)),
    (6, "Shallow box (7.5 mm body)", dict(squeeze=0.5, depth=7.5, taper_start=3.5, wall=0.8, bump_z=-6.0, bump_r=1.3, slit_root=2.3)),
]
COLS, PITCH_X, PITCH_Y = 3, 28.0, 36.0

def prepare(idx, kw):
    p = build(**kw)
    assert p.val().isValid() and len(p.solids().vals()) == 1, f"config {idx} not a single valid solid"
    flipped = p.rotate((0, 0, 0), (0, 1, 0), 180)          # face goes to the bed
    bb = flipped.val().BoundingBox()
    flipped = flipped.translate((-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))
    # engrave the number on the inside of the face skin (readable from above)
    label = cq.Workplane("XY").workplane(offset=SKIN - 0.4).center(4.3, 0).text(str(idx), 5.0, 0.4, combine=False)
    flipped = flipped.cut(label)
    return flipped

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    solids = []
    for n, (idx, name, kw) in enumerate(CONFIGS):
        part = prepare(idx, kw)
        col, row = n % COLS, n // COLS
        part = part.translate((col * PITCH_X, -row * PITCH_Y, 0))
        solids.append(part.val())
        bb = part.val().BoundingBox()
        print(f"{idx}: {name:34s} {bb.xlen:5.1f} x {bb.ylen:5.1f} x {bb.zlen:5.1f} mm   vol {part.val().Volume()/1000:.2f} cm3")
    comp = cq.Compound.makeCompound(solids)
    cq.exporters.export(cq.Workplane("XY").newObject([comp]), f"{out}/strike_plug_v1_six_pack.stl", tolerance=0.02, angularTolerance=0.1)
    print("exported")
