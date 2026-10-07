"""Step 5: MaGaRM logo engraved on the outer flat of the upper arm (L3_back).

Font: Russo One (SIL OFL 1.1, cad/fonts). Engraving 0.8 mm into a 4 mm wall; it prints without supports on the side
wall and can be filled with a second colour (filament swap at the engraving depth, or paint).
Usage: python step5_logo.py <magarm_step4_gripper.step> <out_dir>
"""
import sys, os
from build123d import *

SRC, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)
FONT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'fonts', 'RussoOne-Regular.ttf')
TEXT, DEPTH, MAX_W, MAX_H = 'MaGaRM', 0.8, 95.0, 22.0

a = import_step(SRC)
parts = [[c.label, c] for c in a.children]
p = [p for p in parts if p[0] == 'L3_back'][0]
arm = p[1].solids()[0]
flat = max((f for f in arm.faces() if f.geom_type == GeomType.PLANE and f.normal_at().Y > 0.99), key=lambda f: f.area)
bb = flat.bounding_box()
cx, cz, y = (bb.min.X + bb.max.X) / 2, (bb.min.Z + bb.max.Z) / 2, bb.min.Y
# text in a local sketch, scaled to fit, then placed on the flat: reads left to right seen from +Y, upright
txt = Text(TEXT, font_size=20, font_path=FONT, align=(Align.CENTER, Align.CENTER))
tb = txt.bounding_box()
k = min(MAX_W / tb.size.X, MAX_H / tb.size.Y)
txt = Text(TEXT, font_size=20 * k, font_path=FONT, align=(Align.CENTER, Align.CENTER))
plane = Plane(origin=(cx, y, cz), x_dir=(-1, 0, 0), z_dir=(0, 1, 0))
cut = extrude(plane * txt, amount=DEPTH + 0.1, dir=(0, -1, 0)).moved(Location((0, 0.1, 0)))
area = sum(f.area for f in txt.faces())
new = arm - cut
assert new.is_valid and len(new.solids()) == 1, 'logo cut failed'
assert abs((arm.volume - new.volume) - area * DEPTH) < 0.02 * area * DEPTH + 1, 'unexpected logo volume'
p[1] = new
tbb = (plane * txt).bounding_box()
log = [f'L3_back: "{TEXT}" engraved {DEPTH} mm, {tbb.size.X:.1f} x {tbb.size.Z:.1f} mm, centre x={cx:.1f} z={cz:.1f} on the y={y:.2f} flat']
kids = [Compound([s], label=n) for n, s in parts]
export_step(Compound(children=kids, label='magarm'), OUT + '/magarm_step5_logo.step')
open(OUT + '/step5_log.txt', 'w').write('\n'.join(log))
print('\n'.join(log))
