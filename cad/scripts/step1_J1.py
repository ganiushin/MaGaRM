"""Step 1 (J1): MaGaRM base with Damiao DM-J4340P instead of RobStride RS00.

Start: the base assembly (cad/base/base_assembly.step, unchanged parts).
Changes:
  * L1_base: Ø39.5 x 2.5 mm counterbore under the top plate for the Ø38 rotating output flange of the 4340P
             (RS00 flange was Ø34, the plate opening is Ø36);
             tower made 7 mm taller in its constant-section zone at the cable windows (stretch, no seams):
             both windows (0° and 180°) become 17 mm tall with their original frame, the 4340P side XT30
             connectors sit inside them, and the motor ends 11 mm above the table.
  * motor:   official Damiao STEP of DM-J4340P-2EC V1.1, front face on the plate underside (z=0),
             turned so both XT30 connectors face the windows.
  * everything above J1 is lifted 0.75 mm (4340P output flange stands 1.75 mm proud of its front face,
             RS00 1.0 mm). No other part changes.
Usage: python step1_J1.py <base_assembly.step> <dm4340p.stp> <out_dir>
"""
import sys, os, math
from build123d import *

SRC, MOTOR, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
J1_X, J1_Y = 0.6, 0.0            # J1 axis (vertical)
LIFT = 1.75 - 1.0                # 4340P output flange height minus RS00's
ROT = 90.0                       # motor rotation about J1 so the connectors face the windows (0° / 180°)

a = import_step(SRC)
parts = [[c.label, c] for c in a.children]
def get(label, i=0): return [p for p in parts if p[0] == label][i]
log = []

def align_rotor(motor, origin, axis, deg):
    """The official Damiao STEP has the output rotor fused to the body at an arbitrary angle. Cut the output disc
    (r 16.3, inside the stator ring, down to the bottom of the M3 threads) and turn it so its screw/pin holes face the
    part's holes, as the real rotor is turned by hand at assembly. Body and connectors stay where they are."""
    s0, rest = motor.solids()[0], motor.solids()[1:]
    o, w = Vector(*origin), Vector(*axis)
    cut = Solid.make_cylinder(16.3, 8.0, Plane(origin=o - w * 0.1, z_dir=w))
    rotor, body = s0 & cut, s0 - cut
    assert body.is_valid and rotor.is_valid and abs(rotor.volume + body.volume - s0.volume) < 1.0, 'rotor split failed'
    rotor = rotor.rotate(Axis(o, w), deg)
    return Compound(body.solids() + rotor.solids() + rest)

# ---------------- L1_base ----------------
p = get('L1_base'); b = p[1]; v0 = b.volume
b = b - Cylinder(19.75, 2.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((J1_X, J1_Y, -0.01)))
TOWER_DZ = 7.0                   # base tower made taller in its constant-section window zone (z -47.25..-39.5)
def halfspace_z(z, above):
    return Box(2000, 2000, 1000, align=(Align.CENTER, Align.CENTER, Align.MIN if above else Align.MAX)).moved(Location((0, 0, z)))
def stretch_z_down(shape, z, dz):
    """split at z, move the lower part down by dz and fill with the section at z (seamless in a prismatic zone)"""
    up, low = shape & halfspace_z(z, True), shape & halfspace_z(z, False)
    sec = shape & Plane(origin=(0, 0, z), z_dir=(0, 0, 1))
    fill = extrude(sec, amount=dz, dir=(0, 0, -1))
    r = up + fill + low.moved(Location((0, 0, -dz)))
    assert r.is_valid and abs(r.volume - shape.volume - sec.area * dz) < 5, 'stretch failed'
    return r
b = stretch_z_down(b, -44.0, TOWER_DZ)
p[1] = b
log.append(f'L1_base: Ø39.5x2.5 counterbore for the 4340P output flange; tower +{TOWER_DZ:.0f} mm in the window zone (both windows 17 mm tall, motor 11 mm above the table)')

# ---------------- L2_base: dowel pins for the 4340P output ----------------
# RS00: 3x Ø3 pins on Ø24.5 (holes Ø3.1, z 1..23); 4340P output: 3x Ø4 H7 pin holes on Ø24, 6 deep.
# Screws are the same (6x M3 on Ø27). Pins sit between the screws in both (30° offset), angles 0/120/240.
p = get('L2_base'); l2 = p[1]; v0 = l2.volume
for ang in (0, 120, 240):
    a_ = math.radians(ang)
    plug = Cylinder(1.6, 22.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((J1_X + 12.23 * math.cos(a_), J1_Y + 12.23 * math.sin(a_), 1.0)))
    l2 = l2 + plug
    hole = Cylinder(2.0, 22.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((J1_X + 12.0 * math.cos(a_), J1_Y + 12.0 * math.sin(a_), 0.99)))
    l2 = l2 - hole
assert l2.is_valid and abs(l2.volume - v0) < 600, 'pin hole edit failed'
p[1] = l2
log.append('L2_base: 3 dowel holes changed from Ø3.1 on Ø24.5 to Ø4 on Ø24 (Damiao 4340P pins, Ø4x20); screw holes unchanged (6x M3 on Ø27)')

# ---------------- motor ----------------
m = import_step(MOTOR)          # local: axis +Y, output end at y=0, front face y=1.75, rear y=56.5, XT30 at local ±Z
m = m.rotate(Axis.X, -90)       # local y -> world -z
m = m.moved(Location(Plane(origin=(J1_X, J1_Y, 1.75),
                           x_dir=(math.cos(math.radians(ROT)), math.sin(math.radians(ROT)), 0), z_dir=(0, 0, 1))))
m = Compound(m.solids())
m = align_rotor(m, (J1_X, J1_Y, 1.75), (0, 0, -1), -30.0)   # output holes onto the L2_base pattern (pins 0/120/240)
m.label = 'DM4340P-J1'
parts = [p for p in parts if p[0] != 'RS00-L1']
parts.append(['DM4340P-J1', m])
log.append('RS00-L1 -> DM-J4340P-2EC (official model), front face on the plate underside, connectors toward the windows')

# ---------------- lift everything above J1 ----------------
FIXED = {'L1_base', '6707', 'DM4340P-J1'}
for p in parts:
    if p[0] not in FIXED: p[1] = p[1].moved(Location((0, 0, LIFT)))
log.append(f'all parts above J1 lifted {LIFT:+.2f} mm')

# ---------------- export ----------------
kids = [Compound([s], label=n) for n, s in parts]
export_step(Compound(children=kids, label='magarm_step1_J1'), OUT + '/magarm_step1_J1.step')
export_step(get('L1_base')[1], OUT + '/L1_base.step')
export_step(get('L2_base')[1], OUT + '/L2_base.step')
export_stl(get('L2_base')[1], OUT + '/L2_base.stl', tolerance=0.02, angular_tolerance=0.1)
export_stl(get('L1_base')[1], OUT + '/L1_base.stl', tolerance=0.02, angular_tolerance=0.1)
open(OUT + '/step1_log.txt', 'w').write('\n'.join(log))
print('\n'.join(log))
