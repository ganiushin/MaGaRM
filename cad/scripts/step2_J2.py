"""Step 2 (J2 + upper arm): Damiao DM-J4340P on J2 and J3, symmetric widening about the arm mid-plane.

Input: assembly after step 1 (magarm_step1_J1.step, everything above J1 already lifted 0.75 mm).
Numbers (official Damiao model vs RobStride RS00):
  4340P: output flange 1.75 proud of the front face, body 54.75  ->  total 56.5
  RS00 : output flange 1.00,                         body 51.00  ->  total 52.0
  The output-side fork gap grows by 0.75 (1.0 -> 1.75) whatever we do, so the other side is set to 1.75 too.
  Everything moves symmetrically about the arm mid-plane (y = 2.25, unchanged):
    fork sides (L2_base cheek / L2_cover, L4_right / L4_left)  -/+ 2.625
    upper-arm halves (L3-motor / L3_back outer faces)          -/+ 1.875  (arm +3.75, the 4340P body difference)
Stator holes (6x M3 on Ø50 front, 4x M3 on Ø38 rear, 2 pins) and the XT30 positions (top/bottom) of the 4340P
match the RS00 exactly, so the upper-arm motor seats are not changed except a Ø39.5 counterbore for the
rotating Ø38 output flange. Output dowel holes go from Ø3 on Ø24.5 (RS00) to Ø4 on Ø24 (Damiao).
The wrist stays centred on the mid-plane; it is rebuilt in the J4 step.
Usage: python step2_J2.py <magarm_step1_J1.step> <DM-J4340P-2EC.stp> <out_dir>
"""
import sys, os, math
from build123d import *

SRC, MOTOR, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
AZ = 70.75                         # J2/J3 axes height (after the step-1 lift)
J2X, J3X = -9.0, 181.0
D_FORK = 2.625                     # each fork side
D_ARM = 1.875                      # each upper-arm half
Y_OUT = -26.5 - D_FORK             # new output face of the J2/J3 motors
SEAT = Y_OUT + 1.75                # 4340P front face (= L3-motor seat)

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
BIG = 2000
def half(axis, pos, plus):
    d = {'X': (1, 0, 0), 'Y': (0, 1, 0), 'Z': (0, 0, 1)}[axis]
    size = [4 * BIG] * 3; size['XYZ'.index(axis)] = BIG
    c = Vector(*d) * (pos + (BIG / 2 if plus else -BIG / 2))
    return Box(*size).moved(Location(c))
def stretch(shape, axis, pos, delta):
    """split at the plane, move the side the delta points to by delta, fill with the section (seamless in a prismatic zone)"""
    d = Vector(*{'X': (1, 0, 0), 'Y': (0, 1, 0), 'Z': (0, 0, 1)}[axis])
    keep = shape & half(axis, pos, delta < 0)
    move = shape & half(axis, pos, delta > 0)
    sec = shape & Plane(origin=d * pos, z_dir=d)
    fill = extrude(sec, amount=abs(delta), dir=d * (1 if delta > 0 else -1))
    # fuzzy fuse: the plain union fails on L3_back (tangent internal faces at the split plane)
    r = keep.fuse(fill, move.moved(Location(d * delta)), tol=0.001)
    exp = sec.area * abs(delta)
    assert r.is_valid and abs(r.volume - shape.volume - exp) < 0.01 * exp + 25, f'stretch failed at {axis}={pos}: {r.volume - shape.volume:.0f} vs {exp:.0f}'
    return r
def open_sym(shape, axis, pos, d):
    """split once at the plane, move both sides apart by -/+d, fill the 2d gap with the section (one boolean union)"""
    dv = Vector(*{'X': (1, 0, 0), 'Y': (0, 1, 0), 'Z': (0, 0, 1)}[axis])
    lo = shape & half(axis, pos, False); hi = shape & half(axis, pos, True)
    sec = shape & Plane(origin=dv * pos, z_dir=dv)
    fill = extrude(sec, amount=2 * d, dir=dv).moved(Location(dv * (-d)))
    r = lo.moved(Location(dv * (-d))) + fill + hi.moved(Location(dv * d))
    exp = sec.area * 2 * d
    assert r.is_valid and abs(r.volume - shape.volume - exp) < 0.01 * exp + 25, f'open failed at {axis}={pos}'
    return r
def move(label, dy, i=0):
    p = get(label, i); p[1] = p[1].moved(Location((0, dy, 0)))
def pins(shape, ax, y_face, angles, depth):
    """plug the RS00 Ø3 dowel holes (Ø24.5) and drill Ø4 on Ø24 from the output face toward -Y"""
    v0 = shape.volume
    for ang in angles:
        a_ = math.radians(ang)
        shape = shape + Cylinder(1.75, depth, rotation=(90, 0, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)) \
            .moved(Location((ax + 12.25 * math.cos(a_), y_face, AZ + 12.25 * math.sin(a_))))
        shape = shape - Cylinder(2.0, depth, rotation=(90, 0, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)) \
            .moved(Location((ax + 12.0 * math.cos(a_), y_face + 0.01, AZ + 12.0 * math.sin(a_))))
    assert shape.is_valid and abs(shape.volume - v0) < 800, 'pin edit failed'
    return shape

# ---------------- J2 fork: L2_base widened symmetrically (cradle only, J1 spigot below z=14.75 untouched) ----
p = get('L2_base'); b = p[1]
# The cradle is turned about the J1 axis (no prismatic zone along Y), so it is split exactly through the J1 axis
# (y=0, tangent-continuous for a body of revolution) and opened symmetrically by -/+2.625.
# The J1 spigot/neck below z=14.75 stays round (bearing fit); J1 dowel holes above the split are re-drilled in place.
J1X, J1Y = 0.6, 0.0
up, low = b & half('Z', 14.75, True), b & half('Z', 14.75, False)
up = open_sym(up, 'Y', 0.3, D_FORK)
for ang in (0, 120, 240):                          # J1 dowel holes (Ø4 on Ø24, z 1.75..23.75) - plug the shifted copies, re-drill
    a_ = math.radians(ang); px, py = J1X + 12.0 * math.cos(a_), J1Y + 12.0 * math.sin(a_)
    for dy in (-D_FORK, 0.0, D_FORK):
        up = up + Cylinder(2.3, 9.4, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((px, py + dy, 14.74)))
    up = up + Box(4.6, 2 * D_FORK + 4.6, 9.4, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((px, py, 14.74))) if abs(py) < 2.5 else up
    up = up - Cylinder(2.0, 9.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((px, py, 14.7)))
b = up + low
b = pins(b, J2X, Y_OUT, (80, 200, 320), 11.0)
p[1] = b
log.append(f'L2_base: fork opened -/+{D_FORK} through the J1 axis (y=0) above z=14.75, J1 spigot untouched, J1 dowel holes re-drilled; J2 dowels Ø4 on Ø24')
move('L2_cover', +D_FORK); log.append(f'L2_cover: +{D_FORK} (no shape change)')

# ---------------- upper arm ----------------
p = get('L3-motor'); m3 = p[1].moved(Location((0, -D_ARM, 0)))
for ax in (J2X, J3X):   # Ø39.5 x 2.5 counterbore at each motor seat for the rotating Ø38 output flange
    m3 = m3 - Cylinder(19.75, 2.5, rotation=(90, 0, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((ax, SEAT + 0.01, AZ)))
p[1] = m3; log.append(f'L3-motor: -{D_ARM} (no shape change) + Ø39.5x2.5 counterbores at both motor seats')
p = get('L3_back'); p[1] = stretch(p[1], 'Y', 10.25, 2 * D_ARM).moved(Location((0, -D_ARM, 0)))
log.append(f'L3_back: +{2*D_ARM} at y=10.25 (constant zone 5.25..15.25), then -{D_ARM}: outer face +{D_ARM}')
for i in range(2): move('6705', +D_ARM, i)
log.append(f'6705 x2 (pressed in L3_back): +{D_ARM}')

# ---------------- forearm (J3 fork) widened symmetrically ----------------
p = get('L4_right'); r = stretch(p[1], 'Y', -11.0, -D_FORK)          # constant zone -16.75..-6.0
p[1] = pins(r, J3X, Y_OUT, (20, 140, 260), 11.2)   # through, like the original pin holes
p = get('L4_left'); p[1] = stretch(p[1], 'Y', 13.0, +D_FORK)         # constant zone 10.2..16.95
move('6704', +D_FORK)
log.append(f'L4_right: -{D_FORK} at y=-11 (constant zone), J3 dowels Ø4 on Ø24; L4_left: +{D_FORK} at y=13 (constant zone); 6704 +{D_FORK}')
log.append('wrist kept centred on the mid-plane (gaps of 2.625 at J4 are closed in the J4 step)')

# ---------------- motors ----------------
m = import_step(MOTOR)   # local axis +Y, output end y=0, XT30 at local ±Z (top/bottom, like the RS00)
# rotor turned so its holes match the part (rotation about +Y lowers atan2(z, x) angles): J2 pins 0 -> 80, J3 0 -> 20
for old, new, ax, rot in (('RS00-L2', 'DM4340P-J2', J2X, -80.0), ('RS00-L3', 'DM4340P-J3', J3X, -20.0)):
    mm = Compound(m.moved(Location((ax, Y_OUT, AZ))).solids())
    mm = align_rotor(mm, (ax, Y_OUT, AZ), (0, 1, 0), rot); mm.label = new
    parts = [p for p in parts if p[0] != old]
    parts.append([new, mm])
log.append(f'RS00 J2/J3 -> DM-J4340P-2EC (official), output face y={Y_OUT}, connectors top/bottom')

# ---------------- export ----------------
kids = [Compound([s], label=n) for n, s in parts]
export_step(Compound(children=kids, label='magarm_step2_J2'), OUT + '/magarm_step2_J2.step')
for n in ('L2_base', 'L3-motor', 'L3_back', 'L4_right', 'L4_left'):
    export_step(get(n)[1], f'{OUT}/{n}.step'); export_stl(get(n)[1], f'{OUT}/{n}.stl', tolerance=0.02, angular_tolerance=0.1)
open(OUT + '/step2_log.txt', 'w').write('\n'.join(log))
print('\n'.join(log))
