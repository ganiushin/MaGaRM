"""Step 2 (J2 + upper arm): Damiao DM-JH17-101-2EC on J2 inside the upper arm, DM-J4340P on J3.

Input: assembly after step 1 (new base and J1 motor, everything else lifted 0.75 mm).
J2 is built like J3: the motor sits in the end of the upper arm, the fork holds its output flange on one side and a
bearing on the other. The old fork (L2_base, L2_cover) and its motor are dropped.
  L3-motor, L3_back: the J2 end (drum Ø75, inside 59.25) becomes a drum Ø88.8 x 68.75 for the JH17 (body Ø80 x 40,
           rear cover Ø70 x 19): front wall 6 with the Ø64 opening for the output flange and 8x M3 countersunk on
           Ø74 into the stator, rear wall 3.5 with the spigot of the 6705 (Ø25.1; it was Ø25.6 for a Ø25 bore). The halves keep their lap joint.
           The two half-joining screws at the beam root sat where the new wall is: moved 3 mm outward.
           The central cable channel of the beam opens into the drum. The connectors of the motor are radial: the
           one facing the beam gets a window in L3_back at the beam root, the other one is closed by the drum
           (it would sweep through the fork floor).
  L2_base (new): floor 70 x 15 on the J1 output (8x M3x25 into the flange, 9 under the head, 4 sockets for the pins)
           with the motor-side cheek: pocket for the Ø62 output flange, 8x Ø3.4 on Ø54 (M3x25 from outside, 9 under
           the head), 4 pin sockets. 3 heat-set inserts M4 in the end of the floor for the other cheek.
  L2_cheek (new): plate 10 thick with a Ø32 seat for the 6705, 3x M4x16 with washers through Ø5.5 holes: the motor
           already holds the J2 axis, so the cheek is tightened last, where the bearing puts it (±0.75 mm), and
           does not bend the output of the reducer.
J3 and the arm width, as before (official Damiao model vs RobStride RS00):
  4340P: output flange 1.75 proud of the front face, body 54.75  ->  total 56.5
  RS00 : output flange 1.00,                         body 51.00  ->  total 52.0
  upper-arm halves (L3-motor / L3_back outer faces)  -/+ 1.875  (arm +3.75, the 4340P body difference)
  J3 fork sides (L4_right / L4_left)                 -/+ 2.625  (gaps 1.75 on both sides)
  Everything stays symmetric about the arm mid-plane (y = 2.25). Output dowel holes go from Ø3 on Ø24.5 (RS00)
  to Ø4 on Ø24 (Damiao). The wrist stays centred on the mid-plane; it is rebuilt in the J4 step.
Usage: python step2_J2.py <magarm_step1_J1.step> <DM-JH17-2EC.stp> <DM-J4340P-2EC.stp> <out_dir>
"""
import sys, os, math
from build123d import *

SRC, MOTOR_J2, MOTOR, OUT = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
os.makedirs(OUT, exist_ok=True)
AZ = 70.75                         # J2/J3 axes height (after the step-1 lift)
J2X, J3X = -9.0, 181.0
D_FORK = 2.625                     # each fork side
D_ARM = 1.875                      # each upper-arm half
Y_OUT = -26.5 - D_FORK             # output face of the J3 motor
SEAT = Y_OUT + 1.75                # 4340P front face (= L3-motor seat, inner face of its wall)
Y_ARM = -29.5 - D_ARM              # outer face of L3-motor
# J2 (JH17 model: axis +Y, output face y=30, body front y=19, body rear y=-21, rear end y=-40)
J1X, J1Y = -9.0, 0.0               # J1 axis (step 1)
R_BORE, R_DRUM, R_LAP = 40.4, 44.4, 42.4   # drum of the arm around the Ø80 body; radius where the lap joint splits the wall
WALL_F, WALL_B = 6.0, 3.5          # drum walls: under the stator screws, behind the motor
Y_F = -(WALL_F + 59.0 + 0.25 + WALL_B) / 2   # drum faces at -/+ 34.375 (were -/+ 31.375)
Y_B, Y_CAP = -Y_F, -Y_F - WALL_B   # rear wall: outer and inner face
SEAT2 = Y_F + WALL_F               # front face of the motor body
Y_OUT2 = SEAT2 - 11.0              # J2 output face
R_OPEN = 32.0                      # opening for the Ø62 flange
Y_SPLIT, Y_LIP = 1.125, -1.175     # lap joint of the halves (as in the original drum)
R_SPIGOT = 12.55                   # Ø25.1 for the Ø25 bore of the 6705 (the old spigot was Ø25.6)
ROOT_X, ROOT_DZ = 31.0, (15.0, 18.0)       # half-joining screws at the beam root: old and new distance from the axis height
# fork
Z_FL = AZ - R_DRUM - 1.6           # floor top, 1.6 under the drum
Z_OUT1 = Z_FL - 15.0               # J1 output face (step 1)
W_FL = 35.0                        # half-width of the floor and of the cheeks
Y_C1 = Y_F - 1.0                   # motor-side cheek: inner face 1 mm off the drum, the flange sits in a pocket
Y_C0 = Y_OUT2 - 9.0                # its outer face: 9 under the screw heads
Y_CH = Y_B + 1.0                   # bearing cheek: 1 mm off the drum
CH_T, R_CH = 10.0, 24.0
Y_BRG = Y_B + 4.5                  # outer face of the 6705 on the spigot (0.5 shoulder + 4)
CH_X, Z_CHS = (-27.0, 0.0, 27.0), Z_OUT1 + 7.5        # cheek screws
Z_CB = Z_OUT1 + 9.0                # seat of the J1 screw heads

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

def ycyl(r, y0, y1, x=J2X, z=AZ):
    return Cylinder(r, y1 - y0, rotation=(-90, 0, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y0, z)))
def zcyl(r, z0, z1, x=J1X, y=J1Y):
    return Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, z0)))
def solid(shape, name):
    assert shape.is_valid and len(shape.solids()) == 1, f'{name} failed'
    return shape.solids()[0]

def ycone(r0, r1, y0, y1, x, z):
    return Cone(r0, r1, y1 - y0, rotation=(-90, 0, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y0, z)))
def cheek(r, y0, y1):
    """hull of a disc on the J2 axis and the foot on the floor, drawn in (x, z) and stood up"""
    with BuildSketch() as sk:
        with Locations((J2X, AZ)): Circle(r)
        with Locations((J1X, (Z_OUT1 + Z_FL) / 2)): Rectangle(2 * W_FL, Z_FL - Z_OUT1)
        make_hull()
    return extrude(sk.sketch, amount=y1 - y0).rotate(Axis.X, 90).moved(Location((0, y1, 0)))

# ---------------- J2: shoulder fork on the J1 output ----------------
assert abs(Z_OUT1 - 9.75) < 1e-6, 'J1 output face: keep in step with step1_J1.py'
parts = [p for p in parts if p[0] not in ('L2_base', 'L2_cover', 'RS00-L2')]
k = cheek(W_FL, Y_C0, Y_C1)
k = chamfer(k.edges().filter_by(lambda e: abs(e.center().Y - Y_C0) < 0.01), 1.0)
b = Box(2 * W_FL, Y_CH - Y_C0 - 1.0, Z_FL - Z_OUT1, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(Location((J1X, Y_C0 + 1.0, Z_OUT1))) + k
b -= ycyl(31.4, Y_OUT2, Y_C1 + 1)                                    # pocket for the output flange (Ø62, 5 proud of the drum)
b -= ycyl(8.0, Y_C0 - 1, Y_OUT2 + 1)                                 # in line with the hollow shaft
for ang in (15, 75, 105, 165, -15, -75, -105, -165):                 # M3x25 from outside: 9 of cheek + 10 + thread
    b -= ycyl(1.7, Y_C0 - 1, Y_OUT2 + 1, J2X + 27.0 * math.cos(math.radians(ang)), AZ + 27.0 * math.sin(math.radians(ang)))
for ang in (45, 135, -45, -135):                                     # output pins (Ø3, 2 proud)
    b -= ycyl(1.6, Y_OUT2 - 3.0, Y_OUT2 + 1, J2X + 27.0 * math.cos(math.radians(ang)), AZ + 27.0 * math.sin(math.radians(ang)))
for ang in (30, 60, 120, 150, 210, 240, 300, 330):                   # J1 output screws, all in the open floor
    x, y = J1X + 27.0 * math.cos(math.radians(ang)), J1Y + 27.0 * math.sin(math.radians(ang))
    b -= zcyl(1.7, Z_OUT1 - 1, Z_CB + 1, x, y)
    b -= zcyl(3.25, Z_CB, Z_FL + 1, x, y)
for ang in (0, 90, 180, 270):                                        # J1 output pins (Ø3, 2 proud)
    b -= zcyl(1.6, Z_OUT1 - 1, Z_OUT1 + 3.0, J1X + 27.0 * math.cos(math.radians(ang)), J1Y + 27.0 * math.sin(math.radians(ang)))
for dx in CH_X:                                                      # heat-set inserts M4 (Ø5.6 x 8) for the bearing cheek
    b -= ycyl(2.8, Y_CH - 10.0, Y_CH + 1, J2X + dx, Z_CHS)
b = solid(b, 'L2_base')
parts.append(['L2_base', b])
log.append(f'L2_base: new shoulder fork, floor {2 * W_FL:.0f} x {Z_FL - Z_OUT1:.0f} y {Y_C0}..{Y_CH} with 8x M3 on Ø54 + 4 pin sockets for J1, '
           f'motor-side cheek {Y_C1 - Y_C0:.0f} thick with a Ø62.8 x {Y_C1 - Y_OUT2:.0f} pocket, 8x Ø3.4 on Ø54 and 4 pin sockets, '
           f'3 inserts M4 for the other cheek, {b.volume / 1000:.0f} cm3')

k = cheek(R_CH, Y_CH, Y_CH + CH_T)
k = chamfer(k.edges().filter_by(lambda e: abs(e.center().Y - (Y_CH + CH_T)) < 0.01), 1.0)
k -= ycyl(16.05, Y_CH - 1, Y_BRG)                                    # seat of the 6705 (Ø32 x 4), 3.5 of it in the cheek
k -= ycyl(14.5, Y_BRG - 1, Y_CH + CH_T + 1)                          # shoulder for the outer ring, the spigot end is free
for dx in CH_X:                                                      # M4x16 + washer: Ø5.5 holes let the cheek settle on the bearing
    k -= ycyl(2.75, Y_CH - 1, Y_CH + CH_T + 1, J2X + dx, Z_CHS)
    k -= ycyl(5.25, Y_CH + CH_T - 4.5, Y_CH + CH_T + 1, J2X + dx, Z_CHS)
k = solid(k, 'L2_cheek')
parts.append(['L2_cheek', k])
log.append(f'L2_cheek: new, {CH_T:.0f} thick at y {Y_CH}, Ø32.1 seat for the 6705, 3x M4 through Ø5.5 (cb Ø10.5 x 4.5), {k.volume / 1000:.0f} cm3')

# ---------------- upper arm ----------------
p3m, p3b = get('L3-motor'), get('L3_back')
m0 = p3m[1].moved(Location((0, -D_ARM, 0)))
b0 = stretch(p3b[1], 'Y', 10.25, 2 * D_ARM).moved(Location((0, -D_ARM, 0)))
log.append(f'L3-motor: -{D_ARM}; L3_back: +{2*D_ARM} at y=10.25 (constant zone 5.25..15.25), then -{D_ARM}: outer face +{D_ARM}')
# Ø39.5 x 2.5 counterbore at the J3 motor seat for the rotating Ø38 output flange
m3 = m0 - Cylinder(19.75, 2.5, rotation=(90, 0, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((J3X, SEAT + 0.01, AZ)))
b3 = b0
# J2 end: the old drum goes, the two screws at the beam root are plugged (re-made further out below)
for sz in (-1, 1):
    z = AZ + sz * ROOT_DZ[0]
    m3 += ycyl(3.3, -24.375, -22.8, ROOT_X, z) + ycyl(1.75, -22.9, -1.875, ROOT_X, z)
    b3 += ycyl(3.05, -1.675, -1.17, ROOT_X, z) + ycyl(2.45, -1.18, 18.55, ROOT_X, z)
m0p, b0p = m3, b3                # the halves as they were, holes plugged: the new walls stay out of each other's way
m3 -= ycyl(R_BORE, -60, 60)
b3 = b3 - ycyl(R_BORE, -60, Y_CAP) - ycyl(R_DRUM + 1, Y_CAP, 60)
# new drum, motor half: front wall and the wall down to the lap joint (outer lip)
d = ycyl(R_DRUM, Y_F, Y_SPLIT)
d = chamfer(d.edges().filter_by(GeomType.CIRCLE).filter_by(lambda e: e.center().Y < Y_F + 0.01), 2.0)
d = d - ycyl(R_BORE, SEAT2, Y_SPLIT + 1) - ycyl(R_OPEN, Y_F - 1, SEAT2 + 1) - ycyl(R_LAP + 0.05, Y_LIP, Y_SPLIT + 1)
m3 += d - b0p
# new drum, back half: wall from the lap joint (inner lip), rear wall, spigot of the 6705
d = ycyl(R_DRUM, Y_SPLIT, Y_B)
d = chamfer(d.edges().filter_by(GeomType.CIRCLE).filter_by(lambda e: e.center().Y > Y_B - 0.01), 2.0)
d += ycyl(R_LAP - 0.05, Y_LIP, Y_SPLIT)
d -= ycyl(R_BORE, Y_LIP - 1, Y_CAP)
sp = ycyl(R_SPIGOT, Y_B, Y_B + 4.5) - ycyl(8.8, Y_B, Y_B + 5.5)
sp = fillet(sp.edges().filter_by(GeomType.CIRCLE).filter_by(lambda e: e.center().Y > Y_B + 4.49), 1.0)
d += sp + (ycyl(14.3, Y_B, Y_B + 0.5) - ycyl(8.8, Y_B, Y_B + 1))
b3 += d - m0p
for k8 in range(8):              # stator screws: M3x12 countersunk from outside, 6 of wall + 6 in the thread
    x, z = J2X + 37.0 * math.cos(k8 * math.pi / 4), AZ + 37.0 * math.sin(k8 * math.pi / 4)
    m3 -= ycyl(1.7, Y_F - 1, SEAT2 + 1, x, z)
    m3 -= ycone(3.3, 1.7, Y_F - 0.01, Y_F + 1.6, x, z)
for sz in (-1, 1):               # the two screws at the beam root, same shape as before
    z = AZ + sz * ROOT_DZ[1]
    m3 -= ycyl(1.7, -22.83, Y_SPLIT + 1, ROOT_X, z)
    m3 -= ycone(3.25, 1.7, -24.385, -22.82, ROOT_X, z)
    b3 -= ycyl(3.0, -1.685, -1.17, ROOT_X, z)
    b3 -= ycyl(2.4, -1.18, 17.08, ROOT_X, z)
    b3 -= ycone(2.4, 0.01, 17.07, 18.52, ROOT_X, z)
ch = Box(9.0, Y_SPLIT + 4.875, 8.0, align=(Align.MIN, Align.MIN, Align.CENTER)).moved(Location((J2X + 38.0, -4.875, AZ)))
m3 -= ch; b3 -= ch               # the cable channel of the beam (6 x 8 at the split) opens into the drum
w = Box(48.0 - (J2X + 33.0), Y_CAP - 8.0, 18.0, align=(Align.MIN, Align.MIN, Align.CENTER))
w = fillet(w.edges().filter_by(Axis.Y), 2.0).moved(Location((J2X + 33.0, 8.0, AZ)))
b3 -= w                          # window at the motor connector that faces the beam (first two cells of the beam)
m3, b3 = solid(m3, 'L3-motor'), solid(b3, 'L3_back')
clash = (m3 & b3).volume if (m3 & b3) else 0.0
assert clash < 1.0, f'arm halves overlap: {clash:.1f} mm3'
p3m[1], p3b[1] = m3, b3
log.append(f'L3-motor / L3_back: J2 end rebuilt as a drum Ø{2 * R_DRUM} x {Y_B - Y_F} (bore Ø{2 * R_BORE}, front wall {WALL_F} with a Ø{2 * R_OPEN:.0f} opening '
           f'and 8x M3 csk on Ø74, rear wall {WALL_B} with the 6705 spigot), root screws moved to ±{ROOT_DZ[1]:.0f}, cable channel and '
           f'connector window; {m3.volume / 1000:.0f} / {b3.volume / 1000:.0f} cm3 (were {m0.volume / 1000:.0f} / {b0.volume / 1000:.0f})')
for q in parts:                  # 6705: the J3 one follows L3_back, the J2 one sits on the moved spigot
    if q[0] == '6705':
        j2 = q[1].bounding_box().center().X < 50
        q[1] = q[1].moved(Location((0, Y_B + 0.5 - 30.0 if j2 else D_ARM, 0)))
log.append(f'6705 at J3 (pressed in L3_back): +{D_ARM}; 6705 at J2 on the spigot, y {Y_B + 0.5}..{Y_B + 4.5}')

# ---------------- forearm (J3 fork) widened symmetrically ----------------
p = get('L4_right'); r = stretch(p[1], 'Y', -11.0, -D_FORK)          # constant zone -16.75..-6.0
p[1] = pins(r, J3X, Y_OUT, (20, 140, 260), 11.2)   # through, like the original pin holes
p = get('L4_left'); p[1] = stretch(p[1], 'Y', 13.0, +D_FORK)         # constant zone 10.2..16.95
move('6704', +D_FORK)
log.append(f'L4_right: -{D_FORK} at y=-11 (constant zone), J3 dowels Ø4 on Ø24; L4_left: +{D_FORK} at y=13 (constant zone); 6704 +{D_FORK}')
log.append('wrist kept centred on the mid-plane (gaps of 2.625 at J4 are closed in the J4 step)')

# ---------------- motors ----------------
m = Compound(import_step(MOTOR_J2).solids()).rotate(Axis.Z, 180).moved(Location((J2X, Y_OUT2 + 30.0, AZ)))   # output toward -Y, connectors at ±X
m = Compound(m.solids()); m.label = 'DM-JH17-J2'
for q in (m3, b3):
    v = (q & m).volume if (q & m) else 0.0
    assert v < 1.0, f'J2 motor cuts into the arm: {v:.1f} mm3'
parts.append(['DM-JH17-J2', m])
log.append(f'RS00 J2 -> DM-JH17-101-2EC (official) in the drum of the arm, output face y={Y_OUT2}')
m = import_step(MOTOR)   # local axis +Y, output end y=0, XT30 at local ±Z (top/bottom, like the RS00)
# rotor turned so its holes match the part (rotation about +Y lowers atan2(z, x) angles): pins 0 -> 20
mm = Compound(m.moved(Location((J3X, Y_OUT, AZ))).solids())
mm = align_rotor(mm, (J3X, Y_OUT, AZ), (0, 1, 0), -20.0); mm.label = 'DM4340P-J3'
parts = [p for p in parts if p[0] != 'RS00-L3']
parts.append(['DM4340P-J3', mm])
log.append(f'RS00 J3 -> DM-J4340P-2EC (official), output face y={Y_OUT}, connectors top/bottom')

# ---------------- export ----------------
kids = [Compound([s], label=n) for n, s in parts]
export_step(Compound(children=kids, label='magarm_step2_J2'), OUT + '/magarm_step2_J2.step')
for n in ('L2_base', 'L2_cheek', 'L3-motor', 'L3_back', 'L4_right', 'L4_left'):
    export_step(get(n)[1], f'{OUT}/{n}.step'); export_stl(get(n)[1], f'{OUT}/{n}.stl', tolerance=0.02, angular_tolerance=0.1)
open(OUT + '/step2_log.txt', 'w').write('\n'.join(log))
print('\n'.join(log))
