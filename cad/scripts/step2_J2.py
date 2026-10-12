"""Step 2 (J2 + upper arm): Damiao DM-JH17-101-2EC on J2 in a shoulder housing, DM-J4340P on J3.

Input: assembly after step 1 (new base and J1 motor, everything else lifted 0.75 mm).
J2: the fork (old L2_base, L2_cover), its motor and the J2 bearing 6705 are dropped. The JH17 sits in a new shoulder
housing on the J1 output and the upper arm is bolted straight to its output flange, beside the housing: the
crossed-roller bearing of the motor carries the arm. The arm itself stays where it was; the J2 axis is unchanged.
  L2_base (new): shell Ø90 around the motor with a front ring 8 thick (8x M3x12 through the ring into the stator,
           heads sunk; the output flange stands 3 mm proud), turntable Ø70 x 15 on the J1 output with a pedestal up
           to the shell. 8x M3x25 into the J1 flange (9 under the head): four in the floor of the bore, four on the
           turntable under the arm hub, so the shoulder goes onto J1 before the J2 motor and the arm are fitted.
           4 sockets for the J1 pins, two windows at the J2 connectors (±X).
  L2_cover (new): rear lid, 4 posts onto the M4 holes of the motor rear cover (M4x12, cover only, no load);
           13 mm behind the motor for the cable that goes through the Ø15 bore of J2 into the arm.
  L3-motor: J2 end closed into a hub 8 thick (2.5 boss outside, 1.5 inside): 8x Ø3.4 on Ø54 for M3x25 from inside
           the arm, 4 pin sockets, Ø16 cable hole. The boss keeps the housing 1.2 mm clear of the folded forearm.
  L3_back: J2 end unchanged (the bearing spigot stays, unused).
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
# shoulder (JH17 model: axis +Y, output face y=30, body front y=19, body rear y=-21, rear end y=-40)
J1X, J1Y, Z_OUT1 = -9.0, -40.875, 16.75   # J1 axis and output face (step 1)
BOSS, HUB_IN = 2.5, 1.5            # arm hub: boss outside the arm face, thickening inside (wall 4 -> 8)
Y_OUT2 = Y_ARM - BOSS              # J2 output face
RING = 8.0
Y_RF, Y_RB = Y_OUT2 - 11.0 + RING, Y_OUT2 - 11.0      # front ring of the housing (J1 axis under its middle)
Y_REAR = Y_OUT2 - 70.0             # rear end of the motor
Y_END = Y_REAR - 13.0              # end of the shell
LID = 3.0
R_OUT, R_IN, R_OPEN = 45.0, 41.0, 32.0      # shell, bore for the Ø80 body, opening for the Ø62 flange
R_TT, Z_TT = 35.0, AZ - 39.0       # turntable; its top clears the arm hub (r 37.5) by 1.5
Z_CB = Z_OUT1 + 9.0                # seat of the J1 screw heads
Y_CONN = Y_OUT2 - 30.0 - 35.2      # J2 connectors

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

# ---------------- J2: shoulder housing on the J1 output ----------------
assert abs((Y_RF + Y_RB) / 2 - J1Y) < 1e-6, 'J1 axis must sit under the middle of the front ring'
parts = [p for p in parts if p[0] not in ('L2_base', 'L2_cover', 'RS00-L2')]
parts = [p for p in parts if not (p[0] == '6705' and p[1].bounding_box().center().X < 50)]
b = ycyl(R_OUT, Y_END, Y_RF)
b = chamfer(b.edges().filter_by(GeomType.CIRCLE), 1.0)
b += zcyl(R_TT, Z_OUT1, Z_TT)
b += zcyl(R_TT, Z_OUT1, AZ) & half('Y', Y_RF, False)                 # pedestal under the shell
b -= ycyl(R_IN, Y_END - 1, Y_RB)
b -= ycyl(R_OPEN, Y_RB - 1, Y_RF + 1)
for k in range(8):               # motor screws: M3x12 from the front, heads sunk 3.5 (4.5 of ring under the head)
    x, z = J2X + 37.0 * math.cos(k * math.pi / 4), AZ + 37.0 * math.sin(k * math.pi / 4)
    b -= ycyl(1.7, Y_RB - 1, Y_RF + 1, x, z)
    b -= ycyl(3.25, Y_RF - 3.5, Y_RF + 1, x, z)
b -= ycyl(3.6, Y_RF - 3.5, J1Y + R_TT + 1, J2X, AZ - 37.0)           # the lowest screw goes in along a groove in the turntable
for ang in (30, 60, 120, 150, 210, 240, 300, 330):                   # J1 output screws, clear of the ring above
    x, y = J1X + 27.0 * math.cos(math.radians(ang)), J1Y + 27.0 * math.sin(math.radians(ang))
    b -= zcyl(1.7, Z_OUT1 - 1, Z_CB + 1, x, y)
    b -= zcyl(3.25, Z_CB, AZ if y < J1Y else Z_TT + 1, x, y)
for ang in (0, 90, 180, 270):                                        # J1 output pins (Ø3, 2 proud)
    b -= zcyl(1.6, Z_OUT1 - 1, Z_OUT1 + 3.0, J1X + 27.0 * math.cos(math.radians(ang)), J1Y + 27.0 * math.sin(math.radians(ang)))
for sx in (-1, 1):                                                   # windows at the XT30 connectors
    w = fillet(Box(14, 22, 20).edges().filter_by(Axis.X), 4.0)
    b -= w.moved(Location((J2X + sx * (R_OUT + R_IN) / 2, Y_CONN, AZ)))
b = solid(b, 'L2_base')
parts.append(['L2_base', b])
log.append(f'L2_base: new shoulder housing, shell Ø{2 * R_OUT:.0f} y {Y_END}..{Y_RF}, ring {RING:.0f} with 8x M3 on Ø74, '
           f'turntable Ø{2 * R_TT:.0f} x {Z_TT - Z_OUT1:.0f} with 8x M3 on Ø54 + 4 pin sockets, {b.volume / 1000:.0f} cm3')

c = ycyl(R_OUT, Y_END - LID, Y_END)
c = chamfer(c.edges().filter_by(GeomType.CIRCLE).filter_by(lambda e: e.center().Y < Y_END - LID + 0.01), 1.0)
c += ycyl(R_IN - 0.3, Y_END, Y_END + 2.0) - ycyl(R_IN - 2.5, Y_END - 1, Y_END + 3.0)      # centring lip in the bore
for ang in (52.5, 127.5, -52.5, -127.5):                             # posts onto the 4x M4 (4.5 deep) of the rear cover
    x, z = J2X + 30.5 * math.cos(math.radians(ang)), AZ + 30.5 * math.sin(math.radians(ang))
    c += ycyl(6.0, Y_END - 0.01, Y_REAR, x, z)
    c -= ycyl(2.2, Y_END - LID - 1, Y_REAR + 1, x, z)
    c -= ycyl(3.8, Y_END - LID - 1, Y_REAR - 8.0, x, z)              # M4x12: 8 under the head, 4 in the thread
c = solid(c, 'L2_cover')
parts.append(['L2_cover', c])
log.append(f'L2_cover: new rear lid Ø{2 * R_OUT:.0f} x {LID:.0f} with 4 posts onto the M4 holes of the motor rear cover, {c.volume / 1000:.0f} cm3')

# ---------------- upper arm ----------------
p = get('L3-motor'); m3 = p[1].moved(Location((0, -D_ARM, 0)))
# Ø39.5 x 2.5 counterbore at the J3 motor seat for the rotating Ø38 output flange
m3 = m3 - Cylinder(19.75, 2.5, rotation=(90, 0, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((J3X, SEAT + 0.01, AZ)))
# J2 end: the RS00 seat (opening, 6 holes) closed into a hub on the JH17 output flange
v0 = m3.volume
m3 = m3 + ycyl(33.5, Y_ARM, SEAT + HUB_IN) + ycyl(R_OPEN, Y_OUT2, Y_ARM + 0.5)
m3 -= ycyl(8.0, Y_OUT2 - 1, SEAT + HUB_IN + 1)
for ang in (15, 75, 105, 165, -15, -75, -105, -165):                 # M3x25 from inside the arm: 8 of hub + 10 + thread
    m3 -= ycyl(1.7, Y_OUT2 - 1, SEAT + HUB_IN + 1, J2X + 27.0 * math.cos(math.radians(ang)), AZ + 27.0 * math.sin(math.radians(ang)))
for ang in (45, 135, -45, -135):                                     # output pins (Ø3, 2 proud)
    m3 -= ycyl(1.6, Y_OUT2 - 1, Y_OUT2 + 3.0, J2X + 27.0 * math.cos(math.radians(ang)), AZ + 27.0 * math.sin(math.radians(ang)))
m3 = solid(m3, 'L3-motor')
p[1] = m3; log.append(f'L3-motor: -{D_ARM}, Ø39.5x2.5 counterbore at the J3 seat; J2 end closed into a hub {BOSS + 4 + HUB_IN:.0f} thick '
                      f'(boss Ø{2 * R_OPEN:.0f} x {BOSS}, 8x Ø3.4 on Ø54, 4 pin sockets, Ø16 cable hole; {m3.volume - v0:+.0f} mm3)')
p = get('L3_back'); p[1] = stretch(p[1], 'Y', 10.25, 2 * D_ARM).moved(Location((0, -D_ARM, 0)))
log.append(f'L3_back: +{2*D_ARM} at y=10.25 (constant zone 5.25..15.25), then -{D_ARM}: outer face +{D_ARM}')
move('6705', +D_ARM)
log.append(f'6705 at J3 (pressed in L3_back): +{D_ARM}; the J2 one is dropped')

# ---------------- forearm (J3 fork) widened symmetrically ----------------
p = get('L4_right'); r = stretch(p[1], 'Y', -11.0, -D_FORK)          # constant zone -16.75..-6.0
p[1] = pins(r, J3X, Y_OUT, (20, 140, 260), 11.2)   # through, like the original pin holes
p = get('L4_left'); p[1] = stretch(p[1], 'Y', 13.0, +D_FORK)         # constant zone 10.2..16.95
move('6704', +D_FORK)
log.append(f'L4_right: -{D_FORK} at y=-11 (constant zone), J3 dowels Ø4 on Ø24; L4_left: +{D_FORK} at y=13 (constant zone); 6704 +{D_FORK}')
log.append('wrist kept centred on the mid-plane (gaps of 2.625 at J4 are closed in the J4 step)')

# ---------------- motors ----------------
m = Compound(import_step(MOTOR_J2).solids()).moved(Location((J2X, Y_OUT2 - 30.0, AZ)))     # connectors at ±X
m = Compound(m.solids()); m.label = 'DM-JH17-J2'
parts.append(['DM-JH17-J2', m])
log.append(f'RS00 J2 -> DM-JH17-101-2EC (official) in the shoulder housing, output face y={Y_OUT2}')
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
for n in ('L2_base', 'L2_cover', 'L3-motor', 'L3_back', 'L4_right', 'L4_left'):
    export_step(get(n)[1], f'{OUT}/{n}.step'); export_stl(get(n)[1], f'{OUT}/{n}.stl', tolerance=0.02, angular_tolerance=0.1)
open(OUT + '/step2_log.txt', 'w').write('\n'.join(log))
print('\n'.join(log))
