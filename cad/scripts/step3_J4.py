"""Step 3 (J4): Damiao DM-J4340 (Ø57) instead of RobStride EL05 (Ø46.5) in the wrist pitch joint, "drum" layout.

Input: assembly after step 2 (magarm_step2_J2.step).
Layout (agreed with the user: drum housing, forearm height unchanged):
  * J4 housing becomes a drum Ø63.1 (bore Ø57.2, wall 2.95 as the original), 55.25 long, centred between the fork
    faces (±28.625) with 1.0 mm gaps on both sides. The motor is held by its 4 rear M3 screws (Ø38, countersunk, the
    same positions as the EL05 rear screws); the output side is open toward L4_right, the output flange bolts
    straight onto the L4_right plate face (6x M3 Ø27 + 3 dowels Ø4 Ø24).
  * The housing keeps the base concept: L5_motor (left wall, J5 plate, journal, rear wall half) + L5_back (roof, right
    half of the drum), split on the same plane through the J4 axis as the original. J5 plate, left wall, ears and the
    6704 journal are the original geometry, moved; the drum and the roof (tangent to the drum) are new.
    The drum is wider than the J5 arm (±27.625 vs ±25): the plate and L5_shake keep their original width.
  * Forearm unchanged except the motor zone: the L4_right hub is redone for the DM4340 output and the concave face of
    the crossbar (both halves) is re-cut on the drum radius. J4 axis and the 6704 seat stay where they are.
  * J5 unit with the gripper moves 5.35 mm further out (the housing grows 5.35 toward it).
Usage: python step3_J4.py <magarm_step2_J2.step> <DM-J4340-2EC.stp> <out_dir>
"""
import sys, os, math
from build123d import *

SRC, MOTOR, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
AX0, AZ = -4.0, 134.75             # original J4 axis (along Y)
RI0, RO0 = 23.25, 26.2             # original housing bore / outer radius
RI, RO = 28.6, 31.55               # new drum: DM4340 r 28.5 + 0.1, same 2.95 wall
DX = round(RO - RO0, 3)            # 5.35: the housing grows this much around the (unchanged) J4 axis
AX = AX0                           # J4 axis stays: forearm length unchanged
SX = -DX                           # J5 unit shift (housing grows DX toward it)
YF = 28.625                        # fork faces ±YF (after step 2)
YH = YF - 1.0                      # drum ends ±27.625 (1.0 gaps)
T = RO - RI                        # 2.95 wall
YR = YH - T                        # 24.675: rear wall inner face = motor rear face
Y_OUT = -YF                        # motor output face on the L4_right plate face
PART_ANG = 127.0                   # L5_motor / L5_back parting plane through the J4 axis (original)
ROT_M = 53.0                       # motor turn about +Y: rear holes from 45° to -8° (the original rear screw angles)

a = import_step(SRC)
parts = [[c.label, c] for c in a.children]
def get(label, i=0): return [p for p in parts if p[0] == label][i]
log = []
BIG = 2000
V = lambda *v: Vector(*v)
def half(axis, pos, plus):
    d = {'X': (1, 0, 0), 'Y': (0, 1, 0), 'Z': (0, 0, 1)}[axis]
    size = [4 * BIG] * 3; size['XYZ'.index(axis)] = BIG
    return Box(*size).moved(Location(V(*d) * (pos + (BIG / 2 if plus else -BIG / 2))))
def fuse(*s):
    r = s[0].fuse(*s[1:], tol=0.001)
    return r if isinstance(r, Solid) or len(r.solids()) != 1 else r.solids()[0]
def ycyl(r, y0, y1, x=AX, z=AZ):
    """cylinder along Y from y0 to y1"""
    return Solid.make_cylinder(r, y1 - y0, Plane(origin=(x, y0, z), z_dir=(0, 1, 0)))
def yprism(pts, y0, y1):
    """polygon given as (x, z) points, extruded along Y from y0 to y1"""
    f = Face(Wire.make_polygon([V(x, y0, z) for x, z in pts], close=True))
    return extrude(f, amount=y1 - y0, dir=(0, 1, 0))
def check(shape, name):
    assert shape.is_valid and len(shape.solids()) == 1, f'{name}: invalid or {len(shape.solids())} solids'
    return shape

# ---------------- motor ----------------
m = import_step(MOTOR)              # local axis +Y, output face y=-0.05, rear face y=53.25, Ø57
m = m.rotate(Axis.Y, ROT_M).moved(Location((AX, Y_OUT + 0.05, AZ)))
motor = Compound(m.solids()); motor.label = 'DM4340-J4'
def motor_holes(dia, pcd):
    """world angles (atan2(z, x) about the new axis) of the motor's Y-axis holes of this size"""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    out = set()
    for f in motor.faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() != GeomAbs_Cylinder: continue
        c = s.Cylinder(); d = c.Axis().Direction(); o = c.Axis().Location()
        if abs(abs(d.Y()) - 1) > 1e-3 or abs(2 * c.Radius() - dia) > 0.05: continue
        r = math.hypot(o.X() - AX, o.Z() - AZ)
        if abs(2 * r - pcd) < 0.3: out.add(round(math.degrees(math.atan2(o.Z() - AZ, o.X() - AX)) % 360, 1))
    return sorted(out)
OUT_SCREWS, OUT_PINS, REAR = motor_holes(2.5, 27.0), motor_holes(4.0, 24.0), motor_holes(2.5, 38.0)
assert len(OUT_SCREWS) == 6 and len(OUT_PINS) == 3 and len(REAR) == 4, (OUT_SCREWS, OUT_PINS, REAR)
log.append(f'EL05_L4 -> DM-J4340-2EC (official), axis x={AX}, output face y={Y_OUT}, rear face y={YR}; '
           f'output screws {OUT_SCREWS}, pins {OUT_PINS}, rear screws {REAR} (deg)')

# ---------------- forearm: only the motor zone changes ----------------
R_HUB = 17.5                       # L4_right hub = Ø35 output disc of the DM4340
# L4_right: the EL05 hub (boss + holes, r < 17.5) replaced by a flat full-thickness disk with the DM4340 pattern
p = get('L4_right')
r = fuse(p[1] - ycyl(R_HUB, -60, 60), ycyl(R_HUB, -YF - 7.0, -YF))
for ang in OUT_SCREWS:                # M3 through + counterbore Ø6.5x3.5 from outside
    a_ = math.radians(ang); x, z = AX + 13.5 * math.cos(a_), AZ + 13.5 * math.sin(a_)
    r = r - ycyl(1.7, -YF - 7.1, -YF + 0.1, x, z) - ycyl(3.25, -YF - 7.1, -YF - 7.0 + 3.5, x, z)
for ang in OUT_PINS:                  # dowel Ø4 through
    a_ = math.radians(ang)
    r = r - ycyl(2.0, -YF - 7.1, -YF + 0.1, AX + 12.0 * math.cos(a_), AZ + 12.0 * math.sin(a_))
p[1] = check(r, 'L4_right')
log.append('L4_right: EL05 hub replaced by a flat disk Ø35 with 6x M3 Ø27 (cb Ø6.5x3.5) + 3x Ø4 Ø24 for the DM4340 output')
# the crossbar between the fork faces hugs the housing: its concave face re-cut on the drum radius (+0.5 gap);
# the rest of L4_right/L4_left (and the 6704 seat) is unchanged
for nm in ('L4_right', 'L4_left'):
    p = get(nm); v0 = p[1].volume
    p[1] = check(p[1] - ycyl(RO + 0.5, -YF + 0.01, YF - 0.01), nm)
    log.append(f'{nm}: crossbar concave face re-cut to r {RO + 0.5} ({v0 - p[1].volume:.0f} mm3 removed)')

# ---------------- J5 unit moves out ----------------
ARM = {'L1_base', 'L2_base', 'L2_cover', 'L3-motor', 'L3_back', 'L4_right', 'L4_left', '6704', '6705',
       'DM-JH17-J1', 'DM-JH17-J2', 'DM4340P-J3', 'L5_motor', 'L5_back', 'EL05_L4'}
moved = []
for p in parts:
    if p[0] not in ARM:
        p[1] = p[1].moved(Location((SX, 0, 0))); moved.append(p[0])
# Part_27 (a plate between the drum and L6_link, 0.2 mm from both) locks J5 from ±15°: removed from the arm
parts = [p for p in parts if p[0] != 'Part_27']
log.append(f'J5 unit (L5_shake, J5/J6 motors, L6_link, gripper, camera: {len(moved) - 1} parts) moved {SX} in X; '
           f'Part_27 removed (it blocked J5)')

# ---------------- housing ----------------
L5m, L5b = get('L5_motor')[1], get('L5_back')[1]
# original pieces kept: left wall + J5 plate (+ ears), moved with the J5 unit
A_m = L5m & fuse(Box(200, 200, 200, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Location((AX0 - RI0, 0, AZ))),
                 Box(200, 200, 100, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Location((-18.7, 0, 166.75))))
A_b = L5b & Box(200, 200, 100, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Location((-18.7, 0, 169.7)))
A_m, A_b = A_m.moved(Location((SX, 0, 0))), A_b.moved(Location((SX, 0, 0)))
journal = (L5m & ycyl(11.5, 25.0, 40.0, AX0)).moved(Location((0, YH - 25.0, 0)))
# new drum + roof profile (XZ), outer O and inner I = O offset by the wall
XW = AX0 - RO0 + SX                  # left wall outer face (moved with the plate)
ZT, XP, ZP = 169.75, -18.7 + SX, 172.75   # top of the body under the ear plate, roof start (plate end) and its top
def tangent(px, pz, R):
    dx, dz = px - AX, pz - AZ; d = math.hypot(dx, dz)
    ang = math.atan2(dz, dx) - math.acos(R / d)
    return ang
ANG_T = tangent(XP, ZP, RO)
T_O = (AX + RO * math.cos(ANG_T), AZ + RO * math.sin(ANG_T))
T_I = (AX + RI * math.cos(ANG_T), AZ + RI * math.sin(ANG_T))
nx, nz = math.cos(ANG_T), math.sin(ANG_T)        # roof normal (outward); inner roof line: n.(p-C) = RI
x_ri = AX + (RI - nz * (ZT - T - AZ)) / nx       # inner roof meets the inner top z = ZT - T
O_pts = [(XW, AZ), (XW, ZT), (XP, ZT), (XP, ZP), T_O, (AX, AZ)]
I_pts = [(XW + T, AZ), (XW + T, ZT - T), (x_ri, ZT - T), T_I, (AX, AZ)]
outer = fuse(ycyl(RO, -YH, YH), yprism(O_pts, -YH, YH))
cav = fuse(ycyl(RI, -YH - 1, YR), yprism(I_pts, -YH + T, YR))
body = check(outer - cav, 'body')
# rear wall: 4 countersunk M3 for the motor rear screws
for ang in REAR:
    a_ = math.radians(ang); x, z = AX + 19.0 * math.cos(a_), AZ + 19.0 * math.sin(a_)
    body = body - ycyl(1.7, YR - 0.1, YH + 0.1, x, z)
    body = body - Solid.make_cone(1.7, 3.4, 1.7, Plane(origin=(x, YH - 1.7, z), z_dir=(0, 1, 0)))
# split like the original: plane through the axis at PART_ANG, the journal boss stays in L5_motor
# half-space on the L5_back side (angles PART_ANG-180 .. PART_ANG); rotation about +Y lowers atan2(z, x) angles
Hb = Box(400, 400, 400, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(Location((AX, 0, AZ))) \
    .rotate(Axis((AX, 0, AZ), (0, 1, 0)), -(PART_ANG - 90))
boss = ycyl(11.0, YR - 0.1, YH + 0.1)
back = check(fuse(body & Hb, A_b) - boss, 'L5_back')
motor_half = check(fuse(body - Hb, body & Hb & boss, A_m, journal), 'L5_motor')
# clearance for the J6 motor sweeping about the J5 axis (±90°): concentric relief R 24.5 in the drum side only
J5X = -53.2 + SX
relief = Solid.make_cylinder(24.5, 48.0, Plane(origin=(J5X, 0, AZ - 23.5), z_dir=(0, 0, 1))) \
    & Box(10, 200, 100, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(Location((AX - RO - 1.0, 0, AZ)))
v0 = motor_half.volume
motor_half = check(motor_half - relief, 'L5_motor')
log.append(f'L5_motor: J6-motor sweep relief R24.5 about the J5 axis ({v0 - motor_half.volume:.0f} mm3), J5 free ±90°')
get('L5_motor')[1] = motor_half; get('L5_back')[1] = back
parts = [p for p in parts if p[0] != 'EL05_L4'] + [['DM4340-J4', motor]]
log.append(f'L5_motor/L5_back: drum Ø{2*RO:.1f} (bore Ø{2*RI:.1f}) y ±{YH}, roof tangent to the drum from the plate end, '
           f'4x M3 countersunk rear screws at r 19; J5 plate, left wall, ears and 6704 journal from the original (moved)')

# ---------------- export ----------------
kids = [Compound([s], label=nm) for nm, s in parts]
export_step(Compound(children=kids, label='magarm_step3_J4'), OUT + '/magarm_step3_J4.step')
for nm in ('L4_right', 'L4_left', 'L5_motor', 'L5_back'):
    export_step(get(nm)[1], f'{OUT}/{nm}.step'); export_stl(get(nm)[1], f'{OUT}/{nm}.stl', tolerance=0.02, angular_tolerance=0.1)
open(OUT + '/step3_log.txt', 'w').write('\n'.join(log))
print('\n'.join(log))
