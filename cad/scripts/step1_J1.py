"""Step 1 (J1): new base for the Damiao DM-JH17-101-2EC harmonic joint motor.

Start: the base assembly (cad/base/base_assembly.step). The original base, its J1 motor and the 6707 bearing are
dropped: the JH17 carries the arm on its own crossed-roller bearing.
Numbers (official Damiao model DM-JH17-2EC, axis +Y, output face at y = 30):
  body Ø80 x 40 (y -21..19), output flange Ø62 11 proud (y 19..30), rear cover Ø70 19 long (y -40..-21)
  stator: 8x M3 on Ø74, 10.5 deep, in the front face of the body, at 0/45/90...
  output: 8x Ø3.2 x 10 + M3 x 7.5 on Ø54 at ±15/±75/±105/±165, 4x Ø3 pins on Ø54 at ±45/±135 (2 proud)
  two XT30(2+2) on the rear cover, radial, 180° apart, 54.2 behind the front face of the body
New L1_base: foot 95 x 95 x 7 with the old hole pattern (4x Ø6.5 on 75 x 75), tower Ø92 with a top ring 8 thick.
The motor goes in from below and is screwed to the underside of the ring through it (8x M3x12, heads sunk in the
ring); the output flange stands 3 mm above the ring. Two windows in the tower face the connectors (±X).
The J1 axis is at (-9, 0): under the J2 axis, in the mid-plane of the arm (the shoulder fork of step 2 stands on it).
The J1 output face is 61 below the J2 axis: fork floor 15 + the Ø88.8 drum of the arm with the J2 motor in it.
Everything else is lifted 0.75 mm, as in the previous revision: the later steps use those arm coordinates.
The teleoperation handles of the base assembly (handle, handle_m) are dropped.
Usage: python step1_J1.py <base_assembly.step> <DM-JH17-2EC.stp> <out_dir>
"""
import sys, os, math
from build123d import *

SRC, MOTOR, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
J1X, J1Y = -9.0, 0.0             # J1 axis (vertical), in the mid-plane of the arm
Z_OUT = 9.75                     # J1 output face: 61 below the J2 axis (z = 70.75)
Z_TABLE = Z_OUT - 82.75
RING = 8.0                       # top ring; the output flange (11 proud of the body) stands 3 above it
Z_TOP = Z_OUT - 11.0 + RING      # ring top
R_OUT, R_IN, R_OPEN = 46.0, 41.0, 32.0      # tower, bore for the Ø80 body, opening for the Ø62 flange
FOOT, FOOT_T, FOOT_HOLES = 95.0, 7.0, 37.5
WIN_Y, WIN_Z = 24.0, 20.0        # connector windows
Z_CONN = Z_TOP - RING - 54.2     # connector centre
LIFT = 0.75

a = import_step(SRC)
parts = [[c.label, c] for c in a.children]
log = []

def zcyl(r, z0, z1, x=J1X, y=J1Y):
    return Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, z0)))

def turn_output(motor, deg):
    """The official STEP has the output at an arbitrary angle. Turn the solids of the output flange (inside r 31.6,
    reaching the front) about the motor axis (+Y), as the real output is turned by hand at assembly."""
    out = []
    for s in motor.solids():
        b = s.bounding_box()
        rot = max(abs(b.min.X), abs(b.max.X), abs(b.min.Z), abs(b.max.Z)) <= 31.6 and b.max.Y >= 20.5
        out.append(s.rotate(Axis.Y, deg) if rot else s)
    return Compound(out)

# ---------------- L1_base ----------------
foot = Box(FOOT, FOOT, FOOT_T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((J1X, J1Y, Z_TABLE)))
foot = fillet(foot.edges().filter_by(Axis.Z), 8.0)
b = foot + zcyl(R_OUT, Z_TABLE, Z_TOP)
b = fillet(b.edges().filter_by(GeomType.CIRCLE).filter_by(lambda e: abs(e.radius - R_OUT) < 0.01
                                                             and abs(e.center().Z - (Z_TABLE + FOOT_T)) < 0.01), 2.0)
b = chamfer(b.edges().filter_by(GeomType.CIRCLE).filter_by(lambda e: abs(e.radius - R_OUT) < 0.01
                                                              and abs(e.center().Z - Z_TOP) < 0.01), 1.0)
b -= zcyl(R_IN, Z_TABLE - 1, Z_TOP - RING)
b -= zcyl(R_OPEN, Z_TOP - RING - 1, Z_TOP + 1)
for sx in (-1, 1):
    for sy in (-1, 1):
        b -= zcyl(3.25, Z_TABLE - 1, Z_TABLE + FOOT_T + 1, J1X + sx * FOOT_HOLES, J1Y + sy * FOOT_HOLES)
for k in range(8):               # motor screws: M3x12 from above, heads sunk 3.5 (4.5 of ring under the head)
    x, y = J1X + 37.0 * math.cos(k * math.pi / 4), J1Y + 37.0 * math.sin(k * math.pi / 4)
    b -= zcyl(1.7, Z_TOP - RING - 1, Z_TOP + 1, x, y)
    b -= zcyl(3.25, Z_TOP - 3.5, Z_TOP + 1, x, y)
for sx in (-1, 1):               # windows at the XT30 connectors
    w = Box(14, WIN_Y, WIN_Z)
    w = fillet(w.edges().filter_by(Axis.X), 4.0)
    b -= w.moved(Location((J1X + sx * (R_OUT + R_IN) / 2, J1Y, Z_CONN)))
assert b.is_valid and len(b.solids()) == 1, 'L1_base failed'
b = b.solids()[0]
log.append(f'L1_base: new, foot {FOOT:.0f} x {FOOT:.0f} x {FOOT_T:.0f}, tower Ø{2 * R_OUT:.0f} x {Z_TOP - Z_TABLE:.2f}, ring {RING:.0f} with '
           f'8x M3 on Ø74 (cb Ø6.5 x 3.5), windows {WIN_Y:.0f} x {WIN_Z:.0f} at ±X, {b.volume / 1000:.0f} cm3')

# ---------------- motor ----------------
m = Compound(import_step(MOTOR).solids())
m = turn_output(m, 45.0)         # output holes to ±30/±60/±120/±150 about J1, pins to 0/90/180/270 (shoulder pattern, step 2)
m = m.rotate(Axis.X, 90).moved(Location((J1X, J1Y, Z_OUT - 30.0)))      # axis up, connectors at ±X
m = Compound(m.solids()); m.label = 'DM-JH17-J1'
log.append('RS00-L1, 6707, the old base and the teleop handles dropped; DM-JH17-101-2EC (official model) under the ring, connectors toward the windows')

# ---------------- everything else keeps the arm height of the previous revision ----------------
parts = [p for p in parts if p[0] not in ('L1_base', '6707', 'RS00-L1', 'handle', 'handle_m')]
for p in parts: p[1] = p[1].moved(Location((0, 0, LIFT)))
parts += [['L1_base', b], ['DM-JH17-J1', m]]
log.append(f'all other parts lifted {LIFT:+.2f} mm (arm coordinates of the later steps)')

# ---------------- export ----------------
kids = [Compound([s], label=n) for n, s in parts]
export_step(Compound(children=kids, label='magarm_step1_J1'), OUT + '/magarm_step1_J1.step')
export_step(b, OUT + '/L1_base.step')
export_stl(b, OUT + '/L1_base.stl', tolerance=0.02, angular_tolerance=0.1)
open(OUT + '/step1_log.txt', 'w').write('\n'.join(log))
print('\n'.join(log))
