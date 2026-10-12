"""Step 4 (gripper): Damiao DM-H3510 instead of RobStride EL05 (EL05_L7) in the gripper.

Input: assembly after step 3 (magarm_step3_J4.step).
The EL05 was a structural member: L7_cover screwed to its rear, the jaw frame ('rack') to its front, the camera
clamp (Part_1/Part_2) around its Ø46 body. The H3510 (Ø42 x 29 + output disc, mounting holes only at the rear) is put
into a printed cup that takes over these jobs:
  * L7_motor (new): Ø46 tube like the EL05 body (the camera clamp fits unchanged), 4 mm bottom screwed to the
    L7_cover holes (M3 heat-set inserts), motor on the bottom with 4 countersunk M3 (inserted from the front), Ø60 front flange for
    the jaw frame (6x M3 countersunk into heat-set inserts in the frame), window for the motor connectors.
  * gripper mechanism (frame, crank, links, jaws, rods) 11 mm closer to the wrist (shorter motor);
    camera group moved 4.2 mm so its clamp sits on the tube in front of the connector window.
  * rack: ring OD 50 -> 60 for the inserts, the unused EL05 screw holes plugged.
  * crank: hub re-drilled for the H3510 output (6x M3 Ø16, counterbored), recess for the Ø7 centre boss.
Usage: python step4_gripper.py <magarm_step3_J4.step> <DM-H3510.stp> <out_dir>
"""
import sys, os, math
from build123d import *

SRC, MOTOR, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
ZC = 134.75                        # gripper axis (along X) at y = 0
X_COVER = -112.5                   # L7_cover front face (EL05 rear face)
BOT = 4.0                          # cup bottom
X_REAR = X_COVER - BOT             # motor rear face
X_OUT = X_REAR - 29.0              # motor output face (H3510: output 0, rear face -29)
DM = X_OUT - (-156.5)              # mechanism shift toward the wrist (EL05 output was at -156.5)
FL = 3.0                           # front flange thickness
R_TUBE, R_BORE, R_FL = 23.0, 21.2, 30.0
R_FS = 27.0                        # frame screws: countersunk heads clear of the Ø46 tube
DC = (X_OUT + FL + 0.5) - (-146.2) # camera clamp front edge 0.5 behind the flange
ROLL = 30.0                        # motor roll: rear holes 45° away from the L7_cover holes, connectors at 300°
MECH = ('crank', 'rack', 'jaw_1', 'jaw_1_m', 'jaw_2', 'jaw_2_m', 'link-1', 'link-2',
        'MANIFOLD_SOLID_BREP_39_1', 'MANIFOLD_SOLID_BREP_40_1')
CAM = ('Part_1', 'Part_2', 'MD0603901_exportMA2', 'ID_50_PIN_20709-050EC_PAD_4')

a = import_step(SRC)
parts = [[c.label, c] for c in a.children]
def get(label, i=0): return [p for p in parts if p[0] == label][i]
log = []
V = lambda *v: Vector(*v)
def xcyl(r, x0, x1, y=0.0, z=ZC):
    """cylinder along X from x0 to x1"""
    lo, hi = min(x0, x1), max(x0, x1)
    return Solid.make_cylinder(r, hi - lo, Plane(origin=(lo, y, z), z_dir=(1, 0, 0)))
def at(r, ang):
    return r * math.cos(math.radians(ang)), ZC + r * math.sin(math.radians(ang))     # (y, z), angle atan2(z, y)
def check(shape, name):
    assert shape.is_valid and len(shape.solids()) == 1, f'{name}: invalid or {len(shape.solids())} solids'
    return shape
def fuse(*s):
    r = s[0].fuse(*s[1:], tol=0.001)
    return r if isinstance(r, Solid) or len(r.solids()) != 1 else r.solids()[0]

# ---------------- moves ----------------
for p in parts:
    if p[0] in MECH: p[1] = p[1].moved(Location((DM, 0, 0)))
    elif p[0] in CAM: p[1] = p[1].moved(Location((DC, 0, 0)))
log.append(f'gripper mechanism ({", ".join(MECH)}) moved {DM:+.1f} in X (toward the wrist); camera group moved {DC:+.1f}')

# ---------------- motor ----------------
m = import_step(MOTOR)              # local: output face y=0 (+Y), rear face y=-29, Ø42; connectors at local -Z
m = m.rotate(Axis.Y, -ROLL).rotate(Axis.Z, 90).moved(Location((X_OUT, 0, ZC)))
motor = Compound(m.solids()); motor.label = 'DM-H3510-J7'
def motor_holes(dia, pcd, xmin, xmax):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    out = set()
    for f in motor.faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() != GeomAbs_Cylinder: continue
        c = s.Cylinder(); d = c.Axis().Direction(); o = c.Axis().Location(); bb = f.bounding_box()
        if abs(abs(d.X()) - 1) > 1e-3 or abs(2 * c.Radius() - dia) > 0.05 or bb.max.X < xmin or bb.min.X > xmax: continue
        r = math.hypot(o.Y(), o.Z() - ZC)
        if abs(2 * r - pcd) < 0.3: out.add(round(math.degrees(math.atan2(o.Z() - ZC, o.Y())) % 360, 1))
    return sorted(out)
OUT_H = motor_holes(3.0, 16.0, X_OUT - 1, X_OUT + 4)
REAR_H = motor_holes(3.0, 36.0, X_REAR - 7, X_REAR + 1)
assert len(OUT_H) == 6 and len(REAR_H) == 4, (OUT_H, REAR_H)
parts = [p for p in parts if p[0] != 'EL05_L7'] + [['DM-H3510-J7', motor]]
log.append(f'EL05_L7 -> DM-H3510 (official), output face x={X_OUT}, rear face x={X_REAR}; output holes {OUT_H}, rear holes {REAR_H} (deg)')

# ---------------- L7_motor: the cup ----------------
COVER_H = (30.0, 121.0, 211.0, 301.0)        # L7_cover screw holes (Ø38.5), formerly into the EL05 rear
FRAME_H = tuple(30.0 + 60 * k for k in range(6))
cup = fuse(xcyl(R_TUBE, X_OUT + FL, X_COVER), xcyl(R_FL, X_OUT, X_OUT + FL))
cup = cup - xcyl(R_BORE, X_OUT - 0.1, X_REAR)
cup = cup - xcyl(10.5, X_REAR - 0.1, X_COVER + 0.1)                 # rear boss Ø20 of the motor
for ang in COVER_H:                                                  # M3 heat-set inserts (Ø4 x 4) for the L7_cover screws
    y, z = at(19.25, ang); cup = cup - xcyl(2.0, X_REAR - 0.1, X_COVER + 0.1, y, z)
for ang in REAR_H:                                                   # motor screws, countersunk from the L7_cover side
    y, z = at(18.0, ang)
    cup = cup - xcyl(1.7, X_REAR - 0.1, X_COVER + 0.1, y, z)
    cup = cup - Solid.make_cone(1.7, 3.3, 1.6, Plane(origin=(X_COVER - 1.6, y, z), z_dir=(1, 0, 0)))
for ang in FRAME_H:                                                  # frame screws, countersunk on the flange back
    y, z = at(R_FS, ang)
    cup = cup - xcyl(1.7, X_OUT - 0.1, X_OUT + FL + 0.1, y, z)
    cup = cup - Solid.make_cone(1.7, 3.3, 1.6, Plane(origin=(X_OUT + FL - 1.6, y, z), z_dir=(1, 0, 0)))
# connector window (connectors 2.7..8.4 mm in front of the motor rear face, at 300°)
win = Box(10.0, 20, 10, align=(Align.MIN, Align.CENTER, Align.CENTER)) \
    .moved(Location((X_REAR - 10.0, 0, ZC - R_TUBE)))
win = win.rotate(Axis((0, 0, ZC), (1, 0, 0)), 300.0 - 270.0)
cup = check(cup - (win & (xcyl(R_TUBE + 1, X_REAR - 10.0, X_REAR) - xcyl(R_BORE - 0.5, X_REAR - 10.0, X_REAR))), 'L7_motor')
parts.append(['L7_motor', cup])
log.append(f'L7_motor (new): tube Ø{2*R_TUBE:.0f} x {X_COVER - X_OUT - FL:.1f}, bore Ø{2*R_BORE}, bottom {BOT} on L7_cover (4x M3 heat-set inserts), '
           f'motor on 4x M3 countersunk, flange Ø{2*R_FL:.0f} x {FL} with 6x M3 countersunk on Ø{2*R_FS:.0f}, connector window at 300°')

# ---------------- rack: bigger ring with inserts ----------------
p = get('rack'); r = p[1]
x_back = X_OUT                                                       # frame back face on the motor output face
r = fuse(r, xcyl(R_FL, x_back - 5.0, x_back) - xcyl(24.9, x_back - 5.1, x_back + 0.1))
for ang in (45, 90, 135, 225, 270, 315):                             # old EL05 screw holes (Ø3.4 on Ø41.5)
    y, z = at(20.75, ang); r = fuse(r, xcyl(1.8, x_back - 3.3, x_back, y, z))
for ang in FRAME_H:                                                  # heat-set inserts M3 (Ø4 x 4)
    y, z = at(R_FS, ang); r = r - xcyl(2.0, x_back - 4.0, x_back + 0.1, y, z)
p[1] = check(r, 'rack')
log.append('rack: ring Ø50 -> Ø60 (back 5 mm), 6 EL05 screw holes plugged, 6x Ø4x4 for M3 heat-set inserts on Ø54')

# ---------------- crank: hub for the H3510 output ----------------
p = get('crank'); r = p[1]
x_hf = -167.0 + DM                                                   # crank front face (arm plate)
# EL05 hub: 6x M4 Ø4.4 on Ø24 at 35°+60k (countersunk Ø9.4 from the front, chamfered at the back),
# 3 pin slots Ø4.3 on Ø17.7..19 at 65/185/305° - each filled by its own plug, everything stays inside the Ø32 hub
def void(y, z, rad):
    """the empty space of an old hole: cylinder minus the crank, only the piece on the hole axis (no outside air)"""
    v = xcyl(rad, -167.0 + DM, X_OUT, y, z) - r
    keep = [sl for sl in v.solids() if math.hypot(sl.center().Y - y, sl.center().Z - z) < 2.0]
    assert keep, 'no void found'
    return keep
plugs = []
for ang in range(35, 360, 60):
    plugs += void(*at(12.0, ang), 5.0)
for ang in (65, 185, 305):
    plugs += void(*at(9.18, ang), 3.0)
sec = r & Plane(origin=(-163.0 + DM, 0, 0), z_dir=(1, 0, 0))           # outline below the old countersinks (intact)
env = [extrude(Face(f.outer_wire()), amount=X_OUT - (-167.0 + DM) + 0.2, dir=(1, 0, 0)).moved(Location((-167.1 + DM - (-163.0 + DM), 0, 0)))
       for f in sec.faces()]
env = env[0] if len(env) == 1 else fuse(*env)
r = fuse(r, *[pl & env for pl in plugs]).clean()
for ang in OUT_H:
    y, z = at(8.0, ang)
    r = r - xcyl(1.7, x_hf - 0.1, X_OUT + 0.1, y, z) - xcyl(3.1, x_hf - 0.1, x_hf + 3.2, y, z)
r = r - xcyl(3.8, X_OUT - 1.2, X_OUT + 0.1)                          # Ø7 centre boss of the output
p[1] = check(r, 'crank')
log.append('crank: hub filled and re-drilled: 6x M3 Ø16 counterbored Ø6.2x3.2 from the front, Ø7.6x1.2 recess for the output boss')

# ---------------- export ----------------
kids = [Compound([s], label=n) for n, s in parts]
export_step(Compound(children=kids, label='magarm_step4_gripper'), OUT + '/magarm_step4_gripper.step')
for n in ('L7_motor', 'rack', 'crank'):
    export_step(get(n)[1], f'{OUT}/{n}.step'); export_stl(get(n)[1], f'{OUT}/{n}.stl', tolerance=0.02, angular_tolerance=0.1)
open(OUT + '/step4_log.txt', 'w').write('\n'.join(log))
print('\n'.join(log))
