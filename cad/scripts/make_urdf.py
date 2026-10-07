"""Generate the MaGaRM URDF (kinematics, meshes, mass properties) from the final CAD assembly.

Usage: python make_urdf.py <magarm_step5_logo.step> <urdf_dir>
Frames: every link frame is parallel to base_link and sits on its joint axis (zero pose = CAD pose, arm folded).
Joint axes and signs follow the motor directions used by the firmware (see software/magarm/config.py).
Masses: printed parts PLA at ~55 % fill (0.68 g/cm3), motors with their datasheet mass, steel parts 7.8 g/cm3.
"""
import sys, os, math
import numpy as np
from build123d import *
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

SRC, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT + '/meshes', exist_ok=True)
BASE = np.array([0.6, 0.0, -66.0])          # CAD point of base_link: J1 axis at table level (mm)
AZ = 134.75                                  # J4 / J6 axis height
# joint: (CAD point mm, axis, lower, upper, effort Nm, velocity rad/s)
JOINTS = [
    ('joint1', 'base_link', 'link1', (0.6, 0, 1.75), (0, 0, 1), -2.79253, 2.79253, 27.0, 5.8),
    ('joint2', 'link1', 'link2', (-9.0, 0, 70.75), (0, -1, 0), 0.0, 3.66519, 27.0, 5.8),
    ('joint3', 'link2', 'link3', (181.0, 0, 70.75), (0, -1, 0), -4.01426, -0.0349, 27.0, 5.8),     # last 2° of the fold blocked
    ('joint4', 'link3', 'link4', (-4.0, 0, AZ), (0, -1, 0), -0.8727, 1.5708, 27.0, 5.8),            # up-tilt 50° (drum roof)
    ('joint5', 'link4', 'link5', (-58.55, 0, 172.75), (0, 0, -1), -1.5708, 1.5708, 6.0, 22.0),
    ('joint6', 'link5', 'link6', (-51.74, 0, AZ), (1, 0, 0), -1.5708, 1.5708, 6.0, 22.0),
    ('gripper_joint', 'link6', 'gripper_link', (-145.5, 0, AZ), (1, 0, 0), -1.5708, 1.5708, 0.45, 50.0),
]
LINKS = {
    'base_link': ['L1_base', 'DM4340P-J1', '6707'],
    'link1': ['L2_base', 'L2_cover'],
    'link2': ['L3-motor', 'L3_back', '6705', 'DM4340P-J2', 'DM4340P-J3'],
    'link3': ['L4_right', 'L4_left', '6704'],
    'link4': ['L5_motor', 'L5_back', 'DM4340-J4', 'L5_shake', 'EL05_L5'],
    'link5': ['L6_link', 'EL05_L6'],
    'link6': ['L7_base', 'L7_cover', 'L7_motor', 'DM-H3510-J7', 'rack', 'MANIFOLD_SOLID_BREP_39_1', 'MANIFOLD_SOLID_BREP_40_1',
              'Part_1', 'Part_2', 'MD0603901_exportMA2', 'ID_50_PIN_20709-050EC_PAD_4',
              'jaw_1', 'jaw_1_m', 'jaw_2', 'jaw_2_m', 'link-1', 'link-2', 'handle', 'handle_m'],
    'gripper_link': ['crank'],
}
MOTOR_KG = {'DM4340P-J1': 0.362, 'DM4340P-J2': 0.362, 'DM4340P-J3': 0.362, 'DM4340-J4': 0.362,
            'EL05_L5': 0.242, 'EL05_L6': 0.242, 'DM-H3510-J7': 0.116}
STEEL = {'6707', '6705', '6704', 'MANIFOLD_SOLID_BREP_39_1', 'MANIFOLD_SOLID_BREP_40_1'}
OTHER = {'MD0603901_exportMA2': 1.5, 'ID_50_PIN_20709-050EC_PAD_4': 1.5}   # camera, adapter board (g/cm3)
PLA = 0.68                                    # g/cm3 effective

a = import_step(SRC)
parts = [(c.label, c) for c in a.children]
labels = {l for l, _ in parts}
assigned = {l for v in LINKS.values() for l in v}
assert labels <= assigned, f'parts without a link: {labels - assigned}'
origin = {'base_link': BASE}
for j in JOINTS: origin[j[2]] = np.array(j[3], float)

def props(shape):
    """volume (mm3), centre (mm), inertia about the centre (mm5, unit density)"""
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape.wrapped, g)
    c = g.CentreOfMass(); m = g.MatrixOfInertia()
    return g.Mass(), np.array([c.X(), c.Y(), c.Z()]), np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])

def mesh(shape, o):
    """OCC mesher on the whole shape (shared edges, no skipped faces), vertices relative to o"""
    import tempfile, trimesh
    with tempfile.NamedTemporaryFile(suffix='.stl') as t:
        export_stl(shape, t.name, tolerance=0.4, angular_tolerance=0.6)
        m = trimesh.load(t.name, process=False)
    return np.asarray(m.vertices) - o, np.asarray(m.faces)

def fmt(v): return ' '.join(f'{x:.6g}' for x in v)
links_xml, joints_xml = [], []
for link, labs in LINKS.items():
    o = origin[link]
    mass, mom1, items, meshes = 0.0, np.zeros(3), [], []
    for lab, shp in parts:
        if lab not in labs: continue
        sol = Compound(shp.solids())
        V, c, I = props(sol)
        if V <= 0: continue
        if lab in MOTOR_KG:
            rho = MOTOR_KG[lab] / V                       # kg/mm3
        else:
            rho = (7.8 * 0.6 if lab in STEEL else OTHER.get(lab, PLA)) * 1e-6
        m = rho * V
        items.append((m, c, I * rho)); mass += m; mom1 += m * c
        meshes.append(mesh(sol, o))
    com = mom1 / mass
    Itot = np.zeros((3, 3))
    for m, c, I in items:
        d = c - com
        Itot += I + m * (d @ d * np.eye(3) - np.outer(d, d))
    Itot *= 1e-6                                          # kg mm2 -> kg m2
    # mesh (metres, link frame)
    V_, F_, off = [], [], 0
    for v, t in meshes:
        V_.append(v / 1000.0); F_.append(t + off); off += len(v)
    V_, F_ = np.vstack(V_), np.vstack(F_)
    with open(f'{OUT}/meshes/{link}.stl', 'wb') as fh:
        fh.write(b'MaGaRM'.ljust(80, b' ')); fh.write(np.uint32(len(F_)).tobytes())
        tri = V_[F_]; n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        rec = np.zeros(len(F_), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
        rec['n'] = n; rec['v'] = tri; fh.write(rec.tobytes())
    c_ = (com - o) / 1000.0
    links_xml.append(f'''  <link name="{link}">
    <inertial>
      <origin xyz="{fmt(c_)}" rpy="0 0 0"/>
      <mass value="{mass:.4f}"/>
      <inertia ixx="{Itot[0,0]:.6g}" ixy="{Itot[0,1]:.6g}" ixz="{Itot[0,2]:.6g}" iyy="{Itot[1,1]:.6g}" iyz="{Itot[1,2]:.6g}" izz="{Itot[2,2]:.6g}"/>
    </inertial>
    <visual><geometry><mesh filename="meshes/{link}.stl"/></geometry><material name="{'grey' if link in ('base_link', 'link2', 'link4') else 'blue'}"/></visual>
    <collision><geometry><mesh filename="meshes/{link}.stl"/></geometry></collision>
  </link>''')
    print(f'{link:13s} mass {mass:.3f} kg, com {np.round(com, 1)} mm (CAD)')
for name, parent, child, pt, ax, lo, hi, eff, vel in JOINTS:
    xyz = (origin[child] - origin[parent]) / 1000.0
    joints_xml.append(f'''  <joint name="{name}" type="revolute">
    <parent link="{parent}"/><child link="{child}"/>
    <origin xyz="{fmt(xyz)}" rpy="0 0 0"/>
    <axis xyz="{fmt(ax)}"/>
    <limit lower="{lo}" upper="{hi}" effort="{eff}" velocity="{vel}"/>
  </joint>''')
# TCP between the finger pads
pads = [s for l, s in parts if l in ('jaw_2', 'jaw_2_m')]
tcp = np.array([np.mean([p.bounding_box().center().X for p in pads]), 0.0, AZ])
joints_xml.append(f'''  <joint name="tcp_joint" type="fixed">
    <parent link="link6"/><child link="tcp"/>
    <origin xyz="{fmt((tcp - origin['link6']) / 1000.0)}" rpy="0 0 0"/>
  </joint>''')
xml = ['<?xml version="1.0"?>', '<!-- MaGaRM, generated by cad/scripts/make_urdf.py from the CAD assembly. Zero pose = CAD pose (arm folded). -->',
       '<robot name="magarm">', '  <material name="blue"><color rgba="0.27 0.55 0.9 1"/></material>',
       '  <material name="grey"><color rgba="0.3 0.31 0.34 1"/></material>'] + links_xml + ['  <link name="tcp"/>'] + joints_xml + ['</robot>']
open(f'{OUT}/magarm.urdf', 'w').write('\n'.join(xml) + '\n')
print('tcp at', np.round(tcp, 1), 'mm (CAD); urdf ->', OUT)
