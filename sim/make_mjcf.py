"""Build the MuJoCo model of MaGaRM (sim/magarm.xml) from urdf/magarm.urdf and the gripper export (sim/gripper.json).

Usage: python sim/make_mjcf.py
Arm: kinematics, limits, masses and meshes from the URDF. Actuators are PD position servos like the MIT mode of the
motors (tau = kp (q* - q) - kd dq, clipped to the URDF effort), default gains from software/magarm/config.py.
Gripper: the real mechanism as a closed chain. The motor turns the crank (gripper_joint), two rods push the jaws
along the rail (slide joints jaw_l, jaw_r); each rod end is tied to its jaw pin by a connect constraint.
The crank range is cut to where it is useful: from the widest opening to the closest approach of the pads.
Collisions: convex hulls of the meshes. The arm does not collide with itself (check self-collisions separately),
only with the scene (contype 2, conaffinity 1 against the default 1/1 of scene objects).
"""
import importlib.util, json, math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

SIM = Path(__file__).resolve().parent
ROOT = SIM.parent
_spec = importlib.util.spec_from_file_location('config', ROOT / 'software' / 'magarm' / 'config.py')   # without python-can
config = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(config)
JOINTS = config.JOINTS

GAINS = {('gripper_joint' if j.name == 'gripper' else j.name): (j.kp, j.kd) for j in JOINTS}
# Reflected rotor inertia (kg m2). Not measured: rotor ~1.3e-5 kg m2 x 40^2 for the J4340, ~1e-5 x 101^2 for the JH17,
# small for the EL05 and the H3510. Replace with values from a system identification of the real joints.
ARMATURE = {'joint1': 0.1, 'joint2': 0.1, 'joint3': 0.02, 'joint4': 0.02, 'joint5': 0.005, 'joint6': 0.005,
            'gripper_joint': 1e-4}
# Dry friction of the harmonic drives (N m at the output), also a guess until the real joints are measured.
FRICTION = {'joint1': 1.0, 'joint2': 1.0}
COLORS = {'grey': '0.3 0.31 0.34 1', 'blue': '0.27 0.55 0.9 1'}
WRIST_FOVY = 70                                         # deg, vertical: a 100 deg diagonal lens at 4:3 (docs/bom.md)
KEYS = {'folded': [0, 0, -0.0349, 0, 0, 0],          # CAD pose; arm joints 1-6, gripper open
        'ready': [0, 1.85, -1.78, 1.5, 0, 0]}           # tcp 25 cm out over the work area (-X), 15 cm up, jaws down

GRIP = json.load(open(SIM / 'gripper.json'))
urdf = ET.parse(ROOT / 'urdf' / 'magarm.urdf').getroot()
links = {l.get('name'): l for l in urdf.findall('link')}
joints = [j for j in urdf.findall('joint') if j.get('type') == 'revolute']
tcp_joint = next(j for j in urdf.findall('joint') if j.get('name') == 'tcp_joint')
AXIS = np.array(next(j for j in joints if j.get('name') == 'gripper_joint').find('origin').get('xyz').split(), float)


def f(v): return ' '.join(f'{0.0 if abs(x) < 1e-9 else x:.6g}' for x in v)


def chain(theta, side):
    """closed chain at crank angle theta: (jaw slide, rod hinge) for side 'l'/'r', or None past the dead point"""
    rod, jaw = GRIP[f'rod_{side}'], GRIP[f'jaw_{side}']
    p0, e0 = np.array(rod['pos'][1:]), np.array(rod['end'][1:])          # crank pin (crank axis at y=z=0), rod
    j0 = np.array(jaw['pos'][1:]) - AXIS[1:]                               # jaw pin
    c, s = math.cos(theta), math.sin(theta)
    p = np.array([p0[0] * c - p0[1] * s, p0[0] * s + p0[1] * c])
    dz = j0[1] - p[1]; L = np.linalg.norm(e0)
    if dz * dz > L * L: return None
    y = p[0] + math.copysign(math.sqrt(L * L - dz * dz), e0[0])
    phi = math.atan2(dz, y - p[0]) - math.atan2(e0[1], e0[0]) - theta     # rod angle relative to the crank
    return y - j0[0], (phi + math.pi) % (2 * math.pi) - math.pi


def gap(theta):
    """distance between the pad faces"""
    c = chain(theta, 'l')
    return None if c is None else 2 * (GRIP['pad_in'] + c[0])


th = np.radians(np.arange(-90.0, 90.01, 0.25))
g = np.array([np.nan if gap(t) is None else gap(t) for t in th])
i0, i1 = sorted((int(np.nanargmax(g)), int(np.nanargmin(g))))
CRANK = (th[i0], th[i1])
travel = {s: sorted(chain(t, s)[0] for t in th[i0:i1 + 1]) for s in 'lr'}
OPEN = th[int(np.nanargmax(g))]
print(f'gripper: crank {math.degrees(CRANK[0]):.1f}..{math.degrees(CRANK[1]):.1f} deg, open at {math.degrees(OPEN):.1f} deg, '
      f'gap {np.nanmax(g) * 1000:.1f}..{np.nanmin(g) * 1000:.1f} mm')


def inertial(parent, pos, mass, I):
    I = np.asarray(I)
    ET.SubElement(parent, 'inertial', pos=f(pos), mass=f'{mass:.6g}',
                  fullinertia=f((I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])))


def geoms(body, mesh, rgba, collide=True, cls='collision'):
    ET.SubElement(body, 'geom', {'class': 'visual', 'mesh': mesh, 'rgba': rgba})
    if collide: ET.SubElement(body, 'geom', {'class': cls, 'mesh': mesh})


m = ET.Element('mujoco', model='magarm')
ET.SubElement(m, 'compiler', angle='radian', autolimits='true')
ET.SubElement(m, 'option', integrator='implicitfast', cone='elliptic', impratio='10')
d = ET.SubElement(ET.SubElement(m, 'default'), 'default', {'class': 'magarm'})
ET.SubElement(d, 'joint', damping='0.05')
ET.SubElement(d, 'position', inheritrange='1')
ET.SubElement(ET.SubElement(d, 'default', {'class': 'visual'}), 'geom', type='mesh', contype='0', conaffinity='0', group='2')
ET.SubElement(ET.SubElement(d, 'default', {'class': 'collision'}), 'geom', type='mesh', contype='2', conaffinity='1', group='3')
ET.SubElement(ET.SubElement(d, 'default', {'class': 'pad'}), 'geom', type='mesh', contype='2', conaffinity='1', group='3',
              condim='4', friction='1.5 0.01 0.001', priority='1')    # TPU pads

asset = ET.SubElement(m, 'asset')
for l in links.values():
    v = l.find('visual')
    if v is not None and l.get('name') != 'link6':
        ET.SubElement(asset, 'mesh', name=l.get('name'), file=f'../urdf/{v.find("geometry/mesh").get("filename")}')
for b in ('link6', 'jaw_l', 'jaw_r', 'rod_l', 'rod_r'):
    for mesh in GRIP[b]['meshes']: ET.SubElement(asset, 'mesh', name=mesh, file=f'assets/{mesh}.stl')

base = ET.SubElement(ET.SubElement(m, 'worldbody'), 'body', name='base_link', childclass='magarm')


def urdf_inertial(body, link):
    inl = link.find('inertial'); i = inl.find('inertia')
    I = np.array([[float(i.get('i' + ''.join(sorted(a + c)))) for c in 'xyz'] for a in 'xyz'])
    inertial(body, map(float, inl.find('origin').get('xyz').split()), float(inl.find('mass').get('value')), I)


urdf_inertial(base, links['base_link'])
geoms(base, 'base_link', COLORS['grey'])
bodies = {'base_link': base}
for j in joints:
    name, child = j.get('name'), j.find('child').get('link')
    l = links[child]; lim = j.find('limit')
    b = ET.SubElement(bodies[j.find('parent').get('link')], 'body', name=child, pos=j.find('origin').get('xyz'))
    rng = CRANK if name == 'gripper_joint' else (float(lim.get('lower')), float(lim.get('upper')))
    eff = float(lim.get('effort'))
    ET.SubElement(b, 'joint', name=name, axis=j.find('axis').get('xyz'), range=f(rng),
                  armature=f'{ARMATURE[name]:g}', actuatorfrcrange=f((-eff, eff)),
                  **({'frictionloss': f'{FRICTION[name]:g}'} if name in FRICTION else {}))
    if child == 'link6':                                # link6 without the moving gripper parts
        g6 = GRIP['link6']; inertial(b, g6['com'], g6['mass'], g6['inertia'])
        geoms(b, 'link6_static', COLORS['blue'])
        ET.SubElement(b, 'site', name='tcp', pos=tcp_joint.find('origin').get('xyz'), size='0.005', rgba='1 0.2 0.2 1', group='4')
        z = -np.array(GRIP['camera']['view']); x = np.array([0.0, 1.0, 0.0])     # image rows along the jaw travel
        ET.SubElement(b, 'camera', name='wrist', pos=f(GRIP['camera']['pos']), xyaxes=f((*x, *np.cross(z, x))), fovy=f'{WRIST_FOVY:g}')
    else:
        urdf_inertial(b, l)
        geoms(b, child, COLORS[l.find('visual/material').get('name')], collide=child != 'gripper_link')
    bodies[child] = b

eq = ET.Element('equality')
for side in 'lr':
    jaw, rod = GRIP[f'jaw_{side}'], GRIP[f'rod_{side}']
    b = ET.SubElement(bodies['link6'], 'body', name=f'jaw_{side}', pos=f(jaw['pos']))
    inertial(b, jaw['com'], jaw['mass'], jaw['inertia'])
    ET.SubElement(b, 'joint', name=f'jaw_{side}', type='slide', axis='0 1 0', range=f((travel[side][0], travel[side][-1])),
                  damping='0.5')
    geoms(b, f'jaw_{side}', COLORS['blue'])
    geoms(b, f'pad_{side}', '0.15 0.15 0.15 1', cls='pad')
    geoms(b, f'handle_{side}', COLORS['grey'], collide=False)
    b = ET.SubElement(bodies['gripper_link'], 'body', name=f'rod_{side}', pos=f(rod['pos']))
    inertial(b, rod['com'], rod['mass'], rod['inertia'])
    ET.SubElement(b, 'joint', name=f'rod_{side}', axis='1 0 0', damping='0.001')
    geoms(b, f'rod_{side}', COLORS['grey'], collide=False)
    ET.SubElement(eq, 'connect', name=f'rod_{side}', body1=f'rod_{side}', body2=f'jaw_{side}', anchor=f(rod['end']))
m.append(eq)

act = ET.SubElement(m, 'actuator')
for j in joints:
    name = j.get('name'); kp, kd = GAINS[name]
    ET.SubElement(act, 'position', {'class': 'magarm', 'name': name, 'joint': name, 'kp': f'{kp:g}', 'kv': f'{kd:g}'})

sens = ET.SubElement(m, 'sensor')
for j in joints: ET.SubElement(sens, 'actuatorfrc', name=f'tau_{j.get("name")}', actuator=j.get('name'))
ET.SubElement(sens, 'framepos', name='tcp_pos', objtype='site', objname='tcp')
ET.SubElement(sens, 'framequat', name='tcp_quat', objtype='site', objname='tcp')

# keyframes: the closed chain needs a consistent qpos, so compile once to find the joint addresses
out = SIM / 'magarm.xml'
out.write_text(ET.tostring(m, encoding='unicode'))
model = mujoco.MjModel.from_xml_path(str(out))
key = ET.SubElement(m, 'keyframe')
for kname, q in KEYS.items():
    qpos = np.zeros(model.nq)
    for n, v in zip([j.get('name') for j in joints], q + [OPEN]): qpos[model.joint(n).qposadr[0]] = v
    for s in 'lr':
        y, phi = chain(OPEN, s)
        qpos[model.joint(f'jaw_{s}').qposadr[0]] = y; qpos[model.joint(f'rod_{s}').qposadr[0]] = phi
    ET.SubElement(key, 'key', name=kname, qpos=f(qpos), ctrl=f(q + [OPEN]))
ET.indent(m)
out.write_text('<!-- MaGaRM for MuJoCo, generated by sim/make_mjcf.py from urdf/magarm.urdf and sim/gripper.json -->\n'
               + ET.tostring(m, encoding='unicode') + '\n')
print('->', out)
