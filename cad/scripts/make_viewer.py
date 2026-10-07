"""Build viewer/model.json (meshes for the three.js viewer) from the final assembly.

Usage: python make_viewer.py <magarm_step5_logo.step> <viewer_dir> [j3_open_deg]
The forearm and everything after it is shown opened by j3_open_deg about J3 (default 2°: the fold stop).
"""
import sys, os, json, base64, tempfile
import numpy as np
import trimesh
from build123d import *
from export_parts import PRINTED

SRC, OUT = sys.argv[1], sys.argv[2]
J3_OPEN = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
FIXED3 = {'L1_base', 'L2_base', 'L2_cover', 'L3-motor', 'L3_back', '6705', '6707', 'DM4340P-J1', 'DM4340P-J2', 'DM4340P-J3'}
NEW = {'L1_base', 'L2_base', 'L3-motor', 'L3_back', 'L4_right', 'L4_left', 'L5_motor', 'L5_back', 'Part_27', 'L7_motor', 'rack', 'crank'}
DARK = {'L2_cover', 'L3_back', 'L4_left', 'L5_back', 'L7_cover', 'handle', 'handle_m', 'jaw_2', 'jaw_2_m'}
BOUGHT = {'MANIFOLD_SOLID_BREP_40_1': 'rail MGN7', 'MANIFOLD_SOLID_BREP_39_1': 'carriage MGN7C',
          'MD0603901_exportMA2': 'camera', 'ID_50_PIN_20709-050EC_PAD_4': 'camera board',
          '6704': 'bearing 6704', '6705': 'bearing 6705', '6707': 'bearing 6707'}
COL = {'motor': [240, 120, 30], 'steel': [150, 150, 160], 'new': [70, 140, 230], 'dark': [45, 48, 55], 'print': [215, 220, 228]}

a = import_step(SRC)
parts, seen = [], {}
for c in a.children:
    lab = c.label
    shp = c.rotate(Axis((181.0, 0, 70.75), (0, 1, 0)), J3_OPEN) if J3_OPEN and lab not in FIXED3 else c
    with tempfile.NamedTemporaryFile(suffix='.stl') as t:
        export_stl(shp, t.name, tolerance=0.4, angular_tolerance=0.7)
        m = trimesh.load(t.name)
    if lab.startswith(('DM', 'EL05')): col, name = COL['motor'], lab
    elif lab in BOUGHT: col, name = COL['steel'], BOUGHT[lab]
    else:
        name = PRINTED.get(lab, (lab,))[0]
        col = COL['new'] if lab in NEW else COL['dark'] if lab in DARK else COL['print']
    k = seen.get(name, 0); seen[name] = k + 1
    parts.append({'name': name if k == 0 else f'{name}_{k}', 'color': col,
                  'pos': base64.b64encode(m.vertices.astype(np.float32).tobytes()).decode(),
                  'idx': base64.b64encode(m.faces.astype(np.uint32).tobytes()).decode()})
json.dump({'units': 'mm', 'up': 'Z', 'parts': parts}, open(f'{OUT}/model.json', 'w'), separators=(',', ':'))
print(len(parts), 'parts,', round(os.path.getsize(f'{OUT}/model.json') / 1e6, 1), 'MB')
