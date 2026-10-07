"""Export every printed part of the final MaGaRM assembly (STEP + STL) and a manifest for the BOM.

Usage: python export_parts.py <magarm_step5_logo.step> <out_dir>
Parts are exported in their assembly position; lay them on a flat face in the slicer. STLs are checked watertight.
"""
import sys, os, json
import trimesh
from build123d import *
from watertight import make_watertight

# assembly label -> (file name, note); the first instance of a label is exported
PRINTED = {
    'L1_base': ('L1_base', 'base'),
    'L2_base': ('L2_base', 'J1 turntable, J2 fork'),
    'L2_cover': ('L2_cover', 'J2 fork cover'),
    'L3-motor': ('L3_motor', 'upper arm, motor side'),
    'L3_back': ('L3_back', 'upper arm, back side'),
    'L4_right': ('L4_right', 'forearm, J4 motor side'),
    'L4_left': ('L4_left', 'forearm, bearing side'),
    'L5_motor': ('L5_motor', 'wrist drum, J5 plate'),
    'L5_back': ('L5_back', 'wrist drum, roof'),
    'L5_shake': ('L5_shake', 'J5 motor cover'),
    'L6_link': ('L6_link', 'J5-J6 link'),
    'L7_base': ('L7_base', 'J6 output flange'),
    'L7_cover': ('L7_cover', 'gripper rear cover'),
    'L7_motor': ('L7_motor', 'gripper motor cup'),
    'rack': ('gripper_frame', 'jaw frame / rail carrier'),
    'crank': ('gripper_crank', 'crank on the gripper motor'),
    'link-1': ('gripper_link_1', 'crank-jaw link'),
    'link-2': ('gripper_link_2', 'crank-jaw link'),
    'jaw_1': ('jaw_1', 'jaw'),
    'jaw_1_m': ('jaw_1_m', 'jaw (mirrored)'),
    'jaw_2': ('jaw_pad', 'finger pad (TPU recommended)'),
    'jaw_2_m': ('jaw_pad_m', 'finger pad (mirrored)'),
    'handle': ('handle', 'teleop handle (optional)'),
    'handle_m': ('handle_m', 'teleop handle, mirrored (optional)'),
    'Part_1': ('camera_bracket', 'wrist camera bracket'),
    'Part_2': ('camera_clamp', 'wrist camera clamp'),
}
if __name__ == '__main__':
    SRC, OUT = sys.argv[1], sys.argv[2]
    os.makedirs(OUT, exist_ok=True)
    a = import_step(SRC)
    seen, manifest = set(), []
    for c in a.children:
        if c.label not in PRINTED or c.label in seen: continue
        seen.add(c.label)
        name, note = PRINTED[c.label]
        s = c.solids()[0] if len(c.solids()) == 1 else c
        export_step(s, f'{OUT}/{name}.step'); export_stl(s, f'{OUT}/{name}.stl', tolerance=0.02, angular_tolerance=0.1)
        mesh = make_watertight(trimesh.load(f'{OUT}/{name}.stl', process=False))     # close mesher cracks (geometry unchanged)
        assert mesh.is_watertight, f'{name}: STL not watertight'
        mesh.export(f'{OUT}/{name}.stl')
        bb = s.bounding_box()
        manifest.append({'file': name, 'note': note, 'volume_cm3': round(s.volume / 1000, 1),
                         'size_mm': [round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)]})
    missing = set(PRINTED) - seen
    assert not missing, f'missing in the assembly: {missing}'
    json.dump(manifest, open(f'{OUT}/parts.json', 'w'), indent=1, ensure_ascii=False)
    print(f'{len(manifest)} parts -> {OUT}')
