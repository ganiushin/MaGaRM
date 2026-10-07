"""Store motor zeros with the arm in the CAD pose (q = 0): arm fully folded, see docs/software.md.

    python tools/set_zero.py joint2 joint3     # only the listed joints
    python tools/set_zero.py all
"""
import sys, time
sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])
from magarm import Arm

names = sys.argv[1:]
if not names: sys.exit(__doc__)
with Arm() as arm:
    todo = [j.name for j in arm.joints] if names == ['all'] else names
    arm.disable(); time.sleep(0.1)
    for n in todo:
        arm.set_zero(n); time.sleep(0.05); print('zero stored:', n)
