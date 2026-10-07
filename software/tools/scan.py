"""Print every joint's feedback (motor angle and URDF angle) while the arm is moved by hand.

    python tools/scan.py            # motors stay disabled; Damiao answers the refresh query, EL05 the stop frame
"""
import sys, time
sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])
from magarm import Arm
from magarm import damiao as dm, robstride as rs

with Arm() as arm:
    print('Damiao PMAX/VMAX/TMAX:', arm.limits)
    try:
        while True:
            for j in arm.joints:                                  # ask for feedback without enabling
                if j.protocol == 'damiao':
                    arm._send(dm.PARAM_ID, bytes([j.can_id & 0xFF, j.can_id >> 8, 0xCC, 0, 0, 0, 0, 0]))
                else:
                    arm._send(*rs.stop_frame(j.can_id), extended=True)
            time.sleep(0.1)
            line = []
            for j, q in zip(arm.joints, arm.q()):
                s = arm.state[j.name]
                line.append(f'{j.name}: ' + ('--' if s is None else f'm={s.pos:+.3f} q={q:+.3f}'))
            print('  '.join(line), end='\r')
    except KeyboardInterrupt:
        print()
