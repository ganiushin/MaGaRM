"""MaGaRM joint table. Angles are in the URDF convention (urdf/magarm.urdf): q = direction * (motor - offset).

Motors are zeroed in the CAD pose (tools/set_zero.py); joint3 is zeroed at its fold stop, 3.5° short of q = 0,
hence its offset. Adjust `offset` if the zero pose was not exact and check `direction` once with tools/scan.py
by moving each joint by hand: the reported q must grow in the direction of the URDF axis.
"""
from dataclasses import dataclass


@dataclass
class Joint:
    name: str
    protocol: str          # 'damiao' or 'robstride'
    model: str
    can_id: int
    master_id: int         # Damiao feedback id (MST_ID), unused for RobStride
    direction: float
    offset: float          # motor angle (rad) at q = 0 (CAD pose, arm folded)
    lower: float           # URDF limits (rad)
    upper: float
    kp: float              # default MIT gains
    kd: float


JOINTS = [
    Joint('joint1', 'damiao', 'DM-JH17-101', 0x01, 0x11, 1.0, 0.0, -2.79253, 2.79253, 40.0, 4.5),
    Joint('joint2', 'damiao', 'DM-JH17-101', 0x02, 0x12, 1.0, 0.0, 0.0, 3.49066, 60.0, 5.0),
    Joint('joint3', 'damiao', 'DM-J4340P', 0x03, 0x13, 1.0, 0.0611,   # zeroed at the fold stop (q = -3.5°): offset = 0.0611 * direction
           -4.01426, -0.0611, 50.0, 2.5),
    Joint('joint4', 'damiao', 'DM-J4340', 0x04, 0x14, 1.0, 0.0, -0.8727, 1.5708, 30.0, 1.5),
    Joint('joint5', 'robstride', 'EL05', 0x05, 0x00, 1.0, 0.0, -1.5708, 1.5708, 20.0, 1.0),
    Joint('joint6', 'robstride', 'EL05', 0x06, 0x00, 1.0, 0.0, -1.5708, 1.5708, 20.0, 1.0),
    Joint('gripper', 'damiao', 'DM-H3510', 0x07, 0x17, 1.0, 0.0, -1.5708, 1.5708, 2.0, 0.05),
]

CAN_CHANNEL = 'can0'       # SocketCAN (gs_usb / candleLight adapter), 1 Mbit/s
CAN_INTERFACE = 'socketcan'
