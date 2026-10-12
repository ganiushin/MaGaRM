"""Damiao (DM-JH17-101, DM-J4340P, DM-J4340, DM-H3510) on CAN 2.0 standard frames, 1 Mbit/s, MIT mode.

Frames
  command  : id = CAN_ID          data = p16 | v12 | kp12 | kd12 | t12   (big-endian bit packing)
  special  : id = CAN_ID          data = FF FF FF FF FF FF FF cmd        (FC enable, FD disable, FE zero, FB clear error)
  feedback : id = MASTER_ID       data = [id | err<<4, p16, v12, t12, T_mos, T_rotor]
  param rd : id = 0x7FF           data = [id_lo, id_hi, 0x33, rid, 0, 0, 0, 0] -> reply [.., 0x33, rid, value32 LE]
P/V/T in the frames are scaled by the PMAX/VMAX/TMAX stored in the motor (set with the Damiao debugging tool).
"""
import struct
from dataclasses import dataclass

ENABLE, DISABLE, SET_ZERO, CLEAR_ERROR = 0xFC, 0xFD, 0xFE, 0xFB
PARAM_ID, READ_PARAM = 0x7FF, 0x33
RID_PMAX, RID_VMAX, RID_TMAX = 21, 22, 23
KP_MAX, KD_MAX = 500.0, 5.0

# default PMAX / VMAX / TMAX by model (24 V); read the real values from the motor with read_param_frame()
LIMITS = {
    'DM-JH17-101': (12.5, 5.0, 60.0),     # not factory defaults: set PMAX/VMAX/TMAX to these in the motor (54 Nm peak, 30 rpm)
    'DM-J4340P': (12.5, 10.0, 28.0),
    'DM-J4340': (12.5, 8.0, 28.0),
    'DM-H3510': (12.5, 280.0, 1.0),
}
ERRORS = {0x0: 'disabled', 0x1: 'enabled', 0x8: 'over-voltage', 0x9: 'under-voltage', 0xA: 'over-current',
          0xB: 'MOS over-temperature', 0xC: 'rotor over-temperature', 0xD: 'lost communication', 0xE: 'overload'}


def f2u(x, lo, hi, bits):
    x = min(max(x, lo), hi)
    return int(round((x - lo) * ((1 << bits) - 1) / (hi - lo)))


def u2f(u, lo, hi, bits):
    return u * (hi - lo) / ((1 << bits) - 1) + lo


@dataclass
class DamiaoState:
    pos: float = 0.0
    vel: float = 0.0
    tau: float = 0.0
    t_mos: int = 0
    t_rotor: int = 0
    status: int = 0

    @property
    def status_text(self):
        return ERRORS.get(self.status, f'0x{self.status:X}')


def special_frame(can_id, cmd):
    return can_id, bytes([0xFF] * 7 + [cmd])


def mit_frame(can_id, limits, pos, vel, kp, kd, tau):
    pmax, vmax, tmax = limits
    p = f2u(pos, -pmax, pmax, 16)
    v = f2u(vel, -vmax, vmax, 12)
    kp_ = f2u(kp, 0.0, KP_MAX, 12)
    kd_ = f2u(kd, 0.0, KD_MAX, 12)
    t = f2u(tau, -tmax, tmax, 12)
    data = bytes([p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (kp_ >> 8), kp_ & 0xFF,
                  kd_ >> 4, ((kd_ & 0xF) << 4) | (t >> 8), t & 0xFF])
    return can_id, data


def parse_feedback(data, limits):
    pmax, vmax, tmax = limits
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0xF) << 8) | data[5]
    return DamiaoState(u2f(p, -pmax, pmax, 16), u2f(v, -vmax, vmax, 12), u2f(t, -tmax, tmax, 12),
                       data[6], data[7], data[0] >> 4)


def read_param_frame(can_id, rid):
    return PARAM_ID, bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, READ_PARAM, rid, 0, 0, 0, 0])


def parse_param_reply(data):
    """(rid, float value) if this is a parameter reply, else None"""
    if len(data) == 8 and data[2] == READ_PARAM:
        return data[3], struct.unpack('<f', bytes(data[4:8]))[0]
    return None
