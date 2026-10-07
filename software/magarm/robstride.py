"""RobStride EL05 on CAN 2.0 extended frames (29-bit id), 1 Mbit/s.

29-bit id = type << 24 | data2 << 8 | target_id
  type 1  motion control : data2 = torque (u16)         data = pos, vel, kp, kd (u16 big-endian)
  type 2  feedback       : id bits 8..15 = motor id, 16..21 = fault bits, 22..23 = mode; data = pos, vel, torque, temp*10
  type 3  enable, type 4 stop (data[0]=1 clears faults), type 6 set zero (data[0]=1), type 18 write parameter
Before enabling, run_mode (0x7005) must be 0 = motion control.
"""
import struct
from dataclasses import dataclass

MOTION, FEEDBACK, ENABLE, STOP, SET_ZERO, WRITE_PARAM = 1, 2, 3, 4, 6, 18
RUN_MODE = 0x7005
HOST_ID = 0xFD
# EL05 ranges of the motion-control frame: P, V, T, KP, KD
EL05 = dict(p=12.57, v=50.0, t=6.0, kp=500.0, kd=5.0)


def f2u16(x, lo, hi):
    x = min(max(x, lo), hi)
    return int(round((x - lo) * 65535.0 / (hi - lo)))


def u162f(u, lo, hi):
    return u * (hi - lo) / 65535.0 + lo


def ext_id(comm_type, data2, target):
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target & 0xFF)


@dataclass
class RobStrideState:
    pos: float = 0.0
    vel: float = 0.0
    tau: float = 0.0
    temp: float = 0.0
    fault: int = 0
    mode: int = 0


def enable_frame(motor_id):
    return ext_id(ENABLE, HOST_ID, motor_id), bytes(8)


def stop_frame(motor_id, clear_fault=False):
    return ext_id(STOP, HOST_ID, motor_id), bytes([1 if clear_fault else 0]) + bytes(7)


def zero_frame(motor_id):
    return ext_id(SET_ZERO, HOST_ID, motor_id), bytes([1]) + bytes(7)


def run_mode_frame(motor_id, mode=0):
    return ext_id(WRITE_PARAM, HOST_ID, motor_id), struct.pack('<HBxI', RUN_MODE, 0, mode)


def mit_frame(motor_id, pos, vel, kp, kd, tau, r=EL05):
    data = struct.pack('>HHHH', f2u16(pos, -r['p'], r['p']), f2u16(vel, -r['v'], r['v']),
                       f2u16(kp, 0.0, r['kp']), f2u16(kd, 0.0, r['kd']))
    return ext_id(MOTION, f2u16(tau, -r['t'], r['t']), motor_id), data


def parse_feedback(can_id, data, r=EL05):
    """(motor_id, state) for a type-2 frame, else None"""
    if (can_id >> 24) & 0x1F != FEEDBACK or len(data) != 8:
        return None
    p, v, t, temp = struct.unpack('>HHHH', bytes(data))
    return (can_id >> 8) & 0xFF, RobStrideState(u162f(p, -r['p'], r['p']), u162f(v, -r['v'], r['v']),
                                                u162f(t, -r['t'], r['t']), temp / 10.0,
                                                (can_id >> 16) & 0x3F, (can_id >> 22) & 0x03)
