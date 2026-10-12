"""Arm against simulated motors on a python-can virtual bus (no hardware)."""
import threading, time, struct
import numpy as np
import can
from magarm import Arm, JOINTS
from magarm import damiao as dm, robstride as rs


class FakeMotors(threading.Thread):
    """each motor follows its MIT position target and answers like the real firmware"""
    def __init__(self, channel):
        super().__init__(daemon=True)
        self.bus = can.Bus(channel=channel, interface='virtual'); self.pos = {j.name: 0.0 for j in JOINTS}; self.run_ = True

    def run(self):
        while self.run_:
            m = self.bus.recv(0.05)
            if m is None: continue
            for j in JOINTS:
                if j.protocol == 'damiao' and not m.is_extended_id:
                    lim = dm.LIMITS[j.model]
                    if m.arbitration_id == dm.PARAM_ID and m.data[0] == j.can_id and m.data[2] == dm.READ_PARAM:
                        val = lim[m.data[3] - dm.RID_PMAX]
                        self.bus.send(can.Message(arbitration_id=j.master_id, is_extended_id=False,
                                                  data=bytes(m.data[:4]) + struct.pack('<f', val)))
                    elif m.arbitration_id == j.can_id:
                        kp = ((m.data[3] & 0xF) << 8) | m.data[4]
                        if bytes(m.data[:7]) != bytes([0xFF] * 7) and kp > 0:      # follows the target only with stiffness
                            self.pos[j.name] = dm.u2f((m.data[0] << 8) | m.data[1], -lim[0], lim[0], 16)
                        p = dm.f2u(self.pos[j.name], -lim[0], lim[0], 16); v = dm.f2u(0, -lim[1], lim[1], 12); t = dm.f2u(0, -lim[2], lim[2], 12)
                        self.bus.send(can.Message(arbitration_id=j.master_id, is_extended_id=False,
                                                  data=bytes([0x10 | j.can_id, p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (t >> 8), t & 0xFF, 30, 30])))
                if j.protocol == 'robstride' and m.is_extended_id and (m.arbitration_id & 0xFF) == j.can_id:
                    if (m.arbitration_id >> 24) == rs.MOTION and struct.unpack('>H', bytes(m.data[4:6]))[0] > 0:
                        self.pos[j.name] = rs.u162f(struct.unpack('>H', bytes(m.data[:2]))[0], -12.57, 12.57)
                    fid = (rs.FEEDBACK << 24) | (2 << 22) | (j.can_id << 8) | rs.HOST_ID
                    self.bus.send(can.Message(arbitration_id=fid, is_extended_id=True,
                                              data=struct.pack('>HHHH', rs.f2u16(self.pos[j.name], -12.57, 12.57), 32767, 32767, 300)))


def test_move_to_virtual():
    fake = FakeMotors('vbus'); fake.start()
    with Arm(channel='vbus', interface='virtual') as arm:
        assert arm.limits['joint1'] == dm.LIMITS['DM-JH17-101']          # limits read back from the motors
        arm.enable(); time.sleep(0.05)
        goal = np.array([0.3, 0.8, -1.0, 0.4, -0.5, 0.6, 0.2])
        arm.move_to(goal, duration=0.5)
        time.sleep(0.1); arm.passive(); time.sleep(0.1)
        assert np.allclose(arm.q(), goal, atol=0.01)
        arm.command(np.array([5, 9, 1, -3, 4, 4, 4]))                       # clamped to the URDF limits
        time.sleep(0.1); arm.passive(); time.sleep(0.1)
        hi = np.array([j.upper for j in JOINTS]); lo = np.array([j.lower for j in JOINTS])
        assert np.all(arm.q() <= hi + 0.01) and np.all(arm.q() >= lo - 0.01)
        arm.disable()
    fake.run_ = False
