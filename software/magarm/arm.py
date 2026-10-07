"""MaGaRM arm on one CAN bus: Damiao joints (standard frames) and RobStride EL05 joints (extended frames).

    from magarm import Arm
    with Arm() as arm:
        arm.enable()
        arm.move_to([0, 0.6, -1.2, 0.4, 0, 0, 0], duration=3.0)
        print(arm.q())
        arm.disable()
"""
import threading, time
import numpy as np
import can

from . import damiao as dm, robstride as rs
from .config import JOINTS, CAN_CHANNEL, CAN_INTERFACE


class Arm:
    def __init__(self, channel=CAN_CHANNEL, interface=CAN_INTERFACE, joints=JOINTS):
        self.joints = joints
        self.bus = can.Bus(channel=channel, interface=interface, bitrate=1_000_000)
        self.state = {j.name: None for j in joints}
        self.limits = {j.name: dm.LIMITS[j.model] for j in joints if j.protocol == 'damiao'}
        self._by_master = {j.master_id: j for j in joints if j.protocol == 'damiao'}
        self._by_rs = {j.can_id: j for j in joints if j.protocol == 'robstride'}
        self._params = {}
        self._run = True
        self._rx = threading.Thread(target=self._receive, daemon=True); self._rx.start()
        self._read_damiao_limits()

    # ---------------- bus ----------------
    def _send(self, arb_id, data, extended=False):
        self.bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=extended))

    def _receive(self):
        while self._run:
            msg = self.bus.recv(0.1)
            if msg is None: continue
            if msg.is_extended_id:
                r = rs.parse_feedback(msg.arbitration_id, msg.data)
                if r and r[0] in self._by_rs: self.state[self._by_rs[r[0]].name] = r[1]
            else:
                j = self._by_master.get(msg.arbitration_id)
                if j is None: continue
                pr = dm.parse_param_reply(msg.data)
                if pr: self._params[(j.name, pr[0])] = pr[1]
                else: self.state[j.name] = dm.parse_feedback(msg.data, self.limits[j.name])

    def _read_damiao_limits(self, timeout=0.3):
        """PMAX/VMAX/TMAX are set inside each Damiao motor; use the motor's values when it answers"""
        for j in self.joints:
            if j.protocol != 'damiao': continue
            for rid in (dm.RID_PMAX, dm.RID_VMAX, dm.RID_TMAX):
                self._send(*dm.read_param_frame(j.can_id, rid))
        time.sleep(timeout)
        for j in self.joints:
            vals = [self._params.get((j.name, rid)) for rid in (dm.RID_PMAX, dm.RID_VMAX, dm.RID_TMAX)]
            if all(v is not None for v in vals): self.limits[j.name] = tuple(vals)

    # ---------------- motor commands ----------------
    def enable(self):
        for j in self.joints:
            if j.protocol == 'damiao':
                self._send(*dm.special_frame(j.can_id, dm.ENABLE))
            else:
                self._send(*rs.run_mode_frame(j.can_id, 0), extended=True)
                self._send(*rs.enable_frame(j.can_id), extended=True)
            time.sleep(0.002)

    def disable(self):
        for j in self.joints:
            if j.protocol == 'damiao': self._send(*dm.special_frame(j.can_id, dm.DISABLE))
            else: self._send(*rs.stop_frame(j.can_id), extended=True)
            time.sleep(0.002)

    def clear_errors(self):
        for j in self.joints:
            if j.protocol == 'damiao': self._send(*dm.special_frame(j.can_id, dm.CLEAR_ERROR))
            else: self._send(*rs.stop_frame(j.can_id, clear_fault=True), extended=True)

    def set_zero(self, name):
        """store the current motor position as the motor zero (arm in the CAD pose: folded, see docs/software.md)"""
        j = next(j for j in self.joints if j.name == name)
        if j.protocol == 'damiao': self._send(*dm.special_frame(j.can_id, dm.SET_ZERO))
        else: self._send(*rs.zero_frame(j.can_id), extended=True)

    def _mit(self, j, pos, vel, kp, kd, tau):
        if j.protocol == 'damiao':
            self._send(*dm.mit_frame(j.can_id, self.limits[j.name], pos, vel, kp, kd, tau))
        else:
            self._send(*rs.mit_frame(j.can_id, pos, vel, kp, kd, tau), extended=True)

    # ---------------- joint space (URDF convention) ----------------
    def q(self):
        """joint angles in the URDF convention (nan for a joint without feedback yet)"""
        return np.array([np.nan if self.state[j.name] is None else j.direction * (self.state[j.name].pos - j.offset)
                         for j in self.joints])

    def command(self, q, dq=None, kp=None, kd=None, tau=None):
        """one MIT command per joint; q is clamped to the URDF limits"""
        n = len(self.joints)
        dq = np.zeros(n) if dq is None else dq
        tau = np.zeros(n) if tau is None else tau
        for i, j in enumerate(self.joints):
            qi = min(max(q[i], j.lower), j.upper)
            self._mit(j, j.direction * qi + j.offset, j.direction * dq[i],   # q = direction * (motor - offset)
                      j.kp if kp is None else kp[i], j.kd if kd is None else kd[i], j.direction * tau[i])

    def passive(self, damping=0.3):
        """zero stiffness, light damping: move the arm by hand (teleop teaching)"""
        for j in self.joints:
            self._mit(j, 0.0, 0.0, 0.0, damping, 0.0)

    def move_to(self, q_goal, duration=3.0, rate=200.0):
        """minimum-jerk move from the current position"""
        self.passive(); time.sleep(0.05)
        q0 = self.q()
        if np.isnan(q0).any(): raise RuntimeError(f'no feedback from: {[j.name for j, v in zip(self.joints, q0) if np.isnan(v)]}')
        q_goal = np.asarray(q_goal, float); steps = max(1, int(duration * rate))
        for k in range(1, steps + 1):
            s = k / steps; s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            self.command(q0 + (q_goal - q0) * s)
            time.sleep(1.0 / rate)

    def close(self):
        self._run = False; self._rx.join(1.0); self.bus.shutdown()

    def __enter__(self): return self
    def __exit__(self, *a): self.close()
