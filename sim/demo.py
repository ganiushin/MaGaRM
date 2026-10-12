"""Pick-and-place demo in MuJoCo: MaGaRM takes the red cube and puts it on the plate.

Usage:
  python sim/demo.py              # headless: prints the result and the peak joint torques, saves frames to sim/out/
  mjpython sim/demo.py --view     # the same in the viewer (on macOS the passive viewer needs mjpython)
Joint targets come from a damped least-squares IK on the tcp site (gripper pointing down, jaws square to the cube),
started from the previous solution and from random poses near the target. The servos are the same PD as the motors
in MIT mode (soft gains from software/magarm/config.py) plus gravity compensation through the actuators, clipped
to the motor torque together with the PD, which is what Arm.command(tau=...) does on the real arm.
"""
import sys, time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from PIL import Image

SIM = Path(__file__).resolve().parent
# gravity compensation of the whole arm (gripper parts included) routed through the actuators: set before compiling
spec = mujoco.MjSpec.from_file(str(SIM / 'scene.xml'))
for b in spec.body('base_link').find_all(mujoco.mjtObj.mjOBJ_BODY): b.gravcomp = 1
for n in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_joint']: spec.joint(n).actgravcomp = 1
m = spec.compile()
d = mujoco.MjData(m)
ARM = [m.joint(f'joint{i}').id for i in range(1, 7)]
DOF = [m.jnt_dofadr[j] for j in ARM]
QADR = [m.jnt_qposadr[j] for j in ARM]
TCP = m.site('tcp').id
GRIP = m.actuator('gripper_joint').id
OPEN, CLOSE = m.actuator_ctrlrange[GRIP]
ACT_DOF = m.jnt_dofadr[m.actuator_trnid[:, 0]]           # joint torque of each motor, clipped to its limit

def down(yaw):
    """tcp orientation: x up (jaws point down), jaws closing along (cos yaw, sin yaw, 0)"""
    x, y = np.array([0.0, 0, 1]), np.array([np.cos(yaw), np.sin(yaw), 0])
    return np.column_stack([x, y, np.cross(x, y)])


def solve(pos, rot, q0, iters=200):
    """damped least squares from q0; returns (q, pose error)"""
    k = mujoco.MjData(m)
    k.qpos[QADR] = q0
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    for _ in range(iters):
        mujoco.mj_kinematics(m, k); mujoco.mj_comPos(m, k)
        R = k.site_xmat[TCP].reshape(3, 3)
        e = np.concatenate([pos - k.site_xpos[TCP], 0.5 * sum(np.cross(R[:, i], rot[:, i]) for i in range(3))])
        if np.linalg.norm(e) < 1e-6: break
        mujoco.mj_jacSite(m, k, jp, jr, TCP)
        J = np.vstack([jp, jr])[:, DOF]
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
        for a, j, v in zip(QADR, ARM, dq):
            k.qpos[a] = np.clip(k.qpos[a] + v, *m.jnt_range[j])
    return k.qpos[QADR].copy(), np.linalg.norm(e)


rng = np.random.default_rng(0)
BANK = np.array([[rng.uniform(*m.jnt_range[j]) for j in ARM] for _ in range(20000)])
_k = mujoco.MjData(m)
BANK_P, BANK_UP = [], []                               # tcp position, how much the jaws point down
for q in BANK:
    _k.qpos[QADR] = q; mujoco.mj_kinematics(m, _k)
    BANK_P.append(_k.site_xpos[TCP].copy()); BANK_UP.append(_k.site_xmat[TCP][6])
BANK_P, BANK_UP = np.array(BANK_P), np.array(BANK_UP)


def ik(pos, q0, yaw=0.0):
    """gripper down at pos, jaws at yaw + k 90 deg (square object); prefers the solution closest to q0"""
    seeds = [q0] + list(BANK[np.argsort(np.linalg.norm(BANK_P - pos, axis=1) + 0.2 * (1 - BANK_UP))[:12]])
    best = None
    for k in range(4):
        for s in seeds:
            q, err = solve(pos, down(yaw + k * np.pi / 2), s)
            score = (err > 1e-3, np.linalg.norm(q - q0))
            if best is None or score < best[0]: best = (score, q, err)
    return best[1], best[2]


def run(q_goal, grip, T, viewer=None, peak=None):
    """move the servos to q_goal / grip in T seconds (smooth step), gravity compensated"""
    q_start, g_start = d.ctrl[:6].copy(), d.ctrl[GRIP]
    n = int(T / m.opt.timestep)
    for i in range(n):
        s = (i + 1) / n; s = s * s * (3 - 2 * s)
        d.ctrl[:6] = q_start + s * (q_goal - q_start)
        d.ctrl[GRIP] = g_start + s * (grip - g_start)
        mujoco.mj_step(m, d)
        if peak is not None: np.maximum(peak, np.abs(d.qfrc_actuator[ACT_DOF]), out=peak)
        if viewer is not None and i % 8 == 0:
            viewer.sync(); time.sleep(8 * m.opt.timestep)


def main(view=False):
    out = SIM / 'out'; out.mkdir(exist_ok=True)
    mujoco.mj_resetDataKeyframe(m, d, m.key('folded').id)
    mujoco.mj_forward(m, d)
    cube, plate = d.body('cube').xpos.copy(), d.body('plate').xpos.copy()
    up = np.array([0, 0, 0.10])
    grasp = cube + [0, 0, 0.012]                        # pad tips stick out 24 mm past the tcp: keep them off the table
    put = plate + [0, 0, 0.002] + (grasp - cube) + [0, 0, cube[2] + 0.003]
    plan = [('over cube', grasp + up, OPEN, 2.5), ('down', grasp, OPEN, 1.5), ('grip', grasp, CLOSE, 1.0),
            ('lift', grasp + up, CLOSE, 1.5), ('over plate', put + up, CLOSE, 2.0),
            ('put', put, CLOSE, 1.5), ('release', put, OPEN, 1.0), ('away', put + up, OPEN, 1.0)]
    peak = np.zeros(m.nu)                               # motor torque: PD + gravity compensation
    viewer = None
    if view:
        viewer = mujoco.viewer.launch_passive(m, d)
    renderer = mujoco.Renderer(m, 480, 640)
    q = d.qpos[QADR].copy()
    for name, pos, grip, T in plan:
        q, err = ik(pos, q0=q)
        run(q, grip, T, viewer, peak)
        print(f'{name:10s} ik err {err * 1000:4.1f} mm, q {np.degrees(q).round(0)}, tcp {np.round(d.site_xpos[TCP], 3)}, cube {np.round(d.body("cube").xpos, 3)}')
        if name in ('grip', 'put'):
            for cam in ('front', 'wrist'):
                renderer.update_scene(d, camera=cam); Image.fromarray(renderer.render()).save(out / f'{name}_{cam}.png')
    run(q, OPEN, 1.0, viewer, peak)
    on_plate = np.linalg.norm((d.body('cube').xpos - plate)[:2])
    print(f'cube {on_plate * 1000:.0f} mm from the plate centre ->', 'OK' if on_plate < 0.03 else 'MISSED')
    print('peak |tau| (N m) / limit:')
    for i in range(m.nu):
        print(f'  {m.actuator(i).name:14s} {peak[i]:6.2f} / {m.jnt_actfrcrange[m.actuator_trnid[i, 0]][1]:g}')
    print('frames ->', out)
    if viewer is not None:
        while viewer.is_running(): viewer.sync(); time.sleep(0.05)


if __name__ == '__main__':
    main('--view' in sys.argv)
