"""Round trips of the CAN frame encoders (no hardware needed)."""
import math
from magarm import damiao as dm, robstride as rs


def test_damiao_mit_roundtrip():
    lim = dm.LIMITS['DM-J4340P']
    cid, data = dm.mit_frame(0x02, lim, 1.234, -2.5, 30.0, 1.5, 4.0)
    assert cid == 0x02 and len(data) == 8
    p = ((data[0] << 8) | data[1]); v = (data[2] << 4) | (data[3] >> 4)
    kp = ((data[3] & 0xF) << 8) | data[4]; kd = (data[5] << 4) | (data[6] >> 4); t = ((data[6] & 0xF) << 8) | data[7]
    assert math.isclose(dm.u2f(p, -12.5, 12.5, 16), 1.234, abs_tol=1e-3)
    assert math.isclose(dm.u2f(v, -10, 10, 12), -2.5, abs_tol=1e-2)
    assert math.isclose(dm.u2f(kp, 0, 500, 12), 30.0, abs_tol=0.2)
    assert math.isclose(dm.u2f(kd, 0, 5, 12), 1.5, abs_tol=2e-3)
    assert math.isclose(dm.u2f(t, -28, 28, 12), 4.0, abs_tol=2e-2)


def test_damiao_feedback():
    lim = (12.5, 10.0, 28.0)
    p, v, t = dm.f2u(0.5, -12.5, 12.5, 16), dm.f2u(-1.0, -10, 10, 12), dm.f2u(2.0, -28, 28, 12)
    data = bytes([0x12, p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (t >> 8), t & 0xFF, 35, 40])
    s = dm.parse_feedback(data, lim)
    assert s.status == 1 and s.status_text == 'enabled'
    assert abs(s.pos - 0.5) < 1e-3 and abs(s.vel + 1.0) < 1e-2 and abs(s.tau - 2.0) < 2e-2
    assert s.t_mos == 35 and s.t_rotor == 40


def test_damiao_special_and_param():
    assert dm.special_frame(3, dm.ENABLE) == (3, bytes([0xFF] * 7 + [0xFC]))
    cid, data = dm.read_param_frame(0x104, dm.RID_TMAX)
    assert cid == 0x7FF and data[:4] == bytes([0x04, 0x01, 0x33, 23])
    import struct
    assert dm.parse_param_reply(bytes([4, 1, 0x33, 23]) + struct.pack('<f', 28.0)) == (23, 28.0)


def test_robstride_roundtrip():
    cid, data = rs.mit_frame(5, 0.7, -3.0, 20.0, 1.0, -1.5)
    assert (cid >> 24) == rs.MOTION and cid & 0xFF == 5
    assert abs(rs.u162f((cid >> 8) & 0xFFFF, -6, 6) + 1.5) < 1e-3
    fb_id = (rs.FEEDBACK << 24) | (2 << 22) | (5 << 8) | rs.HOST_ID
    import struct
    payload = struct.pack('>HHHH', rs.f2u16(0.7, -12.57, 12.57), rs.f2u16(-3.0, -50, 50), rs.f2u16(1.0, -6, 6), 315)
    mid, st = rs.parse_feedback(fb_id, payload)
    assert mid == 5 and st.mode == 2 and abs(st.pos - 0.7) < 1e-3 and abs(st.temp - 31.5) < 1e-6


def test_robstride_run_mode_frame():
    cid, data = rs.run_mode_frame(6, 0)
    assert (cid >> 24) == rs.WRITE_PARAM and data[:2] == bytes([0x05, 0x70]) and data[4:] == bytes(4)
