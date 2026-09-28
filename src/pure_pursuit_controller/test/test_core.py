import math

from pure_pursuit_controller.pure_pursuit_core import (
    normalize_angle, PurePursuit, quaternion_to_yaw)


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def test_quaternion_to_yaw_roundtrip():
    for yaw in (-3.0, -1.2, 0.0, 0.7, 2.9):
        assert abs(quaternion_to_yaw(*yaw_to_quat(yaw)) - yaw) < 1e-9


def test_normalize_angle():
    assert abs(normalize_angle(3 * math.pi / 2) + math.pi / 2) < 1e-9


def test_straight_line_gives_zero_steer():
    path = [(i * 1.0, 0.0) for i in range(50)]
    pp = PurePursuit(path, closed_loop=False)
    delta, omega, _, _ = pp.compute(0.0, 0.0, 0.0, 3.0)
    assert abs(delta) < 1e-9 and abs(omega) < 1e-9


def test_path_to_the_left_turns_left():
    path = [(i * 1.0, 2.0) for i in range(50)]
    pp = PurePursuit(path, closed_loop=False)
    delta, omega, _, _ = pp.compute(0.0, 0.0, 0.0, 3.0)
    assert delta > 0 and omega > 0


def test_bicycle_mapping():
    path = [(i * 1.0, 2.0) for i in range(50)]
    pp = PurePursuit(path, wheelbase=2.7, closed_loop=False)
    delta, omega, _, _ = pp.compute(0.0, 0.0, 0.0, 3.0)
    assert abs(omega - 3.0 * math.tan(delta) / 2.7) < 1e-12


def test_index_never_goes_back_on_loop():
    circle = [(20 * math.cos(t), 20 * math.sin(t))
              for t in [i * 2 * math.pi / 200 for i in range(200)]]
    pp = PurePursuit(circle, closed_loop=True)
    last = -1
    for i in range(400):
        t = i * 2 * math.pi / 200
        pp.compute(20 * math.cos(t), 20 * math.sin(t), t + math.pi / 2, 3.0)
        assert pp.nearest_idx >= last
        last = pp.nearest_idx
    assert last >= 390  # two laps counted without wrap jumps
