import numpy as np

from teach_replay.motion import blend_to_pose, quintic, write_pose_trajectory
from teach_replay.ros_helpers import FRANKA_HOME


def test_home_file_matches_controller_format(tmp_path):
    path = tmp_path / 'home.csv'
    write_pose_trajectory(str(path), FRANKA_HOME)
    with open(path) as f:
        assert f.readline().startswith('t,q1')  # header row the C++ loader skips
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    assert data.shape == (2, 15)
    assert abs(data[1, 0] - data[0, 0] - 0.001) < 1e-9
    np.testing.assert_allclose(data[:, 1:8], [FRANKA_HOME] * 2)
    np.testing.assert_allclose(data[:, 8:], 0.0)


def test_franka_home_is_the_standard_ready_pose():
    np.testing.assert_allclose(
        FRANKA_HOME, [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4], atol=1e-9)


def test_hand_blend_follows_approach_progress():
    target = blend_to_pose([1.0, 0.5], [0.0, 0.0])
    assert target(0, 0.0) == [1.0, 0.5]
    np.testing.assert_allclose(target(0, 0.5), [0.5, 0.25])
    assert target(0, 1.0) == [0.0, 0.0]
    assert target(2, 0.0) == [0.0, 0.0]
    assert quintic(-1) == 0.0 and quintic(2) == 1.0
