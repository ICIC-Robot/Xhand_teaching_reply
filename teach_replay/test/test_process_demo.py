"""End-to-end: synthetic bag -> process_demo -> files the replay controller accepts."""

import sys

import numpy as np
import pytest
import yaml

rosbag2_py = pytest.importorskip('rosbag2_py')
from rclpy.serialization import serialize_message  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from xhand_control_interfaces.msg import XHandState, XHandStateArray  # noqa: E402

from teach_replay import process_demo  # noqa: E402

HAND_NAMES = ['thumb_bend_joint', 'thumb_rota_joint1', 'thumb_rota_joint2', 'index_bend_joint',
              'index_joint1', 'index_joint2', 'mid_joint1', 'mid_joint2', 'ring_joint1',
              'ring_joint2', 'pinky_joint1', 'pinky_joint2']
HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
T0 = 1_700_000_000.0


def _profile(t, start, duration):
    s = np.clip((t - start) / duration, 0.0, 1.0)
    return s * s * (3 - 2 * s)


def _write_bag(bag_dir, seed=0):
    """2 s idle, a fast shaky 1 s arm move + 0.5 s hand close, 2 s idle."""
    rng = np.random.default_rng(seed)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, typ in ((process_demo.ARM_TOPIC, 'sensor_msgs/msg/JointState'),
                      (process_demo.HAND_TOPIC, 'xhand_control_interfaces/msg/XHandStateArray')):
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ,
                                                     serialization_format='cdr'))
    delta = np.array([0.8, 0.4, -0.3, 0.5, 0.2, -0.4, 0.6])
    for t in np.arange(0.0, 5.0, 1.0 / 500.0):
        msg = JointState()
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(T0 + t), int(((T0 + t) % 1) * 1e9)
        msg.name = process_demo.ARM_JOINTS
        msg.position = list(HOME + delta * _profile(t, 2.0, 1.0) + 0.002 * rng.standard_normal(7))
        msg.velocity = [0.0] * 7
        writer.write(process_demo.ARM_TOPIC, serialize_message(msg), int((T0 + t) * 1e9))
    for t in np.arange(0.0, 5.0, 1.0 / 100.0):
        msg = XHandStateArray()
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(T0 + t), int(((T0 + t) % 1) * 1e9)
        msg.hand_id = [0]
        hs = XHandState()
        hs.name = HAND_NAMES
        hs.position = [1.2 * float(_profile(t, 2.3, 0.5))] * 12
        msg.hand_states = [hs]
        writer.write(process_demo.HAND_TOPIC, serialize_message(msg), int((T0 + t) * 1e9))
    del writer


@pytest.fixture(scope='module')
def processed(tmp_path_factory):
    demo = tmp_path_factory.mktemp('demo')
    _write_bag(demo / 'bag')
    argv = sys.argv
    sys.argv = ['process_demo', str(demo), '--no-plot']
    try:
        process_demo.main()
    finally:
        sys.argv = argv
    arm = np.loadtxt(demo / 'arm_trajectory.csv', delimiter=',', skiprows=1)
    hand = np.loadtxt(demo / 'hand_trajectory.csv', delimiter=',', skiprows=1)
    summary = yaml.safe_load((demo / 'summary.yaml').read_text())
    return arm, hand, summary


def test_arm_file_matches_controller_format(processed):
    arm, _, summary = processed
    assert arm.shape[1] == 15  # t, q1..q7, dq1..dq7
    dt = np.diff(arm[:, 0])
    assert np.allclose(dt, 0.001, atol=1e-6)  # the C++ loader requires an even 1 kHz grid
    assert summary['ok'] and summary['problems'] == []


def test_fast_demo_is_slowed_within_caps(processed):
    arm, _, summary = processed
    # The 1 s arm move peaks at 1.2 rad/s on joint 1, above the 0.87 rad/s cap.
    assert summary['replay_duration_s'] > 1.0 / 0.75
    dq = arm[:, 8:15]
    ddq = np.diff(dq, axis=0) / 0.001
    assert np.all(np.max(np.abs(dq), axis=0) <= 0.4 * process_demo.ARM_DQ_MAX * 1.01)
    assert np.all(np.max(np.abs(ddq), axis=0) <= 0.4 * process_demo.ARM_DDQ_MAX * 1.05)
    assert np.allclose(dq[0], 0.0) and np.allclose(dq[-1], 0.0)


def test_path_endpoints_and_hand_kept(processed):
    arm, hand, _ = processed
    delta = np.array([0.8, 0.4, -0.3, 0.5, 0.2, -0.4, 0.6])
    np.testing.assert_allclose(arm[0, 1:8], HOME, atol=0.01)
    np.testing.assert_allclose(arm[-1, 1:8], HOME + delta, atol=0.01)
    assert hand.shape[1] == 13
    np.testing.assert_allclose(hand[0, 1:], 0.0, atol=0.02)
    np.testing.assert_allclose(hand[-1, 1:], 1.2, atol=0.02)
    assert abs(hand[-1, 0] - arm[-1, 0]) < 0.01  # same timeline


def test_hand_velocity_capped(processed):
    _, hand, summary = processed
    v = np.abs(np.diff(hand[:, 1:], axis=0)) / np.diff(hand[:, 0])[:, None]
    assert v.max() <= 1.0 * 1.05
