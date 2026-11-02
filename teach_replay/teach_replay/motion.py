"""Arm motion through trajectory_replay_controller, with the hand driven alongside.

Used by replay_demo (play a processed demo) and record_demo (return to the pose
captured at the start of the recording). Both go through the same controller:
it always approaches the first sample of its file slowly (quintic, <= 0.2 rad/s),
so "move to a pose" is just a file holding that one pose.
"""

import time
from typing import Callable, List, Optional

import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

from teach_replay.ros_helpers import (
    FRANKA_HOME,
    RECOVERY_HINT,
    REPLAY_CONTROLLER,
    XHAND_DEFAULT_KP,
    XHAND_HOME_POSITION,
    XHAND_POSITION_MODE,
    RobotLink,
)

HAND_RATE_HZ = 83.0
COUNTDOWN_S = 3
PROGRESS_STALE_S = 0.3  # no fresh progress for this long -> stop commanding the hand
PHASES = {0: 'approach', 1: 'replay', 2: 'done'}

# hand_target(phase, value) -> hand joint targets, or None to send nothing this tick
HandTarget = Callable[[int, float], Optional[List[float]]]


def quintic(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def write_pose_trajectory(path: str, arm_q: List[float]) -> None:
    """A two-sample 1 kHz file that just holds `arm_q` (the controller approaches it first)."""
    row = list(arm_q) + [0.0] * 7
    header = 't,' + ','.join(f'q{i}' for i in range(1, 8)) + ',' + \
        ','.join(f'dq{i}' for i in range(1, 8))
    np.savetxt(path, np.array([[0.0] + row, [0.001] + row]), delimiter=',', header=header,
               comments='', fmt='%.9f')


def prepare_replay_controller(link: RobotLink, trajectory_file: str) -> None:
    """Leave trajectory_replay_controller configured (inactive) with `trajectory_file` loaded."""
    link.deactivate_arm_controllers()
    if REPLAY_CONTROLLER in link.controller_states():
        link.unload_controller(REPLAY_CONTROLLER)  # so configure re-reads the file
    link.load_controller(REPLAY_CONTROLLER)
    link.set_controller_parameter_string(REPLAY_CONTROLLER, 'trajectory_file', trajectory_file)
    link.configure_controller(REPLAY_CONTROLLER)


def blend_to_pose(hand_from: List[float], hand_to: List[float]) -> HandTarget:
    """Hand target for a plain move: blend on the approach progress, then hold `hand_to`."""
    def target(phase: int, value: float):
        if phase == 0:
            b = quintic(value)
            return [a + (h - a) * b for a, h in zip(hand_from, hand_to)]
        return list(hand_to)
    return target


def execute(link: RobotLink, hand_id: int, hand_names: List[str], hand_target: HandTarget,
            hand_kp: float = XHAND_DEFAULT_KP, label: str = 'Moving') -> None:
    """Activate the (already prepared) replay controller and run it to the end.

    The hand is commanded at 83 Hz from the controller's own ~/progress, so it follows the
    arm's real timeline. The controller is always deactivated on return, including on
    Ctrl+C or error.
    """
    progress = {'msg': None, 'stamp': 0.0}

    def on_progress(msg):
        progress['msg'] = list(msg.data)
        progress['stamp'] = time.monotonic()

    def hand_tick():
        msg = progress['msg']
        if msg is None or time.monotonic() - progress['stamp'] > PROGRESS_STALE_S:
            return  # arm not reporting -> the hand just holds its last target
        target = hand_target(int(msg[0]), msg[1])
        if target is not None:
            link.send_hand(hand_id, hand_names, target, XHAND_POSITION_MODE, kp=hand_kp)

    sub = link.create_subscription(Float64MultiArray, f'/{REPLAY_CONTROLLER}/progress',
                                   on_progress, 10)
    timer = link.create_timer(1.0 / HAND_RATE_HZ, hand_tick)
    active = False
    try:
        link.require_arm_ready()
        link.switch_controllers([REPLAY_CONTROLLER], [])
        active = True
        print(f'{label} ... (Ctrl+C stops)')
        last_print = 0.0
        while True:
            rclpy.spin_once(link, timeout_sec=0.02)
            if link.robot_mode_name() not in ('IDLE', 'MOVE'):
                raise RuntimeError(f'Franka stopped (robot_mode {link.robot_mode_name()}). '
                                   f'{RECOVERY_HINT}')
            msg = progress['msg']
            if msg is None:
                continue
            phase = int(msg[0])
            if time.monotonic() - last_print > 1.0:
                last_print = time.monotonic()
                value = f'{msg[1] * 100:.0f}%' if phase == 0 else f'{msg[1]:.1f}/{msg[3]:.1f}s'
                waiting = '  (waiting for the arm to catch up)' if msg[2] else ''
                print(f'  {PHASES.get(phase, "?"):8s} {value}{waiting}')
            if phase == 2:
                link.spin_for(1.0)  # let the hand reach its final target
                break
    finally:
        link.destroy_timer(timer)
        link.destroy_subscription(sub)
        if active:
            link.switch_controllers([], [REPLAY_CONTROLLER])


def go_home(link: RobotLink, hand_id: int, hand_index: int, scratch_file: str,
            label: str = 'Moving to home') -> None:
    """Move the arm to FRANKA_HOME and the hand to its calibrated zero, both slowly."""
    write_pose_trajectory(scratch_file, FRANKA_HOME)
    prepare_replay_controller(link, scratch_file)
    link.spin_for(0.1)
    names = link.hand_joint_names(hand_index)
    execute(link, hand_id, names,
            blend_to_pose(link.hand_positions(hand_index), [XHAND_HOME_POSITION] * len(names)),
            label=label)


def countdown(message: str) -> bool:
    """Print a countdown; False if the operator pressed Ctrl+C to skip."""
    print(message)
    try:
        for i in range(COUNTDOWN_S, 0, -1):
            print(f'  {i} ...', flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('  skipped.')
        return False
    return True
