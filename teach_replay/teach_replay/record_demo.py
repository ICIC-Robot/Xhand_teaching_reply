#!/usr/bin/env python3
"""Record a hand-guided demonstration of the Franka arm + XHand into a ros2 bag.

Sequence:
  1. check the arm is IDLE and the hand is publishing
  2. move the arm and the hand to their factory home (slow, <= 0.2 rad/s) -- every demo
     starts there. Arm: Franka's standard ready pose; hand: its calibrated zero (all 0 rad)
  3. raise the collision thresholds (a human push must not be judged a collision)
  4. put the XHand in passive mode (zero stiffness -- pose the fingers by hand)
  5. start `ros2 bag record` (mcap if installed, otherwise sqlite3) of
       /teach_record_controller/joint_states  (arm q, dq, tau, controller timestamps)
       /xhand_control/xhand_state             (hand joints + fingertip sensors)
  6. activate teach_record_controller (gravity compensation + damping tau = -D*dq)
  7. ENTER (or Ctrl+C) stops: controller off, hand holds its pose, bag closed, default
     collision thresholds restored
  8. after a 3 s countdown (let go of the arm and step back; Ctrl+C skips), the arm and
     the hand return to home
"""

import argparse
import datetime
import os
import signal
import subprocess
import sys
import termios
import time

import rclpy

from teach_replay.motion import countdown, go_home
from teach_replay.ros_helpers import (
    DEFAULT_FORCE_THRESHOLDS,
    DEFAULT_JOINT_TORQUE_THRESHOLDS,
    TEACH_CONTROLLER,
    TEACH_FORCE_THRESHOLDS,
    TEACH_JOINT_TORQUE_THRESHOLDS,
    XHAND_STATE_TOPIC,
    RobotLink,
)

ARM_TOPIC = f'/{TEACH_CONTROLLER}/joint_states'
DEFAULT_OUTPUT_DIR = os.path.expanduser('~/xarm_ws/demos')


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('name', nargs='?', default=None,
                   help='Demo name; the bag goes to <output-dir>/<name>/bag. '
                        'Default: demo_<date>_<time>.')
    p.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR,
                   help=f'Default: {DEFAULT_OUTPUT_DIR}')
    p.add_argument('--no-start-home', action='store_true',
                   help='Start recording from wherever the robot is, without moving home first')
    p.add_argument('--no-return-home', action='store_true',
                   help='Do not move back to home after the recording')
    p.add_argument('--hand-index', type=int, default=0,
                   help='Which entry of xhand_state.hand_states to use.')
    p.add_argument('--keep-hand-stiff', action='store_true',
                   help='Do not switch the hand to passive mode.')
    p.add_argument('--keep-collision-thresholds', action='store_true',
                   help='Do not raise the collision thresholds for teaching.')
    return p.parse_args()


def storage_plugin() -> str:
    return 'mcap' if os.path.isdir('/opt/ros/humble/share/rosbag2_storage_mcap') else 'sqlite3'


def save_terminal():
    try:
        return termios.tcgetattr(sys.stdin)
    except (termios.error, ValueError):
        return None  # stdin is not a terminal


def restore_terminal(attrs) -> None:
    if attrs is not None:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, attrs)
        except (termios.error, ValueError):
            pass


def main():
    args = parse_args()
    name = args.name or datetime.datetime.now().strftime('demo_%Y%m%d_%H%M%S')
    demo_dir = os.path.join(args.output_dir, name)
    bag_dir = os.path.join(demo_dir, 'bag')
    if os.path.exists(bag_dir):
        sys.exit(f'ERROR: {bag_dir} already exists -- choose another name.')
    os.makedirs(demo_dir, exist_ok=True)
    home_trajectory = os.path.join(demo_dir, 'go_home_trajectory.csv')

    rclpy.init()
    link = RobotLink('teach_replay_record')
    bag = None
    controller_active = False
    thresholds_raised = False
    recorded = False
    hand_id = 0
    exit_code = 0
    terminal = save_terminal()
    try:
        link.wait_for_states()
        link.require_arm_ready()
        hand_id = link.hand_ids()[args.hand_index]

        if not args.no_start_home:
            link.set_collision_thresholds(DEFAULT_JOINT_TORQUE_THRESHOLDS,
                                          DEFAULT_FORCE_THRESHOLDS)
            if not countdown('Moving arm and hand to HOME in:'):
                raise KeyboardInterrupt
            go_home(link, hand_id, args.hand_index, home_trajectory, 'Moving to home')
            print('At home.')

        link.deactivate_arm_controllers()
        link.ensure_configured(TEACH_CONTROLLER)

        if not args.keep_collision_thresholds:
            link.set_collision_thresholds(TEACH_JOINT_TORQUE_THRESHOLDS, TEACH_FORCE_THRESHOLDS)
            thresholds_raised = True
            print(f'Collision thresholds raised for teaching: joint torque '
                  f'{TEACH_JOINT_TORQUE_THRESHOLDS} Nm, force {TEACH_FORCE_THRESHOLDS}')

        storage = storage_plugin()
        cmd = ['ros2', 'bag', 'record', '-o', bag_dir, '-s', storage, ARM_TOPIC, XHAND_STATE_TOPIC]
        print(f'Starting: {" ".join(cmd)}')
        # stdin=DEVNULL: rosbag2's keyboard controls would otherwise put this terminal in raw
        # mode, and the input() below would immediately read end-of-file.
        bag = subprocess.Popen(cmd, start_new_session=True, stdin=subprocess.DEVNULL,
                               stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        link.spin_for(2.0)  # let the recorder discover both topics
        restore_terminal(terminal)
        if bag.poll() is not None:
            raise RuntimeError('ros2 bag record exited immediately')

        if not args.keep_hand_stiff:
            link.hand_passive(hand_id, args.hand_index)
            print('XHand: passive mode (zero stiffness) -- pose the fingers by hand.')

        link.switch_controllers([TEACH_CONTROLLER], [])
        controller_active = True
        recorded = True
        print('\n=== RECORDING ===  Guide the arm by hand and pose the fingers.')
        print('Press ENTER to stop (Ctrl+C also stops and still saves the bag).')
        try:
            input()
        except KeyboardInterrupt:
            print()
        except EOFError:
            raise RuntimeError('the terminal input closed while waiting for ENTER (no keyboard '
                               'on stdin?) -- recording stopped') from None
    except KeyboardInterrupt:
        print('\nInterrupted.')
        exit_code = 130
    except Exception as exc:  # noqa: BLE001 -- top-level CLI error reporting
        print(f'ERROR: {exc or type(exc).__name__}', file=sys.stderr)
        exit_code = 1
    finally:
        restore_terminal(terminal)
        if controller_active:
            try:
                link.switch_controllers([], [TEACH_CONTROLLER])
            except Exception as exc:  # noqa: BLE001
                print(f'WARNING: could not deactivate {TEACH_CONTROLLER}: {exc}', file=sys.stderr)
        if link.xhand_state is not None:
            try:
                link.hand_hold(hand_id, args.hand_index)
            except Exception as exc:  # noqa: BLE001
                print(f'WARNING: could not put the hand back in position mode: {exc}',
                      file=sys.stderr)
        if bag is not None and bag.poll() is None:
            os.killpg(bag.pid, signal.SIGINT)
            try:
                bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGKILL)
        if thresholds_raised:
            time.sleep(0.5)  # thresholds can only be set while no motion is running
            try:
                link.set_collision_thresholds(DEFAULT_JOINT_TORQUE_THRESHOLDS,
                                              DEFAULT_FORCE_THRESHOLDS)
                print('Collision thresholds restored to defaults.')
            except Exception as exc:  # noqa: BLE001
                print(f'WARNING: could not restore collision thresholds: {exc}', file=sys.stderr)

    if bag is not None and os.path.isdir(bag_dir):
        print(f'\nSaved bag: {bag_dir}')

    if recorded and exit_code == 0 and not args.no_return_home:
        try:
            if countdown('Recording done. Let go of the arm and step back -- returning to HOME '
                         'in (Ctrl+C skips):'):
                go_home(link, hand_id, args.hand_index, home_trajectory, 'Returning home')
                print('Back at home.')
        except KeyboardInterrupt:
            print('\nReturn to home interrupted -- the arm stopped where it was.')
        except Exception as exc:  # noqa: BLE001
            print(f'ERROR while returning home: {exc}', file=sys.stderr)
            exit_code = 1

    link.destroy_node()
    rclpy.shutdown()
    if recorded:
        print(f'Next: ros2 run teach_replay process_demo {demo_dir}')
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
