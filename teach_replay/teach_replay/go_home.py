#!/usr/bin/env python3
"""Move the Franka arm and the XHand to their factory home pose.

  arm:  Franka's standard ready pose [0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]
  hand: its factory-calibrated zero, every joint at 0 rad (open and flat)

The move is slow (no arm joint faster than 0.2 rad/s, at least 3 s) and goes through
trajectory_replay_controller; the hand blends to home on the same timeline. The script
asks for ENTER first (--yes skips that). Ctrl+C stops the arm at any time.
"""

import argparse
import os
import sys

import numpy as np
import rclpy

from teach_replay.motion import go_home
from teach_replay.ros_helpers import (
    DEFAULT_FORCE_THRESHOLDS,
    DEFAULT_JOINT_TORQUE_THRESHOLDS,
    FRANKA_HOME,
    RobotLink,
)

SCRATCH_FILE = os.path.expanduser('~/xarm_ws/demos/.go_home_trajectory.csv')


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--yes', action='store_true', help='Start without waiting for ENTER')
    p.add_argument('--hand-index', type=int, default=0,
                   help='Which entry of xhand_state.hand_states to move (default 0)')
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(os.path.dirname(SCRATCH_FILE), exist_ok=True)
    rclpy.init()
    link = RobotLink('teach_replay_go_home')
    exit_code = 0
    try:
        link.wait_for_states()
        link.require_arm_ready()
        hand_id = link.hand_ids()[args.hand_index]
        offset = np.abs(np.array(link.arm_positions()) - np.array(FRANKA_HOME))
        hand_offset = np.max(np.abs(link.hand_positions(args.hand_index)))
        print(f'Arm is {offset.max():.3f} rad (joint {int(offset.argmax()) + 1}) from home; '
              f'hand is {hand_offset:.3f} rad from home.')
        if offset.max() < 0.01 and hand_offset < 0.02:
            print('Already at home.')
            return
        link.set_collision_thresholds(DEFAULT_JOINT_TORQUE_THRESHOLDS, DEFAULT_FORCE_THRESHOLDS)
        if not args.yes:
            print('\nRobot light solid blue, nobody in reach, hand on the e-stop?  '
                  'Press ENTER to move to home (Ctrl+C aborts).')
            input()
        go_home(link, hand_id, args.hand_index, SCRATCH_FILE)
        print('At home.')
    except KeyboardInterrupt:
        print('\nInterrupted -- the arm stopped where it was.')
        exit_code = 130
    except EOFError:
        print('ERROR: no keyboard on stdin; use --yes to start without ENTER', file=sys.stderr)
        exit_code = 1
    except Exception as exc:  # noqa: BLE001 -- top-level CLI error reporting
        print(f'ERROR: {exc or type(exc).__name__}', file=sys.stderr)
        exit_code = 1
    finally:
        if link.xhand_state is not None:
            try:
                link.hand_hold(link.hand_ids()[args.hand_index], args.hand_index)
            except Exception:  # noqa: BLE001
                pass
        link.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
