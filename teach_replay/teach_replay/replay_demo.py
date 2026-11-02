#!/usr/bin/env python3
"""Replay a processed demo: arm through trajectory_replay_controller, hand at 83 Hz.

Sequence:
  1. check the arm is IDLE and the hand is publishing; restore default collision thresholds
  2. deactivate any active arm controller, (re)load trajectory_replay_controller with this
     demo's arm_trajectory.csv and configure it (the file is parsed here, outside the 1 kHz loop)
  3. hand -> position mode at normal stiffness, holding its current pose
  4. --dry-run stops here. Otherwise ENTER activates the controller:
       approach  -- slow quintic move of the arm to the trajectory start; the hand blends
                    from its current pose to the demo's start pose on the same progress value
       main      -- the arm tracks the 1 kHz file; a 83 Hz timer sends the hand the pose of
                    hand_trajectory.csv at the controller's own trajectory time (~/progress),
                    so the hand follows the arm's real timeline, including pauses
       done      -- both hold the final pose; the controller is deactivated
  5. after a 3 s countdown (Ctrl+C skips; --no-return-home disables), the arm and the hand
     go back to the factory home pose
  Ctrl+C at any time deactivates the controller (the arm stops) and holds the hand.
"""

import argparse
import os
import sys

import numpy as np
import rclpy
import yaml

from teach_replay.motion import countdown, execute, go_home, prepare_replay_controller, quintic
from teach_replay.ros_helpers import (
    DEFAULT_FORCE_THRESHOLDS,
    DEFAULT_JOINT_TORQUE_THRESHOLDS,
    REPLAY_CONTROLLER,
    RobotLink,
)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('demo_dir', help='Directory processed by process_demo')
    p.add_argument('--dry-run', action='store_true',
                   help='Check everything and load the trajectory, but do not move')
    p.add_argument('--yes', action='store_true', help='Start without waiting for ENTER')
    p.add_argument('--no-return-home', action='store_true',
                   help='Stay at the final pose instead of returning to the factory home')
    p.add_argument('--hand-kp', type=float, default=100.0,
                   help='XHand position gain during replay (default 100, the normal value)')
    return p.parse_args()


class HandTrajectory:
    def __init__(self, path):
        with open(path) as f:
            self.names = f.readline().strip().split(',')[1:]
        data = np.loadtxt(path, delimiter=',', skiprows=1)
        self.t, self.q = data[:, 0], data[:, 1:]

    def at(self, t):
        return [float(np.interp(t, self.t, self.q[:, j])) for j in range(self.q.shape[1])]


def main():
    args = parse_args()
    demo_dir = os.path.abspath(os.path.expanduser(args.demo_dir))
    arm_file = os.path.join(demo_dir, 'arm_trajectory.csv')
    hand_file = os.path.join(demo_dir, 'hand_trajectory.csv')
    with open(os.path.join(demo_dir, 'summary.yaml')) as f:
        summary = yaml.safe_load(f)
    if not summary.get('ok', False):
        sys.exit(f'ERROR: process_demo marked this demo NOT replayable: {summary.get("problems")}')
    hand_traj = HandTrajectory(hand_file)
    arm_start = np.array(summary['arm_start_q'])

    rclpy.init()
    link = RobotLink('teach_replay_replay')
    hand_index = summary.get('hand_index', 0)
    exit_code = 0
    try:
        link.wait_for_states()
        link.require_arm_ready()
        hand_id = link.hand_ids()[hand_index]
        if link.hand_joint_names(hand_index) != hand_traj.names:
            raise RuntimeError(f'Hand joints {link.hand_joint_names(hand_index)} differ from the '
                               f'recorded ones {hand_traj.names}')

        link.set_collision_thresholds(DEFAULT_JOINT_TORQUE_THRESHOLDS, DEFAULT_FORCE_THRESHOLDS)
        prepare_replay_controller(link, arm_file)

        offset = np.abs(np.array(link.arm_positions()) - arm_start)
        print(f'Demo: {summary["replay_duration_s"]:.1f}s replay (recorded '
              f'{summary["demonstration_duration_s"]:.1f}s of motion)')
        print(f'Arm is {offset.max():.3f} rad (joint {int(offset.argmax()) + 1}) from the start '
              f'pose; it will move there first at <= 0.2 rad/s.')
        link.hand_hold(hand_id, hand_index)
        print(f'{REPLAY_CONTROLLER} loaded and configured; hand holding its pose.')

        if args.dry_run:
            print('\n--dry-run: everything checked, nothing moved.')
            return
        if not args.yes:
            print('\nRobot light solid blue, nobody in reach, hand on the e-stop?  '
                  'Press ENTER to start (Ctrl+C aborts).')
            input()

        link.spin_for(0.1)
        hand_from = link.hand_positions(hand_index)
        hand_first = hand_traj.at(0.0)
        hand_last = hand_traj.at(hand_traj.t[-1])

        def hand_target(phase, value):
            if phase == 0:  # approach: blend from the current pose to the demo's first pose
                b = quintic(value)
                return [a + (h - a) * b for a, h in zip(hand_from, hand_first)]
            if phase == 1:  # replay: the demo pose at the controller's own trajectory time
                return hand_traj.at(value)
            return hand_last

        execute(link, hand_id, hand_traj.names, hand_target, hand_kp=args.hand_kp,
                label='Replaying')
        print('Replay finished.')

        if not args.no_return_home:
            try:
                if countdown('Returning arm and hand to HOME in (Ctrl+C skips):'):
                    go_home(link, hand_id, hand_index,
                            os.path.join(demo_dir, 'go_home_trajectory.csv'), 'Returning home')
                    print('Back at home.')
            except KeyboardInterrupt:
                print('\nReturn to home interrupted -- the arm stopped where it was.')
    except KeyboardInterrupt:
        print('\nInterrupted -- stopped.')
        exit_code = 130
    except Exception as exc:  # noqa: BLE001 -- top-level CLI error reporting
        print(f'ERROR: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        if link.xhand_state is not None:
            try:
                link.hand_hold(link.hand_ids()[hand_index], hand_index)
            except Exception:  # noqa: BLE001
                pass
        link.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
