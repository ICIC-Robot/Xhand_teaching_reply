#!/usr/bin/env python3
"""Turn a recorded demo bag into replay-ready trajectory files.

Pipeline:
  1. read arm (q, controller timestamps) and hand (joint positions, header stamps) from the bag
  2. trim the idle stretch before the first and after the last motion (keeps a short margin)
  3. resample both onto one common timeline
  4. smooth every joint with a least-squares quintic B-spline (knots every --knot-spacing s)
  5. time-parameterize the smoothed path with toppra, with arm AND hand joints on the same
     path, so the hand stays synchronized with the arm however the timing changes;
     arm velocity/acceleration are capped at --vel-scale / --acc-scale of the Franka limits
  6. check max |dq|, |ddq| (and report jerk) against the limits, and joint positions against
     the hardware range; stop with an error if anything is out
  7. write  arm_trajectory.csv   (1 kHz: t, q1..q7, dq1..dq7)  -> trajectory_replay_controller
            hand_trajectory.csv  (t, 12 hand joints)           -> replay_demo's 83 Hz hand timer
            summary.yaml, and preview.png (raw vs processed)
"""

import argparse
import os
import sys

import numpy as np
import yaml
from scipy.interpolate import make_lsq_spline

# Franka Emika Panda joint limits (Franka datasheet / libfranka documentation).
ARM_Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
ARM_Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
ARM_DQ_MAX = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
ARM_DDQ_MAX = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
ARM_DDDQ_MAX = np.array([7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0])
POSITION_MARGIN = 0.05  # rad kept clear of the hardware range

ARM_TOPIC = '/teach_record_controller/joint_states'
HAND_TOPIC = '/xhand_control/xhand_state'
ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]

ARM_RATE_HZ = 1000.0
HAND_RATE_HZ = 200.0


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('demo_dir', help='Directory made by record_demo (contains bag/)')
    p.add_argument('--vel-scale', type=float, default=0.4,
                   help='Fraction of the Franka joint velocity limits to allow (default 0.4)')
    p.add_argument('--acc-scale', type=float, default=0.4,
                   help='Fraction of the Franka joint acceleration limits to allow (default 0.4)')
    p.add_argument('--hand-max-velocity', type=float, default=1.0,
                   help='XHand joint velocity cap, rad/s (default 1.0; no vendor limit exists)')
    p.add_argument('--hand-max-acceleration', type=float, default=5.0,
                   help='XHand joint acceleration cap, rad/s^2 (default 5.0)')
    p.add_argument('--knot-spacing', type=float, default=0.15,
                   help='B-spline knot spacing in seconds; larger = smoother (default 0.15)')
    p.add_argument('--hand-index', type=int, default=0,
                   help='Which entry of xhand_state.hand_states to use (default 0)')
    p.add_argument('--start', type=float, default=None,
                   help='Use the recording from this many seconds after its start '
                        '(default: automatic idle trimming)')
    p.add_argument('--end', type=float, default=None,
                   help='... up to this many seconds after its start (default: automatic)')
    p.add_argument('--no-plot', action='store_true', help='Skip preview.png')
    return p.parse_args()


# ------------------------------------------------------------------------ bag
def read_bag(bag_dir: str, hand_index: int):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    with open(os.path.join(bag_dir, 'metadata.yaml')) as f:
        storage_id = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (ARM_TOPIC, HAND_TOPIC):
        if topic not in types:
            raise RuntimeError(f'Bag has no {topic} (topics: {sorted(types)})')
    arm_cls, hand_cls = get_message(types[ARM_TOPIC]), get_message(types[HAND_TOPIC])

    arm_t, arm_q, hand_t, hand_q = [], [], [], []
    arm_order = hand_names = None
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        if topic == ARM_TOPIC:
            msg = deserialize_message(data, arm_cls)
            if arm_order is None:
                arm_order = [list(msg.name).index(n) for n in ARM_JOINTS]
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            arm_t.append(stamp if stamp > 0 else bag_ns * 1e-9)
            arm_q.append([msg.position[i] for i in arm_order])
        elif topic == HAND_TOPIC:
            msg = deserialize_message(data, hand_cls)
            hs = msg.hand_states[hand_index]
            if hand_names is None:
                hand_names = list(hs.name)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            hand_t.append(stamp if stamp > 0 else bag_ns * 1e-9)
            hand_q.append(list(hs.position))
    if len(arm_t) < 10 or len(hand_t) < 10:
        raise RuntimeError(f'Too few samples (arm {len(arm_t)}, hand {len(hand_t)})')
    arm_t, arm_q = _sorted_unique(np.array(arm_t), np.array(arm_q))
    hand_t, hand_q = _sorted_unique(np.array(hand_t), np.array(hand_q))
    return arm_t, arm_q, hand_t, hand_q, hand_names


def _sorted_unique(t, values):
    order = np.argsort(t, kind='stable')
    t, values = t[order], values[order]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], values[keep]


# --------------------------------------------------------------- processing
def trim_window(t, q_arm, q_hand, start, end, margin=0.5):
    """Return (t0, t1) on the shared timeline, auto-trimming idle at both ends."""
    if start is not None or end is not None:
        t0 = t[0] + (start or 0.0)
        t1 = t[0] + end if end is not None else t[-1]
        return t0, t1
    # Displacement from the resting pose at each end, not sample-to-sample speed: sensor
    # noise differentiated at 200 Hz looks like motion, a real move does not stay small.
    head = t < t[0] + 0.2
    tail = t > t[-1] - 0.2
    left_start = (np.max(np.abs(q_arm - np.median(q_arm[head], axis=0)), axis=1) > 0.01) | \
        (np.max(np.abs(q_hand - np.median(q_hand[head], axis=0)), axis=1) > 0.05)
    left_end = (np.max(np.abs(q_arm - np.median(q_arm[tail], axis=0)), axis=1) > 0.01) | \
        (np.max(np.abs(q_hand - np.median(q_hand[tail], axis=0)), axis=1) > 0.05)
    if not np.any(left_start):
        raise RuntimeError('No motion found in the recording (arm and hand both still).')
    first = np.argmax(left_start)
    last = len(t) - 1 - np.argmax(left_end[::-1])
    return max(t[0], t[first] - margin), min(t[-1], t[last] + margin)


def smooth(t, values, knot_spacing, degree=5):
    """Least-squares B-spline; endpoints pinned by repeated boundary knots."""
    n_interior = max(int((t[-1] - t[0]) / knot_spacing) - 1, 0)
    interior = np.linspace(t[0], t[-1], n_interior + 2)[1:-1]
    knots = np.concatenate([[t[0]] * (degree + 1), interior, [t[-1]] * (degree + 1)])
    return make_lsq_spline(t, values, knots, k=degree)


def time_parameterize(path_s, path_values, vel_limits, acc_limits):
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint

    ta.setup_logging('WARNING')
    path = ta.SplineInterpolator(path_s, path_values)
    instance = algo.TOPPRA(
        [constraint.JointVelocityConstraint(vel_limits),
         constraint.JointAccelerationConstraint(acc_limits)],
        path, gridpoints=np.linspace(path_s[0], path_s[-1], min(4000, 10 * len(path_s))))
    traj = instance.compute_trajectory(0.0, 0.0)
    if traj is None:
        raise RuntimeError('toppra found no feasible time parameterization')
    return traj


def main():
    args = parse_args()
    demo_dir = os.path.abspath(os.path.expanduser(args.demo_dir))
    bag_dir = os.path.join(demo_dir, 'bag')
    if not 0 < args.vel_scale <= 1 or not 0 < args.acc_scale <= 1:
        sys.exit('ERROR: --vel-scale and --acc-scale must be in (0, 1]')

    print(f'Reading {bag_dir} ...')
    arm_t, arm_q, hand_t, hand_q, hand_names = read_bag(bag_dir, args.hand_index)
    print(f'  arm: {len(arm_t)} samples over {arm_t[-1] - arm_t[0]:.1f}s '
          f'(~{len(arm_t) / (arm_t[-1] - arm_t[0]):.0f} Hz); hand: {len(hand_t)} samples '
          f'(~{len(hand_t) / (hand_t[-1] - hand_t[0]):.0f} Hz), {len(hand_names)} joints')

    # Shared timeline at 200 Hz, inside both recordings.
    t_lo, t_hi = max(arm_t[0], hand_t[0]), min(arm_t[-1], hand_t[-1])
    if t_hi - t_lo < 1.0:
        raise SystemExit('ERROR: arm and hand recordings overlap by less than 1 s')
    grid = np.arange(t_lo, t_hi, 1.0 / 200.0)
    arm_g = np.column_stack([np.interp(grid, arm_t, arm_q[:, j]) for j in range(7)])
    hand_g = np.column_stack([np.interp(grid, hand_t, hand_q[:, j])
                              for j in range(hand_q.shape[1])])

    t0, t1 = trim_window(grid, arm_g, hand_g, args.start, args.end)
    sel = (grid >= t0) & (grid <= t1)
    grid, arm_g, hand_g = grid[sel], arm_g[sel], hand_g[sel]
    s = grid - grid[0]
    print(f'  using {s[-1]:.2f}s of motion ({t0 - t_lo:.2f}s .. {t1 - t_lo:.2f}s of the recording)')

    both = np.hstack([arm_g, hand_g])
    spline = smooth(s, both, args.knot_spacing)
    path_s = np.arange(0.0, s[-1], 0.02)
    if path_s[-1] < s[-1]:
        path_s = np.append(path_s, s[-1])
    path_values = spline(path_s)
    smooth_err = np.max(np.abs(spline(s) - both), axis=0)
    print(f'  B-spline smoothing (knots every {args.knot_spacing}s): max deviation arm '
          f'{smooth_err[:7].max():.4f} rad, hand {smooth_err[7:].max():.4f} rad')

    n_hand = hand_g.shape[1]
    vel_lim = np.concatenate([ARM_DQ_MAX * args.vel_scale, np.full(n_hand, args.hand_max_velocity)])
    acc_lim = np.concatenate([ARM_DDQ_MAX * args.acc_scale,
                              np.full(n_hand, args.hand_max_acceleration)])
    print('  toppra time-parameterization ...')
    traj = time_parameterize(path_s, path_values, vel_lim, acc_lim)

    # Sample on an exact 1 kHz grid (the replay controller requires evenly spaced rows), so
    # the duration is rounded up to whole milliseconds. If the sampled result overshoots the
    # caps anywhere, stretch uniformly first (exact: v scales by 1/k, a by 1/k^2).
    def sample(stretch):
        n = int(np.ceil(traj.duration * stretch * ARM_RATE_HZ))
        t_out = np.arange(n + 1) / ARM_RATE_HZ
        k = t_out[-1] / traj.duration
        tau = np.minimum(t_out / k, traj.duration)
        return t_out, traj(tau), traj(tau, 1) / k, traj(tau, 2) / k ** 2

    tt, q, qd, qdd = sample(1.0)
    k = max(np.max(np.abs(qd) / vel_lim), np.sqrt(np.max(np.abs(qdd) / acc_lim)))
    if k > 1.0:
        print(f'  sampled trajectory exceeded the caps by up to {k:.3f}x; stretching time by that')
        tt, q, qd, qdd = sample(k * 1.001)
    duration = tt[-1]
    qd[0] = qd[-1] = 0.0

    arm_q_out, arm_dq_out, arm_ddq = q[:, :7], qd[:, :7], qdd[:, :7]
    arm_jerk = np.diff(arm_ddq, axis=0) * ARM_RATE_HZ

    # --------------------------------------------------------------- checks
    problems = []
    ratio_v = np.max(np.abs(arm_dq_out), axis=0) / ARM_DQ_MAX
    ratio_a = np.max(np.abs(arm_ddq), axis=0) / ARM_DDQ_MAX
    ratio_j = np.max(np.abs(arm_jerk), axis=0) / ARM_DDDQ_MAX
    lo, hi = ARM_Q_MIN + POSITION_MARGIN, ARM_Q_MAX - POSITION_MARGIN
    for j in range(7):
        qmin, qmax = arm_q_out[:, j].min(), arm_q_out[:, j].max()
        if qmin < lo[j] or qmax > hi[j]:
            problems.append(f'{ARM_JOINTS[j]} range [{qmin:.3f}, {qmax:.3f}] leaves the safe '
                            f'hardware range [{lo[j]:.3f}, {hi[j]:.3f}] rad')
    if np.any(ratio_v > 1.0) or np.any(ratio_a > 1.0):
        problems.append('velocity/acceleration above the Franka limits')
    hand_v = np.max(np.abs(qd[:, 7:]), axis=0)

    print(f'\nResult: {duration:.2f}s (demonstration was {s[-1]:.2f}s)')
    print('  joint     max|dq| (rad/s, % of limit)   max|ddq| (rad/s^2, %)   max|jerk| (%)')
    for j in range(7):
        print(f'  {ARM_JOINTS[j]}  {ratio_v[j] * ARM_DQ_MAX[j]:7.3f} ({ratio_v[j]:5.0%})'
              f'            {ratio_a[j] * ARM_DDQ_MAX[j]:7.3f} ({ratio_a[j]:5.0%})'
              f'          {ratio_j[j]:5.0%}')
    print(f'  hand max|dq| {hand_v.max():.3f} rad/s (cap {args.hand_max_velocity})')
    if np.any(ratio_j > 1.0):
        print('  NOTE: jerk exceeds the datasheet value on some joint; harmless for the torque-'
              'mode replay controller, but consider a larger --knot-spacing.')

    # --------------------------------------------------------------- output
    arm_file = os.path.join(demo_dir, 'arm_trajectory.csv')
    header = 't,' + ','.join(f'q{i}' for i in range(1, 8)) + ',' + \
        ','.join(f'dq{i}' for i in range(1, 8))
    np.savetxt(arm_file, np.column_stack([tt, arm_q_out, arm_dq_out]), delimiter=',',
               header=header, comments='', fmt='%.9f')
    th = np.linspace(0.0, duration, int(round(duration * HAND_RATE_HZ)) + 1)
    hand_out = np.column_stack([np.interp(th, tt, q[:, 7 + j]) for j in range(n_hand)])
    hand_file = os.path.join(demo_dir, 'hand_trajectory.csv')
    np.savetxt(hand_file, np.column_stack([th, hand_out]), delimiter=',',
               header='t,' + ','.join(hand_names), comments='', fmt='%.9f')

    summary = {
        'source_bag': bag_dir,
        'demonstration_duration_s': float(s[-1]),
        'replay_duration_s': float(duration),
        'arm_rate_hz': ARM_RATE_HZ,
        'hand_rate_hz': HAND_RATE_HZ,
        'hand_index': args.hand_index,
        'hand_joint_names': hand_names,
        'vel_scale': args.vel_scale,
        'acc_scale': args.acc_scale,
        'knot_spacing_s': args.knot_spacing,
        'max_velocity_fraction': [float(x) for x in ratio_v],
        'max_acceleration_fraction': [float(x) for x in ratio_a],
        'max_jerk_fraction': [float(x) for x in ratio_j],
        'arm_start_q': [float(x) for x in arm_q_out[0]],
        'hand_start_q': [float(x) for x in hand_out[0]],
        'ok': not problems,
        'problems': problems,
    }
    with open(os.path.join(demo_dir, 'summary.yaml'), 'w') as f:
        yaml.safe_dump(summary, f, sort_keys=False)

    if not args.no_plot:
        _plot(os.path.join(demo_dir, 'preview.png'), s, arm_g, path_s, path_values[:, :7])

    print(f'\nWrote {arm_file}\n      {hand_file}\n      {os.path.join(demo_dir, "summary.yaml")}')
    if problems:
        print('\nNOT REPLAYABLE:', file=sys.stderr)
        for p in problems:
            print(f'  - {p}', file=sys.stderr)
        sys.exit(1)
    print(f'\nNext: ros2 run teach_replay replay_demo {demo_dir} --dry-run')


def _plot(path, t_raw, q_raw, t_smooth, q_smooth):
    """Recorded vs smoothed arm path, both on the demonstration's own time axis."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(7, 1, figsize=(10, 12), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t_raw, q_raw[:, j], color='0.6', lw=1, label='recorded')
        ax.plot(t_smooth, q_smooth[:, j], color='C0', lw=1, label='smoothed path')
        ax.set_ylabel(f'q{j + 1}')
    axes[0].legend(loc='upper right')
    axes[-1].set_xlabel('demonstration time (s); replay timing comes from toppra')
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


if __name__ == '__main__':
    main()
