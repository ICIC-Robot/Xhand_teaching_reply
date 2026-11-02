# teach_replay

[English](#english) | [中文](#中文) — part of the [teach & replay workflow](../README.md)

---

<a id="english"></a>
## English

Command-line tools that drive the whole workflow. They talk to the running Franka driver
(controller_manager services, `franka_robot_state_broadcaster`, the param service for collision
thresholds) and to the XHand driver (`/xhand_control/xhand_state`, `/xhand_control/xhand_command`).
The arm motion itself is done by [teach_replay_controllers](../teach_replay_controllers/README.md).

Run every command after `source ~/xarm_ws/install/setup.bash`, with the Franka driver (Terminal 1)
and the XHand driver (Terminal 2) running — except `process_demo`, which is offline.

### Commands

#### `go_home` — move to the factory home pose
```bash
ros2 run teach_replay go_home          # asks for ENTER first
ros2 run teach_replay go_home --yes    # no confirmation
```
Arm → Franka ready pose `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`; XHand → factory zero (all joints 0 rad,
hand open). At most 0.2 rad/s per arm joint, at least 3 s; the hand blends on the same timeline.
Does nothing if already home. Ctrl+C stops the arm.

#### `record_demo` — record a demonstration
```bash
ros2 run teach_replay record_demo <name>
```
1. Checks that the arm is IDLE and the hand is publishing.
2. 3 s countdown, then moves to home (same as `go_home`).
3. Raises the collision thresholds so pushing by hand is not judged a collision
   (joint torque `[60, 60, 50, 50, 30, 25, 20]` Nm, force `[80, 80, 80, 40, 40, 40]`).
4. Starts `ros2 bag record` of `/teach_record_controller/joint_states` and
   `/xhand_control/xhand_state` (mcap if installed, otherwise sqlite3).
5. XHand → passive mode (zero stiffness): pose the fingers by hand.
6. Activates `teach_record_controller`: push the arm by hand.
7. ENTER (or Ctrl+C) stops: controller off, hand holds its pose, bag closed, default thresholds
   (`[20, 20, 18, 18, 16, 14, 12]` Nm, `[20, 20, 20, 25, 25, 25]`) restored.
8. 3 s countdown (let go, step back; Ctrl+C skips), then returns to home.

| Option | Meaning |
|---|---|
| `--output-dir DIR` | Default `~/xarm_ws/demos` |
| `--no-start-home` | Record from the current pose |
| `--no-return-home` | Stay where the recording ended |
| `--keep-hand-stiff` | Do not make the hand passive |
| `--keep-collision-thresholds` | Do not raise the thresholds |
| `--hand-index N` | Which hand in `xhand_state` (default 0) |

An existing name is refused, so no demo is ever overwritten.

#### `process_demo` — offline processing (the robot does not move)
```bash
ros2 run teach_replay process_demo ~/xarm_ws/demos/<name>
```
1. Reads the bag (arm at ~500 Hz, hand at ~100 Hz, using their header timestamps).
2. Trims the idle stretch before the first and after the last motion (motion = arm more than
   0.01 rad or hand more than 0.05 rad away from its resting pose; keeps 0.5 s margins).
3. Resamples arm and hand onto one 200 Hz timeline.
4. Smooths every joint with a least-squares quintic B-spline (knots every `--knot-spacing` s).
5. Time-parameterizes the smoothed path with **toppra**. The arm and hand joints are on the **same
   path**, so the hand stays synchronized however the timing changes. Arm velocity and acceleration
   are capped at `--vel-scale` / `--acc-scale` of the Franka limits.
6. Samples on an exact 1 kHz grid and checks max |dq|, |ddq| (jerk is reported) and that every joint
   stays at least 0.05 rad inside the Franka hardware range. If anything fails it prints
   `NOT REPLAYABLE` and exits with an error.

| Option | Default | Meaning |
|---|---|---|
| `--vel-scale` | `0.4` | Fraction of the Franka velocity limits (≤ 1) |
| `--acc-scale` | `0.4` | Fraction of the Franka acceleration limits (≤ 1) |
| `--hand-max-velocity` | `1.0` | rad/s (no vendor limit exists) |
| `--hand-max-acceleration` | `5.0` | rad/s² |
| `--knot-spacing` | `0.15` | s; larger = smoother, less detail |
| `--start` / `--end` | auto | Seconds from the start of the recording, instead of auto-trimming |
| `--no-plot` | | Skip `preview.png` |

Note: toppra removes stretches where both the arm and the hand are completely still.

#### `replay_demo` — replay a processed demo
```bash
ros2 run teach_replay replay_demo ~/xarm_ws/demos/<name> --dry-run   # checks + loads, no motion
ros2 run teach_replay replay_demo ~/xarm_ws/demos/<name>             # asks for ENTER, then moves
```
1. Checks that `summary.yaml` says `ok`, the arm is IDLE, and the hand joints match the recording.
2. Restores the default collision thresholds, loads `trajectory_replay_controller` with
   `arm_trajectory.csv` and configures it. The hand holds its pose (position mode, kp 100).
3. `--dry-run` stops here. Otherwise ENTER starts: the arm moves slowly to the start of the
   trajectory while the hand blends to its start pose; then the arm tracks the 1 kHz trajectory and
   an **83 Hz** timer sends the hand the pose of `hand_trajectory.csv` at the controller's own
   trajectory time (`/trajectory_replay_controller/progress`).
4. After a successful replay: 3 s countdown (Ctrl+C skips), then returns to home.

If the arm stops (reflex / user stop), the hand stops getting new targets at once, and the command
exits with the recovery hint.

| Option | Meaning |
|---|---|
| `--dry-run` | Check and load only |
| `--yes` | Start without ENTER |
| `--no-return-home` | Stay at the final pose (returning home opens the hand) |
| `--hand-kp` | Hand position gain during replay (default 100) |

### Output files (`~/xarm_ws/demos/<name>/`)

| File | Written by | Content |
|---|---|---|
| `bag/` | record_demo | Raw ros2 bag |
| `arm_trajectory.csv` | process_demo | 1 kHz: `t, q1..q7, dq1..dq7` — input of the replay controller |
| `hand_trajectory.csv` | process_demo | 200 Hz: `t` + 12 hand joints (header has the names) |
| `summary.yaml` | process_demo | Durations, % of limits per joint, start poses, `ok` / `problems` |
| `preview.png` | process_demo | Recorded vs smoothed arm path |
| `go_home_trajectory.csv` | record_demo / replay_demo | Temporary single-pose file for going home |

`go_home` writes its temporary file to `~/xarm_ws/demos/.go_home_trajectory.csv`.

### Source files

| File | Content |
|---|---|
| `teach_replay/record_demo.py` | Recording |
| `teach_replay/process_demo.py` | B-spline + toppra + checks; Franka limits table at the top |
| `teach_replay/replay_demo.py` | Replay with the 83 Hz hand timer |
| `teach_replay/go_home.py` | Go home |
| `teach_replay/motion.py` | Shared: prepare/run the replay controller with the hand, go home, countdown |
| `teach_replay/ros_helpers.py` | Shared: services, topics, home poses, collision thresholds |
| `test/` | Tests (synthetic bag end to end, file formats) |

### Build and test
```bash
cd ~/xarm_ws && source /opt/ros/humble/setup.bash
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:) colcon build --packages-select teach_replay
source install/setup.bash
PYTHONPATH=src/integration/teach_replay:$PYTHONPATH /usr/bin/python3 -m pytest -q src/integration/teach_replay/test
```
Requires `toppra` for `/usr/bin/python3` (`/usr/bin/python3 -m pip install --user toppra`). Use the
system Python, not conda: conda's Python cannot load ROS 2 Humble.

---

<a id="中文"></a>
## 中文

驱动整个流程的命令行工具。它们和正在运行的 Franka 驱动通信（controller_manager 服务、`franka_robot_state_broadcaster`、设置碰撞阈值的参数服务），也和 XHand 驱动通信（`/xhand_control/xhand_state`、`/xhand_control/xhand_command`）。手臂的运动本身由 [teach_replay_controllers](../teach_replay_controllers/README.md) 完成。

每个命令运行前都要先 `source ~/xarm_ws/install/setup.bash`，并且 Franka 驱动（终端 1）和 XHand 驱动（终端 2）要在运行。`process_demo` 是离线处理，不需要驱动。

### 命令

#### `go_home` — 移动到出厂 home 姿态
```bash
ros2 run teach_replay go_home          # 先等你按回车
ros2 run teach_replay go_home --yes    # 不需要确认
```
手臂 → Franka ready 姿态 `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`；XHand → 出厂零位（所有关节 0 rad，手张开）。手臂每个关节最快 0.2 rad/s，至少用 3 秒；手在同一时间轴上同步插值。已经在 home 就什么都不做。Ctrl+C 可以让手臂停下。

#### `record_demo` — 录制示范
```bash
ros2 run teach_replay record_demo <名字>
```
1. 检查手臂处于 IDLE 状态、手在发布数据。
2. 倒数 3 秒，然后移动到 home（和 `go_home` 一样）。
3. 调高碰撞阈值，用手推动时不会被判定为碰撞（关节力矩 `[60, 60, 50, 50, 30, 25, 20]` Nm，力 `[80, 80, 80, 40, 40, 40]`）。
4. 启动 `ros2 bag record`，录制 `/teach_record_controller/joint_states` 和 `/xhand_control/xhand_state`（装了 mcap 就用 mcap，否则用 sqlite3）。
5. XHand 切换到被动模式（零刚度）：用手摆弄手指。
6. 激活 `teach_record_controller`：用手推动手臂。
7. 按回车（或 Ctrl+C）结束：停用 controller，手保持当前姿态，关闭 bag，恢复默认阈值（`[20, 20, 18, 18, 16, 14, 12]` Nm，`[20, 20, 20, 25, 25, 25]`）。
8. 倒数 3 秒（松手离开；Ctrl+C 跳过），然后回到 home。

| 选项 | 含义 |
|---|---|
| `--output-dir DIR` | 默认 `~/xarm_ws/demos` |
| `--no-start-home` | 从当前姿态开始录制 |
| `--no-return-home` | 录制结束后停在原地 |
| `--keep-hand-stiff` | 不把手切换到被动模式 |
| `--keep-collision-thresholds` | 不调高碰撞阈值 |
| `--hand-index N` | 用 `xhand_state` 里的第几只手（默认 0） |

名字已经存在时会拒绝录制，所以不会覆盖已有的示范。

#### `process_demo` — 离线处理（机械臂不会动）
```bash
ros2 run teach_replay process_demo ~/xarm_ws/demos/<名字>
```
1. 读取 bag（手臂约 500 Hz，手约 100 Hz，使用消息头里的时间戳）。
2. 裁掉第一个动作之前和最后一个动作之后的静止段（判断标准：手臂离静止姿态超过 0.01 rad，或手超过 0.05 rad；前后各保留 0.5 秒）。
3. 把手臂和手的数据重采样到同一条 200 Hz 时间轴上。
4. 每个关节用最小二乘五次 B 样条平滑（节点间隔为 `--knot-spacing` 秒）。
5. 用 **toppra** 给平滑后的路径重新分配时间。手臂和手的关节放在**同一条路径**上，所以无论时间怎么调整，手都和手臂保持同步。手臂的速度和加速度上限是 Franka 限值乘以 `--vel-scale` / `--acc-scale`。
6. 在严格的 1 kHz 时间网格上采样，检查最大 |dq|、|ddq|（同时报告加加速度），以及每个关节是否都离 Franka 硬件极限至少 0.05 rad。任何一项不通过就打印 `NOT REPLAYABLE` 并报错退出。

| 选项 | 默认值 | 含义 |
|---|---|---|
| `--vel-scale` | `0.4` | 使用 Franka 速度限值的比例（≤ 1） |
| `--acc-scale` | `0.4` | 使用 Franka 加速度限值的比例（≤ 1） |
| `--hand-max-velocity` | `1.0` | rad/s（厂商没有公布限值） |
| `--hand-max-acceleration` | `5.0` | rad/s² |
| `--knot-spacing` | `0.15` | 秒；越大越平滑，细节越少 |
| `--start` / `--end` | 自动 | 从录制开始算起的秒数，用来代替自动裁剪 |
| `--no-plot` | | 不生成 `preview.png` |

注意：toppra 会去掉手臂和手都完全静止的时间段。

#### `replay_demo` — 回放处理好的示范
```bash
ros2 run teach_replay replay_demo ~/xarm_ws/demos/<名字> --dry-run   # 只检查和加载，不会动
ros2 run teach_replay replay_demo ~/xarm_ws/demos/<名字>             # 等你按回车后开始运动
```
1. 检查 `summary.yaml` 显示 `ok`、手臂处于 IDLE 状态、手的关节和录制时一致。
2. 恢复默认碰撞阈值，加载 `trajectory_replay_controller` 并读入 `arm_trajectory.csv`，完成配置。手保持当前姿态（位置模式，kp 100）。
3. `--dry-run` 到这里就结束。否则按回车开始：手臂先慢慢移动到轨迹起点，同时手插值到起始姿态；然后手臂跟踪 1 kHz 轨迹，一个 **83 Hz** 的定时器按照 controller 自己的轨迹时间（`/trajectory_replay_controller/progress`），把 `hand_trajectory.csv` 里对应的姿态发给手。
4. 回放成功结束后：倒数 3 秒（Ctrl+C 跳过），然后回到 home。

如果手臂停下（reflex 或用户急停），手会立刻停止接收新的目标，程序退出并提示怎么恢复。

| 选项 | 含义 |
|---|---|
| `--dry-run` | 只检查和加载 |
| `--yes` | 不按回车直接开始 |
| `--no-return-home` | 停在最后的姿态（回 home 时手会张开） |
| `--hand-kp` | 回放时手的位置增益（默认 100） |

### 输出文件（`~/xarm_ws/demos/<名字>/`）

| 文件 | 由谁生成 | 内容 |
|---|---|---|
| `bag/` | record_demo | 原始 ros2 bag |
| `arm_trajectory.csv` | process_demo | 1 kHz：`t, q1..q7, dq1..dq7`，回放 controller 的输入 |
| `hand_trajectory.csv` | process_demo | 200 Hz：`t` 和 12 个手部关节（表头里有关节名） |
| `summary.yaml` | process_demo | 时长、每个关节占限值的百分比、起始姿态、`ok` / `problems` |
| `preview.png` | process_demo | 原始路径与平滑后路径的对比图 |
| `go_home_trajectory.csv` | record_demo / replay_demo | 回 home 用的临时单点轨迹文件 |

`go_home` 的临时文件写在 `~/xarm_ws/demos/.go_home_trajectory.csv`。

### 源码文件

| 文件 | 内容 |
|---|---|
| `teach_replay/record_demo.py` | 录制 |
| `teach_replay/process_demo.py` | B 样条 + toppra + 检查；文件开头是 Franka 限值表 |
| `teach_replay/replay_demo.py` | 回放，带 83 Hz 手部定时器 |
| `teach_replay/go_home.py` | 回 home |
| `teach_replay/motion.py` | 公共部分：准备和运行回放 controller（带手部同步）、回 home、倒数 |
| `teach_replay/ros_helpers.py` | 公共部分：服务、话题、home 姿态、碰撞阈值 |
| `test/` | 测试（合成 bag 端到端测试、文件格式检查） |

### 编译与测试
```bash
cd ~/xarm_ws && source /opt/ros/humble/setup.bash
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:) colcon build --packages-select teach_replay
source install/setup.bash
PYTHONPATH=src/integration/teach_replay:$PYTHONPATH /usr/bin/python3 -m pytest -q src/integration/teach_replay/test
```
需要给 `/usr/bin/python3` 装好 `toppra`（`/usr/bin/python3 -m pip install --user toppra`）。要用系统 Python，不要用 conda：conda 的 Python 加载不了 ROS 2 Humble。
