# teach_replay_controllers

[English](#english) | [中文](#中文) — part of the [teach & replay workflow](../README.md)

---

<a id="english"></a>
## English

Two ros2_control controller plugins for the Franka Panda (single arm, `arm_id: panda`). Both use
the **effort** (torque) command interfaces and the position + velocity state interfaces. In
libfranka torque mode the robot already compensates gravity, so the torques here are on top of that.
They are normally started by the `teach_replay` tools, not by hand.

### TeachRecordController — recording

`teach_replay_controllers/TeachRecordController`, started by `record_demo`.

- Starts from the gravity compensation controller: commanding zero torque leaves the arm free to
  be pushed by hand.
- Adds joint damping **τ = −D · dq**, clamped to ±`tau_max`, so a fast push is resisted before it
  can approach the joint velocity limits.
- Publishes `~/joint_states` (`/teach_record_controller/joint_states`, `sensor_msgs/JointState`:
  position = q, velocity = dq, effort = commanded τ, `header.stamp` = controller time) through a
  `realtime_tools::RealtimePublisher` at `publish_rate`. This is the arm topic that goes into the bag.

| Parameter | Default | Meaning |
|---|---|---|
| `arm_id` | `panda` | Joint name prefix |
| `damping` | `[5, 5, 5, 5, 2, 2, 1]` | D, Nm per rad/s |
| `tau_max` | `[20, 20, 20, 20, 6, 6, 6]` | Damping torque clamp, Nm |
| `publish_rate` | `500` | Hz, ≤ 1000 |

### TrajectoryReplayController — replay and moving to a pose

`teach_replay_controllers/TrajectoryReplayController`, started by `replay_demo`, `go_home` and
`record_demo` (for going home).

- **Loads the trajectory file in `on_configure`**, outside the 1 kHz loop. `on_activate` runs inside
  the loop, so nothing slow happens there. To use another file, unload, set `trajectory_file`, load
  and configure again (`teach_replay` does this for you).
- **Phase 0 — approach:** a quintic move from the pose at activation to the first sample, slow
  enough that no joint exceeds `approach_max_velocity`, and never shorter than `min_approach_duration`.
- **Phase 1 — replay:** the file indexed by time, linearly interpolated between samples. Fully
  deterministic: the commanded target is the processed trajectory, point by point.
- **Phase 2 — done:** holds the last sample.
- **Command:** τ = K (q_d − q) + D (dq_d − dq), clamped to ±`tau_max`.
- **Safety:** if any joint lags its target by more than `max_tracking_error`, the timeline stops
  advancing (the target waits for the arm) until the error is back under half of that. A stalled
  loop never makes the target jump: one cycle advances time by at most 5 ms.
- **Progress:** `~/progress` (`std_msgs/Float64MultiArray`) at `progress_rate`:
  `[phase, value, paused, main_duration]`, with value = approach progress 0..1 in phase 0, and the
  trajectory time in seconds in phases 1 and 2. The hand follows this topic, so it stays on the arm's
  real timeline, including pauses.
- Moving to a fixed pose (going home) is a file with that single pose: the approach phase does the move.

| Parameter | Default | Meaning |
|---|---|---|
| `arm_id` | `panda` | Joint name prefix |
| `trajectory_file` | `""` | Path to the CSV (set by the tools) |
| `k_gains` | `[600, 600, 600, 600, 250, 150, 50]` | Stiffness K, Nm/rad |
| `d_gains` | `[50, 50, 50, 50, 30, 25, 15]` | Damping D, Nm·s/rad |
| `tau_max` | `[60, 60, 60, 60, 10, 10, 10]` | Torque clamp, Nm |
| `approach_max_velocity` | `0.2` | rad/s, move-to-start speed cap |
| `min_approach_duration` | `3.0` | s |
| `max_tracking_error` | `0.15` | rad |
| `progress_rate` | `100` | Hz |

### Trajectory file format

CSV with one header line, then evenly spaced rows (written by `process_demo`; 1 kHz):
```
t,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7
0.000000000,...
0.001000000,...
```
15 columns, finite values, at least 2 rows, constant spacing (±1 µs) — otherwise configure fails and
Terminal 1 prints why.

### Configuration and build

Both controllers are registered in `src/robots/franka_arm/franka_bringup/config/controllers.yaml`
(type entries under `controller_manager`, parameters in their own sections). Change parameters
there, rebuild `franka_bringup`, restart the Franka driver.
```bash
cd ~/xarm_ws && source /opt/ros/humble/setup.bash
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:) \
  colcon build --packages-select teach_replay_controllers franka_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

### Files

| File | Content |
|---|---|
| `include/teach_replay_controllers/teach_record_controller.hpp`, `src/teach_record_controller.cpp` | Recording controller |
| `include/teach_replay_controllers/trajectory_replay_controller.hpp`, `src/trajectory_replay_controller.cpp` | Replay controller |
| `teach_replay_controllers.xml` | pluginlib export |

---

<a id="中文"></a>
## 中文

两个给 Franka Panda（单臂，`arm_id: panda`）用的 ros2_control controller 插件。两者都使用 **effort**（力矩）指令接口，以及位置和速度状态接口。libfranka 力矩模式下机械臂本身会做重力补偿，这里输出的力矩是叠加在重力补偿之上的。一般由 `teach_replay` 工具自动启动，不需要手动操作。

### TeachRecordController — 录制

`teach_replay_controllers/TeachRecordController`，由 `record_demo` 启动。

- 基于重力补偿 controller：指令力矩为零时，手臂可以被人直接推动。
- 加上关节阻尼 **τ = −D · dq**，并限制在 ±`tau_max` 以内。推得太快时会产生阻力，避免接近关节速度上限。
- 通过 `realtime_tools::RealtimePublisher` 以 `publish_rate` 发布 `~/joint_states`（即 `/teach_record_controller/joint_states`，`sensor_msgs/JointState`：position = q，velocity = dq，effort = 指令力矩 τ，`header.stamp` = controller 时间）。这就是录进 bag 的手臂数据。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `arm_id` | `panda` | 关节名前缀 |
| `damping` | `[5, 5, 5, 5, 2, 2, 1]` | 阻尼 D，单位 Nm/(rad/s) |
| `tau_max` | `[20, 20, 20, 20, 6, 6, 6]` | 阻尼力矩上限，Nm |
| `publish_rate` | `500` | Hz，不超过 1000 |

### TrajectoryReplayController — 回放和移动到指定姿态

`teach_replay_controllers/TrajectoryReplayController`，由 `replay_demo`、`go_home` 和 `record_demo`（回 home 时）启动。

- **在 `on_configure` 里加载轨迹文件**，不在 1 kHz 循环里做。`on_activate` 是在循环里执行的，所以那里不做任何耗时操作。要换文件，需要卸载、设置 `trajectory_file`、重新加载并配置（`teach_replay` 会自动做）。
- **阶段 0 — 走到起点：** 用五次多项式从激活时的姿态移动到轨迹的第一个点。速度保证每个关节都不超过 `approach_max_velocity`，时长不少于 `min_approach_duration`。
- **阶段 1 — 回放：** 按时间索引轨迹文件，相邻采样点之间线性插值。结果完全确定：指令目标逐点就是处理后的轨迹。
- **阶段 2 — 完成：** 保持在最后一个点。
- **控制律：** τ = K (q_d − q) + D (dq_d − dq)，限制在 ±`tau_max` 以内。
- **安全：** 任何一个关节落后目标超过 `max_tracking_error` 时，时间轴暂停（目标等手臂跟上），直到误差回到这个值的一半以下。控制循环卡顿后也不会让目标突然跳变：每个周期时间最多前进 5 ms。
- **进度：** 以 `progress_rate` 发布 `~/progress`（`std_msgs/Float64MultiArray`）：`[阶段, 数值, 是否暂停, 主轨迹时长]`。阶段 0 时数值是 0~1 的走到起点进度，阶段 1、2 时是轨迹时间（秒）。手部按这个话题同步，所以始终跟着手臂的真实时间轴走，包括暂停。
- 移动到某个固定姿态（比如回 home）就是一个只包含这个姿态的文件，由“走到起点”阶段完成移动。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `arm_id` | `panda` | 关节名前缀 |
| `trajectory_file` | `""` | CSV 路径（由工具设置） |
| `k_gains` | `[600, 600, 600, 600, 250, 150, 50]` | 刚度 K，Nm/rad |
| `d_gains` | `[50, 50, 50, 50, 30, 25, 15]` | 阻尼 D，Nm·s/rad |
| `tau_max` | `[60, 60, 60, 60, 10, 10, 10]` | 力矩上限，Nm |
| `approach_max_velocity` | `0.2` | rad/s，走到起点的速度上限 |
| `min_approach_duration` | `3.0` | 秒 |
| `max_tracking_error` | `0.15` | rad |
| `progress_rate` | `100` | Hz |

### 轨迹文件格式

CSV 文件，第一行是表头，之后是等间隔的数据行（由 `process_demo` 生成，1 kHz）：
```
t,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7
0.000000000,...
0.001000000,...
```
要求：15 列、数值有限、至少 2 行、间隔恒定（误差 ±1 µs）。不满足的话配置会失败，终端 1 会打印原因。

### 配置与编译

两个 controller 都注册在 `src/robots/franka_arm/franka_bringup/config/controllers.yaml` 里（`controller_manager` 下写类型，参数在各自的段落里）。在那里改参数，重新编译 `franka_bringup`，再重启 Franka 驱动。
```bash
cd ~/xarm_ws && source /opt/ros/humble/setup.bash
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:) \
  colcon build --packages-select teach_replay_controllers franka_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

### 文件

| 文件 | 内容 |
|---|---|
| `include/teach_replay_controllers/teach_record_controller.hpp`、`src/teach_record_controller.cpp` | 录制 controller |
| `include/teach_replay_controllers/trajectory_replay_controller.hpp`、`src/trajectory_replay_controller.cpp` | 回放 controller |
| `teach_replay_controllers.xml` | pluginlib 插件注册 |
