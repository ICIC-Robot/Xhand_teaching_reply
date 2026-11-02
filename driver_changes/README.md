# Driver and configuration changes for teach & replay / 为示教回放所做的驱动和配置修改

[English](#english) | [中文](#中文) — part of the [teach & replay workflow](../README.md)

---

<a id="english"></a>
## English

Changes outside the two new packages that the workflow depends on. This folder has the complete
modified files (same relative paths as in the workspace) and, in `patches/`, the same changes as
unified diffs against the original files — apply them from `~/xarm_ws` with
`for p in patches/*.patch; do patch -p1 < "$p"; done`. On the original machine the originals are also
kept in `~/xarm_ws_old_backup/`.

### 1. franka_hardware — `src/robot.cpp` (bug fixes)

| Problem seen | Cause | Fix |
|---|---|---|
| The driver hangs right after `'activate' hardware 'FrankaHardwareInterface'`; no controller_manager, no topics, Ctrl+C does not stop it | When the driver starts while the arm is still in a reflex from the last session, `setDefaultParams()` is rejected and the error flag is set. During activation, `Robot::read()` then held `read_mutex_` while calling `stopRobot()`, which waits for the reading thread — and that thread needs `read_mutex_`. Deadlock. | **(a)** In the constructor, if `setDefaultParams()` throws a `CommandException`, run `automaticErrorRecovery()` and retry once (this only resets the error state, it does not move the arm). **(b)** `read()` releases `read_mutex_` before calling `stopRobot()`. |
| The driver crashes (exit -6, `std::system_error: Invalid argument` in `stopRobot` / `thread::join`) when the error recovery service is called | `stopRobot()` joined a thread that was not joinable, and could run from two threads at once (controller_manager and the recovery service) | `stopRobot()` takes the previously unused `robot_mutex_` and only joins a joinable thread |

Original: `~/xarm_ws_old_backup/franka_hardware_src/robot.cpp.before_deadlock_fix`.
(`robot.cpp.trylock_version` there is an earlier experiment with non-blocking control callbacks; it is
**not** in use.)

To go back to the original: copy it over `src/robot.cpp` and rebuild `franka_hardware` (see below).

### 2. franka_bringup — `config/controllers.yaml`

Added the two controller types under `controller_manager` and their parameter sections
`teach_record_controller` and `trajectory_replay_controller` (see the
[controllers README](../teach_replay_controllers/README.md) for every parameter).
Nothing existing was changed. Original: `~/xarm_ws_old_backup/controllers.yaml.before_teach_replay`.

### 3. xhand_control_ros2 — `config/xhand_config.yaml` (bug fix)

The top-level key was `xhand_control`, but the node runs as `/xhand_control/xhand_control_ros2`, so
`update_rate: 100.0` never applied and the hand state was published at the built-in default of
**10 Hz**. The key is now `/**`, so the XHand is recorded at 100 Hz. The installed config is a symlink
to this file, so no rebuild is needed; restart the XHand driver. Original:
`~/xarm_ws_old_backup/xhand_config.yaml.orig`.

### 4. System settings (not code)

| Setting | Why | Command |
|---|---|---|
| PC connected directly to the Control box LAN port, no switch | Through the switch, 22% of the 1 kHz commands arrived late (`control_command_success_rate: 0.78`) | — |
| CPU idle states C2/C3 off, every boot | Waking from C3 takes ~1 ms | `for f in /sys/devices/system/cpu/cpu*/cpuidle/state[23]/disable; do echo 1 \| sudo tee $f > /dev/null; done` |
| Optional: NIC interrupt throttling off, every boot | e1000e `rx-usecs 3` can delay packets by up to ~250 µs | `sudo ethtool -C enp0s31f6 rx-usecs 0` |
| `toppra` for the system Python | Offline time-parameterization | `/usr/bin/python3 -m pip install --user toppra` |

### Rebuild after changing any of these
```bash
cd ~/xarm_ws && source /opt/ros/humble/setup.bash
export Franka_DIR="$HOME/.local/lib/cmake/Franka"; export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:) \
  colcon build --packages-select franka_hardware franka_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```
Then restart the Franka driver (Terminal 1).

---

<a id="中文"></a>
## 中文

这套流程依赖的、两个新包以外的修改。这个文件夹里有修改后的完整文件（相对路径和工作区里一致），`patches/` 里是同样的修改，以针对原版文件的 diff 形式保存，可以在 `~/xarm_ws` 下用 `for p in patches/*.patch; do patch -p1 < "$p"; done` 应用。原来那台电脑上，原版文件还保存在 `~/xarm_ws_old_backup/`。

### 1. franka_hardware — `src/robot.cpp`（修复 bug）

| 遇到的问题 | 原因 | 修复 |
|---|---|---|
| 驱动启动到 `'activate' hardware 'FrankaHardwareInterface'` 就卡住；没有 controller_manager，也没有话题，Ctrl+C 停不掉 | 如果启动驱动时机械臂还处于上次留下的 reflex 状态，`setDefaultParams()` 会被拒绝，错误标志被置位。之后在激活阶段，`Robot::read()` 拿着 `read_mutex_` 去调用 `stopRobot()`，而 `stopRobot()` 要等读取线程结束，读取线程又需要 `read_mutex_`，于是死锁。 | **(a)** 构造函数里如果 `setDefaultParams()` 抛出 `CommandException`，先执行 `automaticErrorRecovery()` 再重试一次（只清除错误状态，不会让机械臂运动）。**(b)** `read()` 先释放 `read_mutex_`，再调用 `stopRobot()`。 |
| 调用错误恢复服务时驱动崩溃（退出码 -6，`stopRobot` / `thread::join` 里报 `std::system_error: Invalid argument`） | `stopRobot()` 对一个已经不能 join 的线程调用了 join，而且可能被两个线程同时调用（controller_manager 和恢复服务） | `stopRobot()` 加上原来没被使用的 `robot_mutex_`，并且只 join 还能 join 的线程 |

原版文件：`~/xarm_ws_old_backup/franka_hardware_src/robot.cpp.before_deadlock_fix`。
（同一目录下的 `robot.cpp.trylock_version` 是之前尝试的“控制回调不阻塞”版本，**没有在使用**。）

要恢复原版：把原版文件复制回 `src/robot.cpp`，重新编译 `franka_hardware`（见下文）。

### 2. franka_bringup — `config/controllers.yaml`

在 `controller_manager` 下新增了两个 controller 类型，以及它们各自的参数段 `teach_record_controller` 和 `trajectory_replay_controller`（每个参数的说明见 [controller README](../teach_replay_controllers/README.md)）。原有内容没有改动。原版文件：`~/xarm_ws_old_backup/controllers.yaml.before_teach_replay`。

### 3. xhand_control_ros2 — `config/xhand_config.yaml`（修复 bug）

原来的顶层键是 `xhand_control`，但节点实际的名字是 `/xhand_control/xhand_control_ros2`，所以 `update_rate: 100.0` 从来没有生效过，手的状态一直以代码里的默认值 **10 Hz** 发布。现在键改成了 `/**`，XHand 以 100 Hz 录制。安装目录里的配置文件是指向这个文件的符号链接，所以不用重新编译，重启 XHand 驱动即可。原版文件：`~/xarm_ws_old_backup/xhand_config.yaml.orig`。

### 4. 系统设置（不是代码）

| 设置 | 原因 | 命令 |
|---|---|---|
| 电脑直连控制柜 LAN 口，不经过交换机 | 经过交换机时，22% 的 1 kHz 指令没能按时送达（`control_command_success_rate: 0.78`） | — |
| 关闭 CPU C2/C3 省电状态，每次开机后执行 | 从 C3 唤醒需要约 1 ms | `for f in /sys/devices/system/cpu/cpu*/cpuidle/state[23]/disable; do echo 1 \| sudo tee $f > /dev/null; done` |
| 可选：关闭网卡中断节流，每次开机后执行 | e1000e 的 `rx-usecs 3` 可能让数据包延迟最多约 250 µs | `sudo ethtool -C enp0s31f6 rx-usecs 0` |
| 给系统 Python 安装 `toppra` | 离线重新分配时间 | `/usr/bin/python3 -m pip install --user toppra` |

### 修改后重新编译
```bash
cd ~/xarm_ws && source /opt/ros/humble/setup.bash
export Franka_DIR="$HOME/.local/lib/cmake/Franka"; export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:) \
  colcon build --packages-select franka_hardware franka_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```
然后重启 Franka 驱动（终端 1）。
