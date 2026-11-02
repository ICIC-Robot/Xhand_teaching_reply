# Xhand_teaching_reply — Franka + XHand Teach & Replay / 示教与回放

[English](#english) | [中文](#中文)

---

<a id="english"></a>
## English

Record a demonstration by guiding the Franka Panda arm and the XHand by hand, process it offline
(smoothing + speed limiting), and replay it automatically. Every recording and every replay
starts from and returns to the factory home pose.

```
record_demo ──► bag ──► process_demo ──► arm/hand trajectory files ──► replay_demo
  (home → teach → home)    (B-spline + toppra + checks)                  (approach → replay → home)
```

### Modules

| Module | Path | README |
|---|---|---|
| Controllers (C++, run inside the 1 kHz loop) | [`teach_replay_controllers/`](teach_replay_controllers) | [README](teach_replay_controllers/README.md) |
| Tools (Python: record / process / replay / go home) | [`teach_replay/`](teach_replay) | [README](teach_replay/README.md) |
| Driver and config changes this workflow needs | [`driver_changes/`](driver_changes) | [README](driver_changes/README.md) |

### 0. Install this repository into the workspace

The packages are built inside the existing `~/xarm_ws` workspace (franka_ros2 + XHand driver).
Clone this repository **outside** `~/xarm_ws/src` (a second copy of the same packages there would
break `colcon build`), then copy the two packages in and apply the driver patches:
```bash
cd ~
git clone https://github.com/ICIC-Robot/Xhand_teaching_reply.git
cp -r ~/Xhand_teaching_reply/teach_replay_controllers ~/Xhand_teaching_reply/teach_replay ~/xarm_ws/src/integration/
cd ~/xarm_ws
for p in ~/Xhand_teaching_reply/driver_changes/patches/*.patch; do patch -p1 < "$p"; done
```
The patches apply to the original franka_ros2 / XHand files (skip this step on a machine that already
has them). Then follow section 2 to build. Paths such as `src/robots/...` below are relative to
`~/xarm_ws`.

### 1. Hardware connection

```
PC  enp0s31f6 (172.16.0.5/24) ──── Ethernet cable, DIRECT, no switch ──── Franka Control box, LAN port (172.16.0.2)
Franka Control box ──── dedicated Arm cable (thick round cable, not Ethernet) ──── Franka Arm
PC  USB-to-Ethernet adapter enx00e04c0b3340 ──── EtherCAT ──── XHand (mounted on the arm)
```

- **Connect the PC directly to the LAN port of the Control box.** Do not put a switch between them.
  FCI needs a command from the PC every 1 ms. Through the switch, about 22% of the commands arrived
  late (`control_command_success_rate: 0.78`), and every motion was stopped by a
  `communication_constraints_violation` reflex.
- The PC address on `enp0s31f6` is static `172.16.0.5/24` (NetworkManager profile
  "Wired connection 1"); the robot is `172.16.0.2`.
- Desk (the robot's web UI) runs over the same cable: open `https://172.16.0.2` in a browser.
- The XHand uses its own USB adapter (`enx00e04c0b3340`). Do not use that port for the arm.
- To use a second arm later, add another Ethernet port to the PC and connect that arm's Control box
  directly to it as well.

**In Desk, once:** set the end-effector load to **XHand, 1.1 kg**.
**In Desk, every session:** unlock the joints and activate FCI. The light must be **solid blue**.

| Light | Meaning | Action |
|---|---|---|
| Solid blue | Ready, FCI active | OK |
| White / blinking white | Locked or booting | Unlock the joints and activate FCI in Desk |
| Yellow | User stop pressed | Release the user stop button |
| Red | Error | Clear it in Desk |

Check FCI from the PC: `nc -zv -w 3 172.16.0.2 1337` must say `succeeded`.

### 2. Environment (one-time)

This machine already has all of this. These are the steps for a new machine.

- Ubuntu 22.04, PREEMPT_RT kernel (`uname -r` → `5.15.0-1100-realtime`), ROS 2 Humble.
- User in the `realtime` group with `/etc/security/limits.conf` containing
  `@realtime - rtprio 99` and `@realtime - memlock unlimited`.
- libfranka 0.9.2 installed to `~/.local`, and in `~/.bashrc`:
  ```bash
  export Franka_DIR="$HOME/.local/lib/cmake/Franka"
  export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH}"
  source /opt/ros/humble/setup.bash
  [ -f "$HOME/xarm_ws/install/setup.bash" ] && source "$HOME/xarm_ws/install/setup.bash"
  ```
- Python tools for offline processing (system Python 3.10, not conda):
  ```bash
  /usr/bin/python3 -m pip install --user toppra
  ```
  numpy, scipy, matplotlib and yaml come from Ubuntu. Optional, for mcap bags:
  `sudo apt install ros-humble-rosbag2-storage-mcap` (otherwise bags are sqlite3, which works the same).
- XHand without sudo (already done here):
  `sudo setcap cap_net_raw+ep ~/xarm_ws/build/xhand_control_ros2/xhand_control_ros2_node`,
  plus `/etc/ld.so.conf.d/xhand-ros.conf` listing `/opt/ros/humble/lib` and the two xhand install
  lib directories, then `sudo ldconfig`.

**Build.** CMake must not pick up the conda Python, so take conda off `PATH` for the build:
```bash
cd ~/xarm_ws
source /opt/ros/humble/setup.bash
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:) \
  colcon build --packages-select franka_hardware franka_bringup teach_replay_controllers teach_replay \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```
Restart the Franka driver after rebuilding `franka_hardware`, `franka_bringup` or the controllers.

### 3. Every time the PC boots

Turn off the deep CPU idle states (waking from C3 takes about 1 ms, the whole control period):
```bash
for f in /sys/devices/system/cpu/cpu*/cpuidle/state[23]/disable; do echo 1 | sudo tee $f > /dev/null; done
cat /sys/devices/system/cpu/cpu0/cpuidle/state*/disable   # must print 0 0 1 1
```
Optional, if `control_command_success_rate` is still below ~0.95 (turns off NIC interrupt throttling):
```bash
sudo ethtool -C enp0s31f6 rx-usecs 0
```
Keep the laptop on AC power.

### 4. Run the complete workflow

Use three terminals. Terminals 1 and 2 stay open the whole time.

**Terminal 1 — Franka driver**
```bash
source ~/xarm_ws/install/setup.bash
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2 use_rviz:=false load_gripper:=false
```
Wait for `Configured and activated joint_state_broadcaster`. If the arm is still in a reflex from
the last session, the driver now clears it by itself (`Automatic error recovery at startup succeeded`).
A flood of `Robot state publisher ignored an invalid JointState message` is normal.

**Terminal 2 — XHand driver**
```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xhand_control_ros2 xhand_control.launch.py
```
Wait for `Operational state reached for all slaves.`

**Terminal 3 — the workflow**
```bash
source ~/xarm_ws/install/setup.bash

# (any time) move the arm + hand to the factory home pose
ros2 run teach_replay go_home

# 1. record: goes home -> you teach -> ENTER stops -> 3 s countdown -> returns home
ros2 run teach_replay record_demo grasp_c

# 2. process offline (the robot does not move)
ros2 run teach_replay process_demo ~/xarm_ws/demos/grasp_c

# 3. replay: dry run first (nothing moves), then for real (waits for ENTER)
ros2 run teach_replay replay_demo ~/xarm_ws/demos/grasp_c --dry-run
ros2 run teach_replay replay_demo ~/xarm_ws/demos/grasp_c
```

During recording: when `=== RECORDING ===` appears, push the arm directly by hand (no wrist buttons
needed) and pose the fingers; press ENTER to stop, then let go and step back.
Before every automatic move: light solid blue, nobody within reach, hand near the e-stop.
Ctrl+C stops any motion. A new recording needs a new name; existing demos are never overwritten.

The factory home pose: arm `[0, -π/4, 0, -3π/4, 0, π/2, π/4]` (Franka ready pose), XHand all joints
`0 rad` (factory zero, hand open). Returning home opens the hand, which releases a held object —
use `replay_demo ... --no-return-home` to stay at the final pose.

### 5. Where files are saved

| What | Location |
|---|---|
| One demo (everything about it) | `~/xarm_ws/demos/<name>/` |
| Raw recording (ros2 bag) | `~/xarm_ws/demos/<name>/bag/` |
| Processed arm trajectory, 1 kHz (`t, q1..q7, dq1..dq7`) | `~/xarm_ws/demos/<name>/arm_trajectory.csv` |
| Processed hand trajectory, 200 Hz (`t`, 12 joints) | `~/xarm_ws/demos/<name>/hand_trajectory.csv` |
| Processing report (durations, % of limits, checks) | `~/xarm_ws/demos/<name>/summary.yaml` |
| Recorded vs smoothed arm path plot | `~/xarm_ws/demos/<name>/preview.png` |
| Temporary home-pose files (overwritten each time) | `~/xarm_ws/demos/<name>/go_home_trajectory.csv`, `~/xarm_ws/demos/.go_home_trajectory.csv` |
| Controller parameters (gains, damping, limits) | `src/robots/franka_arm/franka_bringup/config/controllers.yaml` |
| XHand driver rate (100 Hz) | `src/robots/xhand/xhand_control_ros2/config/xhand_config.yaml` |
| ROS logs (driver output, incl. `franka_control2_node_<pid>_*.log`) | `~/.ros/log/` |
| The old waypoint_teleop approach and original files (moved, not deleted) | `~/xarm_ws_old_backup/` |

### 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `robot_mode REFLEX` / yellow light | Release the user stop. Clear the reflex: `ros2 service call /error_recovery_service_server/error_recovery franka_msgs/srv/ErrorRecovery {}` |
| Terminal 1: `communication_constraints_violation`, `control_command_success_rate: 0.7x` | The PC is late with the 1 kHz commands: connect directly to the Control box (no switch), disable the C-states (section 3), try `ethtool -C enp0s31f6 rx-usecs 0`, close heavy programs. |
| `No /franka_robot_state_broadcaster/robot_state` | Terminal 1 is not running or has not finished starting. Look at its last lines. |
| Terminal 1 stuck after `'activate' hardware`, Ctrl+C does not stop it | Fixed in franka_hardware (see the changes README). If it still happens: `pkill -9 franka_control2` and relaunch. |
| `No XHand devices found` | Check the XHand cable / USB adapter; only one XHand driver may run. |
| process_demo says `NOT REPLAYABLE` | A joint came too close to its hardware range during teaching. Re-record with the arm less folded. |
| Replay is much slower than the demonstration | Expected: speed is capped at 40% of the Franka limits. Use `process_demo --vel-scale 0.5 --acc-scale 0.5` for faster (max 1.0). |
| A pause in the demo disappeared | toppra removes stretches where both the arm and the hand are completely still. |

---

<a id="中文"></a>
## 中文

用手拖动 Franka Panda 机械臂和 XHand 录制一段示范，离线做平滑和限速处理，然后自动回放。每次录制和回放都从出厂 home 姿态开始，结束后回到 home。

```
record_demo ──► bag ──► process_demo ──► 手臂/手部轨迹文件 ──► replay_demo
（home → 示教 → home）   （B 样条平滑 + toppra 限速 + 检查）       （走到起点 → 回放 → home）
```

### 模块

| 模块 | 路径 | 说明文档 |
|---|---|---|
| 控制器（C++，在 1 kHz 控制循环里运行） | [`teach_replay_controllers/`](teach_replay_controllers) | [README](teach_replay_controllers/README.md) |
| 工具（Python：录制 / 处理 / 回放 / 回 home） | [`teach_replay/`](teach_replay) | [README](teach_replay/README.md) |
| 这套流程需要的驱动和配置修改 | [`driver_changes/`](driver_changes) | [README](driver_changes/README.md) |

### 0. 把这个仓库安装到工作区

这两个包要放在现有的 `~/xarm_ws` 工作区（franka_ros2 + XHand 驱动）里编译。把仓库 clone 到 `~/xarm_ws/src` **之外**的地方（同样的包在工作区里出现两份会导致 `colcon build` 报错），然后把两个包复制进去，再应用驱动的 patch：
```bash
cd ~
git clone https://github.com/ICIC-Robot/Xhand_teaching_reply.git
cp -r ~/Xhand_teaching_reply/teach_replay_controllers ~/Xhand_teaching_reply/teach_replay ~/xarm_ws/src/integration/
cd ~/xarm_ws
for p in ~/Xhand_teaching_reply/driver_changes/patches/*.patch; do patch -p1 < "$p"; done
```
patch 是针对原版 franka_ros2 / XHand 文件的（已经应用过的电脑跳过这一步）。然后按第 2 节编译。下文中 `src/robots/...` 这类路径都是相对于 `~/xarm_ws` 的。

### 1. 硬件连接

```
电脑 enp0s31f6（172.16.0.5/24）──── 网线，直连，不经过交换机 ──── Franka 控制柜 LAN 口（172.16.0.2）
Franka 控制柜 ──── 专用连接电缆（粗圆形电缆，不是网线）──── Franka 机械臂
电脑 USB 转网口 enx00e04c0b3340 ──── EtherCAT ──── XHand（装在机械臂末端）
```

- **电脑必须用网线直连控制柜的 LAN 口，中间不要接交换机。** FCI 要求电脑每 1 ms 发一次指令。之前经过交换机时，大约 22% 的指令没有按时送达（`control_command_success_rate: 0.78`），每次一运动就会因为 `communication_constraints_violation` 触发 reflex 停下。
- 电脑 `enp0s31f6` 网口的 IP 固定为 `172.16.0.5/24`（NetworkManager 里的 "Wired connection 1"），机械臂是 `172.16.0.2`。
- Desk（机械臂的网页界面）走同一根网线：浏览器打开 `https://172.16.0.2`。
- XHand 用单独的 USB 转网口（`enx00e04c0b3340`），不要拿这个口接机械臂。
- 以后要用第二台机械臂，需要给电脑再加一个网口，第二台的控制柜也直连到这个新网口。

**在 Desk 里设置一次：** 末端负载设为 **XHand，1.1 kg**。
**每次使用前在 Desk 里：** 解锁关节并激活 FCI，灯必须是**蓝色常亮**。

| 灯 | 含义 | 处理 |
|---|---|---|
| 蓝色常亮 | 就绪，FCI 已激活 | 可以开始 |
| 白色 / 白色闪烁 | 关节锁定或正在启动 | 在 Desk 里解锁关节并激活 FCI |
| 黄色 | 按下了用户急停 | 松开急停按钮 |
| 红色 | 出错 | 在 Desk 里清除错误 |

在电脑上确认 FCI：`nc -zv -w 3 172.16.0.2 1337` 必须显示 `succeeded`。

### 2. 环境配置（只需一次）

这台电脑已经全部配置好了。下面是新电脑上需要做的步骤。

- Ubuntu 22.04、PREEMPT_RT 实时内核（`uname -r` 显示 `5.15.0-1100-realtime`）、ROS 2 Humble。
- 用户加入 `realtime` 组，并在 `/etc/security/limits.conf` 里写入 `@realtime - rtprio 99` 和 `@realtime - memlock unlimited`。
- libfranka 0.9.2 安装在 `~/.local`，并在 `~/.bashrc` 里加入：
  ```bash
  export Franka_DIR="$HOME/.local/lib/cmake/Franka"
  export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH}"
  source /opt/ros/humble/setup.bash
  [ -f "$HOME/xarm_ws/install/setup.bash" ] && source "$HOME/xarm_ws/install/setup.bash"
  ```
- 离线处理用的 Python 库（装在系统 Python 3.10 里，不是 conda）：
  ```bash
  /usr/bin/python3 -m pip install --user toppra
  ```
  numpy、scipy、matplotlib、yaml 用 Ubuntu 自带的即可。如果想用 mcap 格式的 bag（可选）：`sudo apt install ros-humble-rosbag2-storage-mcap`。不装的话会用 sqlite3 格式，功能完全一样。
- XHand 不用 sudo 运行（这台电脑已经配置好）：`sudo setcap cap_net_raw+ep ~/xarm_ws/build/xhand_control_ros2/xhand_control_ros2_node`，再加上 `/etc/ld.so.conf.d/xhand-ros.conf`（写入 `/opt/ros/humble/lib` 和两个 xhand 安装库目录），最后执行 `sudo ldconfig`。

**编译。** 不能让 CMake 用到 conda 的 Python，所以编译时要把 conda 从 `PATH` 里去掉：
```bash
cd ~/xarm_ws
source /opt/ros/humble/setup.bash
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda3 | paste -sd:) \
  colcon build --packages-select franka_hardware franka_bringup teach_replay_controllers teach_replay \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```
重新编译 `franka_hardware`、`franka_bringup` 或 controller 之后，要重启 Franka 驱动才会生效。

### 3. 每次开机后

关闭 CPU 深度省电状态（从 C3 唤醒大约要 1 ms，相当于整整一个控制周期）：
```bash
for f in /sys/devices/system/cpu/cpu*/cpuidle/state[23]/disable; do echo 1 | sudo tee $f > /dev/null; done
cat /sys/devices/system/cpu/cpu0/cpuidle/state*/disable   # 必须显示 0 0 1 1
```
可选：如果 `control_command_success_rate` 仍然低于 0.95 左右，关闭网卡中断节流：
```bash
sudo ethtool -C enp0s31f6 rx-usecs 0
```
笔记本保持插着电源。

### 4. 运行完整流程

需要三个终端，终端 1 和终端 2 全程保持打开。

**终端 1 — Franka 驱动**
```bash
source ~/xarm_ws/install/setup.bash
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2 use_rviz:=false load_gripper:=false
```
等到出现 `Configured and activated joint_state_broadcaster`。如果机械臂还处于上次留下的 reflex 状态，驱动现在会自动清除（显示 `Automatic error recovery at startup succeeded`）。刷屏的 `Robot state publisher ignored an invalid JointState message` 是正常的，可以忽略。

**终端 2 — XHand 驱动**
```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xhand_control_ros2 xhand_control.launch.py
```
等到出现 `Operational state reached for all slaves.`

**终端 3 — 运行流程**
```bash
source ~/xarm_ws/install/setup.bash

# （随时可用）把机械臂和手移回出厂 home 姿态
ros2 run teach_replay go_home

# 1. 录制：先回 home → 你示教 → 按回车结束 → 倒数 3 秒 → 自动回 home
ros2 run teach_replay record_demo grasp_c

# 2. 离线处理（机械臂不会动）
ros2 run teach_replay process_demo ~/xarm_ws/demos/grasp_c

# 3. 回放：先干跑（不会动），再真正执行（会等你按回车）
ros2 run teach_replay replay_demo ~/xarm_ws/demos/grasp_c --dry-run
ros2 run teach_replay replay_demo ~/xarm_ws/demos/grasp_c
```

录制时：出现 `=== RECORDING ===` 后，直接用手推动机械臂（不需要按手腕上的引导按钮），同时摆弄手指；按回车结束，然后松手离开机械臂。
每次自动运动之前：灯是蓝色、周围没有人、手放在急停旁边。
Ctrl+C 可以随时停止运动。每次录制要用新的名字，已有的录制不会被覆盖。

出厂 home 姿态：手臂 `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`（Franka 标准 ready 姿态），XHand 所有关节 `0 rad`（出厂标定零位，手张开）。回 home 时手会张开，手里拿着的东西会掉下来；如果想停在最后的姿态，用 `replay_demo ... --no-return-home`。

### 5. 文件保存位置

| 内容 | 位置 |
|---|---|
| 一次录制的全部文件 | `~/xarm_ws/demos/<名字>/` |
| 原始录制（ros2 bag） | `~/xarm_ws/demos/<名字>/bag/` |
| 处理后的手臂轨迹，1 kHz（`t, q1..q7, dq1..dq7`） | `~/xarm_ws/demos/<名字>/arm_trajectory.csv` |
| 处理后的手部轨迹，200 Hz（`t` 和 12 个关节） | `~/xarm_ws/demos/<名字>/hand_trajectory.csv` |
| 处理报告（时长、占限值的百分比、检查结果） | `~/xarm_ws/demos/<名字>/summary.yaml` |
| 原始路径与平滑后路径的对比图 | `~/xarm_ws/demos/<名字>/preview.png` |
| 回 home 用的临时文件（每次覆盖） | `~/xarm_ws/demos/<名字>/go_home_trajectory.csv`、`~/xarm_ws/demos/.go_home_trajectory.csv` |
| controller 参数（增益、阻尼、限值） | `src/robots/franka_arm/franka_bringup/config/controllers.yaml` |
| XHand 驱动频率（100 Hz） | `src/robots/xhand/xhand_control_ros2/config/xhand_config.yaml` |
| ROS 日志（驱动输出，包括 `franka_control2_node_<pid>_*.log`） | `~/.ros/log/` |
| 旧的 waypoint_teleop 方案和原版文件（移走了，没有删除） | `~/xarm_ws_old_backup/` |

### 6. 常见问题

| 现象 | 原因 / 解决办法 |
|---|---|
| `robot_mode REFLEX` / 黄灯 | 松开急停，然后清除 reflex：`ros2 service call /error_recovery_service_server/error_recovery franka_msgs/srv/ErrorRecovery {}` |
| 终端 1 显示 `communication_constraints_violation`、`control_command_success_rate: 0.7x` | 电脑发 1 kHz 指令不够准时：改成直连控制柜（不经过交换机）、关闭 C 状态（第 3 节）、试试 `ethtool -C enp0s31f6 rx-usecs 0`、关掉占资源的程序。 |
| `No /franka_robot_state_broadcaster/robot_state` | 终端 1 没有运行，或者还没启动完成。看一下终端 1 最后几行。 |
| 终端 1 卡在 `'activate' hardware`，Ctrl+C 也停不掉 | franka_hardware 里已经修复（见驱动修改说明）。如果还出现：`pkill -9 franka_control2`，然后重新启动。 |
| `No XHand devices found` | 检查 XHand 线缆和 USB 转网口；同一时间只能运行一个 XHand 驱动。 |
| process_demo 显示 `NOT REPLAYABLE` | 示教时某个关节太接近它的硬件极限。重新录制，手臂不要折得太紧。 |
| 回放比示范慢很多 | 正常：速度上限是 Franka 限值的 40%。想快一些可以用 `process_demo --vel-scale 0.5 --acc-scale 0.5`（最大 1.0）。 |
| 示范里的停顿不见了 | toppra 会去掉手臂和手都完全静止的时间段。 |
