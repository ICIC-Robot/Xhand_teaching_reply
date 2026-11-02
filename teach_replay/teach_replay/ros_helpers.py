"""Shared ROS plumbing for record_demo / replay_demo.

Everything here is a thin, blocking wrapper around services and topics that
already exist on the robot PC:
  - controller_manager services (list / load / configure / switch / unload)
  - franka_hardware's param_service_server (collision thresholds)
  - franka_robot_state_broadcaster (robot_mode, to refuse to start when the
    arm cannot move)
  - xhand_control_ros2 topics (hand state, hand command)
"""

import time
from typing import Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
    UnloadController,
)
from franka_msgs.msg import FrankaState
from franka_msgs.srv import SetForceTorqueCollisionBehavior
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from xhand_control_interfaces.msg import XHandCommand, XHandStateArray

CONTROLLER_MANAGER = '/controller_manager'
ROBOT_STATE_TOPIC = '/franka_robot_state_broadcaster/robot_state'
XHAND_STATE_TOPIC = '/xhand_control/xhand_state'
XHAND_COMMAND_TOPIC = '/xhand_control/xhand_command'
COLLISION_SERVICE_SUFFIX = '/set_force_torque_collision_behavior'

TEACH_CONTROLLER = 'teach_record_controller'
REPLAY_CONTROLLER = 'trajectory_replay_controller'
# Controllers that claim arm command interfaces; at most one may be active.
ARM_COMMAND_CONTROLLERS = [
    TEACH_CONTROLLER, REPLAY_CONTROLLER, 'joint_trajectory_controller',
    'gravity_compensation_example_controller', 'joint_impedance_example_controller',
    'move_to_start_example_controller', 'joint_position_example_controller',
    'joint_velocity_example_controller', 'cartesian_velocity_example_controller',
    'cartesian_keyboard_controller',
]

ROBOT_MODE_NAMES = {
    FrankaState.ROBOT_MODE_OTHER: 'OTHER',
    FrankaState.ROBOT_MODE_IDLE: 'IDLE',
    FrankaState.ROBOT_MODE_MOVE: 'MOVE',
    FrankaState.ROBOT_MODE_GUIDING: 'GUIDING',
    FrankaState.ROBOT_MODE_REFLEX: 'REFLEX',
    FrankaState.ROBOT_MODE_USER_STOPPED: 'USER_STOPPED',
    FrankaState.ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY: 'AUTOMATIC_ERROR_RECOVERY',
}

# Collision thresholds (lower == upper, i.e. one contact/collision level).
# Default: what franka_hardware's Robot::setDefaultParams() sets at startup.
# Teach: high enough that a human pushing the arm is not judged a collision.
DEFAULT_JOINT_TORQUE_THRESHOLDS = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
DEFAULT_FORCE_THRESHOLDS = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
TEACH_JOINT_TORQUE_THRESHOLDS = [60.0, 60.0, 50.0, 50.0, 30.0, 25.0, 20.0]
TEACH_FORCE_THRESHOLDS = [80.0, 80.0, 80.0, 40.0, 40.0, 40.0]

# Factory-defined home poses. Franka: the standard ready pose used by libfranka / franka_ros
# (move_to_start). XHand: its factory-calibrated zero, every joint at 0 rad (the hand open and
# flat) -- the only initial pose the vendor SDK defines in angle units.
FRANKA_HOME = [0.0, -0.785398163397, 0.0, -2.356194490192, 0.0, 1.570796326795, 0.785398163397]
XHAND_HOME_POSITION = 0.0

XHAND_PASSIVE_MODE = 0
XHAND_POSITION_MODE = 3
XHAND_DEFAULT_KP = 100.0
XHAND_DEFAULT_EFFORT_LIMIT = 350.0

RECOVERY_HINT = ('Make sure the robot light is solid blue (release the user stop). To clear a '
                 'reflex: ros2 service call /error_recovery_service_server/error_recovery '
                 'franka_msgs/srv/ErrorRecovery {}')


class RobotLink(Node):
    """One node holding every client/subscription the scripts need."""

    def __init__(self, name: str):
        super().__init__(name)
        self.robot_state: Optional[FrankaState] = None
        self.xhand_state: Optional[XHandStateArray] = None
        self.create_subscription(FrankaState, ROBOT_STATE_TOPIC, self._on_robot_state, 10)
        self.create_subscription(XHandStateArray, XHAND_STATE_TOPIC, self._on_xhand_state, 10)
        self.xhand_pub = self.create_publisher(XHandCommand, XHAND_COMMAND_TOPIC, 10)

    def _on_robot_state(self, msg: FrankaState) -> None:
        self.robot_state = msg

    def _on_xhand_state(self, msg: XHandStateArray) -> None:
        self.xhand_state = msg

    # ---------------------------------------------------------------- waiting
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def wait_for_states(self, timeout_sec: float = 10.0, need_hand: bool = True) -> None:
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end and (self.robot_state is None or
                                          (need_hand and self.xhand_state is None)):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.robot_state is None:
            raise RuntimeError(f'No {ROBOT_STATE_TOPIC}. Is franka_bringup franka.launch.py '
                               'running?')
        if need_hand and self.xhand_state is None:
            raise RuntimeError(f'No {XHAND_STATE_TOPIC}. Is xhand_control.launch.py running?')

    # ----------------------------------------------------------------- franka
    def robot_mode_name(self) -> str:
        if self.robot_state is None:
            return 'UNKNOWN'
        return ROBOT_MODE_NAMES.get(self.robot_state.robot_mode,
                                    f'UNKNOWN({self.robot_state.robot_mode})')

    def require_arm_ready(self) -> None:
        self.spin_for(0.2)
        mode = self.robot_mode_name()
        if mode not in ('IDLE', 'MOVE'):
            raise RuntimeError(f'Franka robot_mode is {mode}, not IDLE. {RECOVERY_HINT}')

    def arm_positions(self) -> List[float]:
        return list(self.robot_state.q)

    def set_collision_thresholds(self, joint_torque: List[float], force: List[float]) -> None:
        names = [n for n, types in self.get_service_names_and_types()
                 if n.endswith(COLLISION_SERVICE_SUFFIX)]
        if not names:
            raise RuntimeError(f'No *{COLLISION_SERVICE_SUFFIX} service found (franka_hardware '
                               'param_service_server).')
        req = SetForceTorqueCollisionBehavior.Request()
        req.lower_torque_thresholds_nominal = list(joint_torque)
        req.upper_torque_thresholds_nominal = list(joint_torque)
        req.lower_force_thresholds_nominal = list(force)
        req.upper_force_thresholds_nominal = list(force)
        res = self._call(SetForceTorqueCollisionBehavior, names[0], req)
        if not res.success:
            raise RuntimeError(f'Setting collision thresholds failed: {res.error}')

    # ------------------------------------------------------------------ xhand
    def hand_ids(self) -> List[int]:
        return list(self.xhand_state.hand_id)

    def hand_joint_names(self, hand_index: int = 0) -> List[str]:
        return list(self.xhand_state.hand_states[hand_index].name)

    def hand_positions(self, hand_index: int = 0) -> List[float]:
        return list(self.xhand_state.hand_states[hand_index].position)

    def send_hand(self, hand_id: int, names: List[str], positions: List[float], mode: int,
                  kp: float = XHAND_DEFAULT_KP) -> None:
        msg = XHandCommand()
        msg.hand_id = int(hand_id)
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        msg.kp = [float(kp)] * len(names)
        msg.ki = [0.0] * len(names)
        msg.kd = [0.0] * len(names)
        msg.effort_limit = [XHAND_DEFAULT_EFFORT_LIMIT] * len(names)
        msg.mode = int(mode)
        self.xhand_pub.publish(msg)

    def hand_passive(self, hand_id: int = 0, hand_index: int = 0) -> None:
        self.send_hand(hand_id, self.hand_joint_names(hand_index), self.hand_positions(hand_index),
                       XHAND_PASSIVE_MODE)

    def hand_hold(self, hand_id: int = 0, hand_index: int = 0) -> None:
        self.spin_for(0.05)
        self.send_hand(hand_id, self.hand_joint_names(hand_index), self.hand_positions(hand_index),
                       XHAND_POSITION_MODE)

    # ------------------------------------------------------- controller manager
    def _call(self, srv_type, name: str, request, timeout_sec: float = 10.0):
        client = self.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f'Service {name} not available')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.result() is None:
                raise RuntimeError(f'Service {name} did not answer within {timeout_sec}s')
            return future.result()
        finally:
            self.destroy_client(client)

    def controller_states(self) -> Dict[str, str]:
        res = self._call(ListControllers, f'{CONTROLLER_MANAGER}/list_controllers',
                         ListControllers.Request())
        return {c.name: c.state for c in res.controller}

    def load_controller(self, name: str) -> None:
        req = LoadController.Request()
        req.name = name
        if not self._call(LoadController, f'{CONTROLLER_MANAGER}/load_controller', req).ok:
            raise RuntimeError(f'Failed to load {name} (is it listed in controllers.yaml?)')

    def configure_controller(self, name: str) -> None:
        req = ConfigureController.Request()
        req.name = name
        if not self._call(ConfigureController, f'{CONTROLLER_MANAGER}/configure_controller',
                          req, timeout_sec=30.0).ok:
            raise RuntimeError(f'Failed to configure {name} -- see Terminal 1 for the reason')

    def unload_controller(self, name: str) -> None:
        req = UnloadController.Request()
        req.name = name
        if not self._call(UnloadController, f'{CONTROLLER_MANAGER}/unload_controller', req).ok:
            raise RuntimeError(f'Failed to unload {name}')

    def switch_controllers(self, activate: List[str], deactivate: List[str]) -> None:
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=5, nanosec=0)
        res = self._call(SwitchController, f'{CONTROLLER_MANAGER}/switch_controller', req)
        if not res.ok:
            # The STRICT switch can report a confirmation timeout although the switch took
            # effect -- trust the actual controller states instead.
            states = self.controller_states()
            if not (all(states.get(n) == 'active' for n in activate) and
                    all(states.get(n) != 'active' for n in deactivate)):
                raise RuntimeError(f'switch_controller failed (activate={activate}, '
                                   f'deactivate={deactivate}); states now {states}')

    def active_arm_controllers(self) -> List[str]:
        states = self.controller_states()
        return [n for n in ARM_COMMAND_CONTROLLERS if states.get(n) == 'active']

    def deactivate_arm_controllers(self) -> None:
        active = self.active_arm_controllers()
        if active:
            self.switch_controllers([], active)

    def ensure_configured(self, name: str) -> None:
        """Load + configure `name` if needed, so that it is 'inactive' (ready to activate)."""
        state = self.controller_states().get(name)
        if state is None:
            self.load_controller(name)
            state = self.controller_states().get(name)
        if state == 'unconfigured':
            self.configure_controller(name)

    def set_controller_parameter_string(self, controller: str, param: str, value: str) -> None:
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param, value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=value))]
        res = self._call(SetParameters, f'/{controller}/set_parameters', req)
        if not res.results or not res.results[0].successful:
            reason = res.results[0].reason if res.results else 'no result'
            raise RuntimeError(f'Setting {controller}.{param} failed: {reason}')
