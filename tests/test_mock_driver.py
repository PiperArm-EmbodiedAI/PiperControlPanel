from enum import IntEnum
from types import SimpleNamespace

import pytest

from src.core.action import Action
from src.core.observation import Observation
from src.core.robot_state import RobotState
from src.drivers.mock_driver import MockDriver
from src.drivers.piper_driver import PiperDriver
from src.gui.view_model import GuiState


class StatusValue(IntEnum):
    ONE = 1
    TWO = 2
    THREE = 3
    FOUR = 4
    FIVE = 5
    SIX = 6


class StatusSdkDouble:
    def __init__(self) -> None:
        self.enable_status = [True] * 6
        err_status = SimpleNamespace(
            **{
                **{
                    f"joint_{index}_angle_limit": index in (2, 4, 6)
                    for index in range(1, 7)
                },
                **{
                    f"communication_status_joint_{index}": index in (1, 3, 5)
                    for index in range(1, 7)
                },
            }
        )
        status = SimpleNamespace(
            ctrl_mode=StatusValue.ONE,
            arm_status=StatusValue.TWO,
            mode_feed=StatusValue.THREE,
            teach_status=StatusValue.FOUR,
            motion_status=StatusValue.FIVE,
            trajectory_num=StatusValue.SIX,
            err_code=0x150A,
            err_status=err_status,
        )
        self.status_feedback = SimpleNamespace(arm_status=status)

    def CreateCanBus(self, *args, **kwargs) -> None:
        pass

    def ConnectPort(self, *args, **kwargs) -> None:
        pass

    def EnableArm(self, *args, **kwargs) -> None:
        pass

    def GetArmJointMsgs(self):
        joints = {f"joint_{index}": index * 1000 for index in range(1, 7)}
        return SimpleNamespace(time_stamp=123.0, joint_state=SimpleNamespace(**joints))

    def GetArmGripperMsgs(self):
        return SimpleNamespace(gripper_state=SimpleNamespace(grippers_angle=35000))

    def GetArmStatus(self):
        self.status_feedback.time_stamp = 124.0
        return self.status_feedback

    def GetArmEndPoseMsgs(self):
        return SimpleNamespace(time_stamp=125.0, end_pose=None)

    def GetArmEnableStatus(self):
        return list(self.enable_status)


class CartesianSdkDouble:
    def __init__(self) -> None:
        self.mode_calls: list[tuple[int, ...]] = []
        self.motion_calls: list[tuple[int, ...]] = []
        self.end_pose_calls: list[tuple[int, ...]] = []
        self.joint_calls: list[tuple[int, ...]] = []
        self.motion_ctrl_1_calls: list[dict[str, int]] = []
        self.enable_status = [True] * 6

    def CreateCanBus(self, *args, **kwargs) -> None:
        pass

    def ConnectPort(self, *args, **kwargs) -> None:
        pass

    def EnableArm(self, *args) -> None:
        pass

    def ModeCtrl(self, *args) -> None:
        self.mode_calls.append(args)

    def MotionCtrl_1(self, **kwargs) -> None:
        self.motion_ctrl_1_calls.append(kwargs)

    def MotionCtrl_2(self, *args) -> None:
        self.motion_calls.append(args)

    def EndPoseCtrl(self, *args) -> None:
        self.end_pose_calls.append(args)

    def JointCtrl(self, *args) -> None:
        self.joint_calls.append(args)

    def GetArmEnableStatus(self):
        return list(self.enable_status)


class CartesianFallbackSdkDouble(CartesianSdkDouble):
    ModeCtrl = None


def test_mock_driver_updates_state_after_action() -> None:
    driver = MockDriver()
    driver.connect()
    driver.enable()
    driver.send_action(Action([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], 0.25))

    state = driver.get_state()
    assert state.joint_position == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert state.gripper_position == 0.25
    assert state.is_enabled is True


def test_piper_driver_drag_teach_uses_exact_motion_ctrl_1_keywords() -> None:
    sdk = CartesianSdkDouble()
    driver = PiperDriver(sdk_client=sdk)
    driver.connect()

    driver.start_drag_teach()
    driver.stop_drag_teach()

    assert sdk.motion_ctrl_1_calls == [
        {"emergency_stop": 0x00, "track_ctrl": 0x00, "grag_teach_ctrl": 0x01},
        {"emergency_stop": 0x00, "track_ctrl": 0x00, "grag_teach_ctrl": 0x02},
    ]


def test_real_driver_streams_joint_positions_without_gripper_command() -> None:
    sdk = CartesianSdkDouble()
    driver = PiperDriver(sdk_client=sdk, move_speed_rate=25)
    driver.connect()
    driver.enable()

    driver.send_joint_positions([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])

    assert sdk.motion_calls[-1] == (0x01, 0x01, 25, 0x00, 0, 0x00)
    assert sdk.joint_calls[-1] == (5730, -11459, 17189, -22918, 28648, -34377)
    assert sdk.end_pose_calls == []


def test_mock_driver_records_master_slave_config_without_motion() -> None:
    driver = MockDriver()
    driver.connect()
    before = driver.get_state()

    driver.configure_master_slave(0xFA, 0x00, 0x00, 0x00)
    after = driver.get_state()

    assert driver.master_slave_config == (0xFA, 0x00, 0x00, 0x00)
    assert after.joint_position == before.joint_position
    assert after.gripper_position == before.gripper_position
    assert after.is_enabled == before.is_enabled
    assert driver.trajectory == []


def test_robot_state_status_defaults_are_neutral_and_independent() -> None:
    first = RobotState()
    second = RobotState()
    legacy = RobotState(1.0, [0.0] * 6, 0.5, True, 7, [0.0] * 6)

    assert legacy.error_code == 7
    assert legacy.ctrl_mode == 0
    assert legacy.joint_limit_flags == [False] * 6
    assert (
        first.ctrl_mode,
        first.arm_status,
        first.mode_feedback,
        first.teach_status,
        first.motion_status,
        first.trajectory_num,
    ) == (0, 0, 0, 0, 0, 0)
    assert first.joint_limit_flags == [False] * 6
    assert first.joint_communication_flags == [False] * 6

    first.joint_limit_flags[0] = True
    first.joint_communication_flags[1] = True
    assert second.joint_limit_flags == [False] * 6
    assert second.joint_communication_flags == [False] * 6


def test_robot_state_copies_and_validates_status_flag_lists() -> None:
    limit_flags = [True, False, True, False, True, False]
    communication_flags = [False, True, False, True, False, True]

    state = RobotState(
        joint_limit_flags=limit_flags,
        joint_communication_flags=communication_flags,
    )
    limit_flags[0] = False
    communication_flags[1] = False

    assert state.joint_limit_flags == [True, False, True, False, True, False]
    assert state.joint_communication_flags == [False, True, False, True, False, True]

    with pytest.raises(ValueError, match="Expected 6 joint limit flags"):
        RobotState(joint_limit_flags=[False] * 5)
    with pytest.raises(ValueError, match="Expected 6 joint communication flags"):
        RobotState(joint_communication_flags=[False] * 7)


def test_state_status_fields_propagate_to_observation_gui_and_mock_copy() -> None:
    state = RobotState(
        ctrl_mode=1,
        arm_status=2,
        mode_feedback=3,
        teach_status=4,
        motion_status=5,
        trajectory_num=6,
        joint_limit_flags=[True, False, False, False, False, False],
        joint_communication_flags=[False, True, False, False, False, False],
    )

    observation_state = Observation(state=state).to_dict()["state"]
    gui_state = GuiState.from_robot_state(connected=True, state=state)

    assert observation_state["mode_feedback"] == 3
    assert observation_state["joint_limit_flags"] == state.joint_limit_flags
    assert gui_state.trajectory_num == 6
    assert gui_state.joint_communication_flags == state.joint_communication_flags
    assert observation_state["joint_limit_flags"] is not state.joint_limit_flags
    assert gui_state.joint_communication_flags is not state.joint_communication_flags

    driver = MockDriver()
    driver._state = state
    driver.connect()
    copied_state = driver.get_state()
    assert copied_state.ctrl_mode == 1
    assert copied_state.joint_limit_flags == state.joint_limit_flags
    assert copied_state.joint_limit_flags is not state.joint_limit_flags

    driver.reset()
    reset_state = driver.get_state()
    assert reset_state.ctrl_mode == 0
    assert reset_state.joint_limit_flags == [False] * 6
    assert reset_state.joint_communication_flags == [False] * 6


def test_piper_driver_normalizes_sdk_shaped_status_feedback() -> None:
    driver = PiperDriver(sdk_client=StatusSdkDouble())
    driver.connect()

    state = driver.get_state()

    assert type(state.ctrl_mode) is int
    assert type(state.arm_status) is int
    assert type(state.mode_feedback) is int
    assert type(state.teach_status) is int
    assert type(state.motion_status) is int
    assert type(state.trajectory_num) is int
    assert (
        state.ctrl_mode,
        state.arm_status,
        state.mode_feedback,
        state.teach_status,
        state.motion_status,
        state.trajectory_num,
    ) == (1, 2, 3, 4, 5, 6)
    assert state.timestamp == 123.0
    assert state.is_enabled is True
    assert state.error_code == 0x150A
    assert state.joint_limit_flags == [False, True, False, True, False, True]
    assert state.joint_communication_flags == [True, False, True, False, True, False]


def test_piper_driver_unwraps_nested_fault_status_without_ctrl_mode() -> None:
    driver = PiperDriver()
    nested = SimpleNamespace(err_code=9, err_status=SimpleNamespace())

    normalized = driver._normalize_status_feedback(SimpleNamespace(arm_status=nested))

    assert normalized["error_code"] == 9


def test_piper_driver_uses_all_six_motor_enable_feedback() -> None:
    sdk = StatusSdkDouble()
    driver = PiperDriver(sdk_client=sdk)
    driver.connect()
    driver.enable()
    sdk.enable_status[3] = False

    state = driver.get_state()

    assert state.is_enabled is False
    with pytest.raises(RuntimeError, match="enabled"):
        driver.send_action(Action([0.0] * 6, 0.5))


def test_piper_driver_rejects_non_finite_cartesian_pose_before_mode_switch() -> None:
    sdk = CartesianSdkDouble()
    driver = PiperDriver(sdk_client=sdk)
    driver.connect()
    driver.enable()

    with pytest.raises(ValueError, match="finite"):
        driver.send_cartesian_pose(float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0)

    assert sdk.mode_calls == []
    assert sdk.motion_calls == []
    assert sdk.end_pose_calls == []


def test_piper_driver_prefers_mode_ctrl_for_cartesian_pose() -> None:
    sdk = CartesianSdkDouble()
    driver = PiperDriver(sdk_client=sdk, move_speed_rate=50)
    driver.connect()
    driver.enable()

    driver.send_cartesian_pose(1.25, -2.5, 3.75, 4.0, -5.0, 6.0)

    assert sdk.mode_calls == [(0x01, 0x00, 50, 0x00)]
    assert sdk.motion_calls == []
    assert sdk.end_pose_calls == [(1250, -2500, 3750, 4000, -5000, 6000)]


def test_piper_driver_falls_back_to_motion_ctrl_2_for_cartesian_pose() -> None:
    sdk = CartesianFallbackSdkDouble()
    driver = PiperDriver(sdk_client=sdk, move_speed_rate=42)
    driver.connect()
    driver.enable()

    driver.send_cartesian_pose(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    assert sdk.motion_calls == [(0x01, 0x00, 42, 0x00, 0, 0x00)]
    assert sdk.end_pose_calls == [(1000, 2000, 3000, 4000, 5000, 6000)]
