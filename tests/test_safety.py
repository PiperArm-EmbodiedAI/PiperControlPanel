from src.core.action import Action
from src.core.robot_state import RobotState
from src.core.safety import SafetyLayer


def build_safety() -> SafetyLayer:
    return SafetyLayer(
        joint_min=[-1.0] * 6,
        joint_max=[1.0] * 6,
        max_delta=[0.5] * 6,
        gripper_min=0.0,
        gripper_max=1.0,
    )


def test_safety_clamps_limits_and_delta() -> None:
    current = RobotState(joint_position=[0.0] * 6, gripper_position=0.5, is_enabled=True)
    action = Action(joint_position=[2.0, -2.0, 0.25, 0.0, 0.0, 0.0], gripper_position=2.0)

    result = build_safety().apply(action, current)

    assert result.accepted is True
    assert result.action is not None
    assert result.action.joint_position[0] == 0.5
    assert result.action.joint_position[1] == -0.5
    assert result.action.gripper_position == 1.0
    assert result.reasons


def test_safety_rejects_when_robot_disabled() -> None:
    current = RobotState(is_enabled=False)
    action = Action(joint_position=[0.0] * 6, gripper_position=0.5)

    result = build_safety().apply(action, current)

    assert result.accepted is False
    assert result.action is None
