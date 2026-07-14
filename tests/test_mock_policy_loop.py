from src.core.controller import Controller
from src.core.executor import Executor
from src.core.safety import SafetyLayer
from src.drivers.mock_driver import MockDriver
from src.policies.mock_policy import MockPolicy


def test_mock_policy_loop_runs() -> None:
    driver = MockDriver()
    driver.connect()
    driver.enable()
    safety = SafetyLayer(
        joint_min=[-3.14] * 6,
        joint_max=[3.14] * 6,
        max_delta=[0.5] * 6,
        gripper_min=0.0,
        gripper_max=1.0,
    )
    controller = Controller(
        driver=driver,
        policy=MockPolicy(mode="fixed_pose"),
        executor=Executor(driver=driver, safety_layer=safety, rate_hz=0),
        prompt="test prompt",
    )

    steps = controller.run(max_steps=3)

    assert len(steps) == 3
    assert driver.get_state().is_enabled is True
