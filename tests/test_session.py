import math

import pytest

from src.app.factory import AppFactory
from src.app.session import RobotSession
from src.drivers.mock_driver import MockDriver


class RecordingMockDriver(MockDriver):
    def __init__(self, stale_feedback: bool = False) -> None:
        super().__init__()
        self.stale_feedback = stale_feedback
        self.disable_calls = 0
        self.stop_calls = 0

    def disable(self) -> None:
        self.disable_calls += 1
        super().disable()

    def stop(self) -> None:
        self.stop_calls += 1
        super().stop()

    def get_state(self):
        state = super().get_state()
        if self.stale_feedback:
            state.timestamp = 1.0
        return state


def replace_session_driver(session: RobotSession, driver: RecordingMockDriver) -> None:
    session.driver = driver
    session.manual_control.driver = driver
    session.executor.driver = driver


def test_real_driver_uses_configured_move_speed_rate() -> None:
    driver = AppFactory().build_driver("real")

    assert driver.move_speed_rate == 50


def test_mock_session_reconnect_and_disconnect() -> None:
    session = RobotSession.from_config(mode="mock")

    session.connect()
    assert session.connected is True
    state = session.read_state()
    assert state.is_enabled is False

    session.enable()
    assert session.read_state().is_enabled is True

    session.disconnect()
    assert session.connected is False

    session.connect()
    assert session.connected is True
    session.close()
    assert session.connected is False


def test_mock_session_manual_home_uses_executor() -> None:
    session = RobotSession.from_config(mode="mock")
    session.connect()
    session.enable()

    session.send_joint_action([0.1, 0.1, 0.1, 0.1, 0.1, 0.1], 0.5)
    session.home()

    state = session.read_state()
    assert state.joint_position == pytest.approx(
        [-math.pi / 2.0, math.pi / 3.0, -math.pi / 3.0, 0.0, 0.0, 0.0]
    )
    assert state.gripper_position == 1.0
    session.close()


def test_explicit_disable_parks_with_advancing_stable_feedback() -> None:
    session = RobotSession.from_config(mode="mock")
    driver = RecordingMockDriver()
    replace_session_driver(session, driver)
    session.factory.robot_config["shutdown"]["park"].update(
        {"stable_samples": 2, "poll_interval_s": 0.0, "timeout_s": 1.0}
    )
    session.connect()
    session.enable()

    session.disable()

    expected = [math.radians(value) for value in [-90.0, 0.0, 0.0, 0.0, 40.0, 0.0]]
    assert driver.trajectory[-2].joint_position == pytest.approx(expected)
    assert driver.trajectory[-1].joint_position == pytest.approx(expected)
    assert driver.disable_calls == 1
    assert session.read_state().is_enabled is False
    session.factory.robot_config["shutdown"]["auto_disable"] = False
    session.close()


def test_shutdown_auto_disable_uses_same_park_flow() -> None:
    session = RobotSession.from_config(mode="mock")
    driver = RecordingMockDriver()
    replace_session_driver(session, driver)
    session.factory.robot_config["shutdown"]["park"].update(
        {"stable_samples": 1, "poll_interval_s": 0.0, "timeout_s": 1.0}
    )
    session.connect()
    session.enable()

    session.close()

    expected = [math.radians(value) for value in [-90.0, 0.0, 0.0, 0.0, 40.0, 0.0]]
    assert driver.disable_calls == 1
    assert driver.trajectory
    assert driver.trajectory[-1].joint_position == pytest.approx(expected)
    assert session.connected is False


def test_disable_fails_closed_on_non_advancing_feedback() -> None:
    session = RobotSession.from_config(mode="mock")
    driver = RecordingMockDriver(stale_feedback=True)
    replace_session_driver(session, driver)
    session.factory.robot_config["shutdown"]["park"].update(
        {"stable_samples": 1, "poll_interval_s": 0.0, "timeout_s": 1.0}
    )
    session.connect()
    session.enable()

    with pytest.raises(RuntimeError, match="did not advance"):
        session.disable()

    assert driver.disable_calls == 0
    assert driver._state.is_enabled is True
    session.factory.robot_config["shutdown"]["auto_disable"] = False
    session.close()


def test_emergency_stop_is_immediate_and_does_not_park_or_disable() -> None:
    session = RobotSession.from_config(mode="mock")
    driver = RecordingMockDriver()
    replace_session_driver(session, driver)
    session.connect()
    session.enable()

    session.stop()

    assert driver.stop_calls == 1
    assert driver.disable_calls == 0
    assert driver.trajectory == []
    assert session.read_state().is_enabled is True
    session.factory.robot_config["shutdown"]["auto_disable"] = False
    session.close()


def test_mock_session_configures_master_slave_without_enable() -> None:
    session = RobotSession.from_config(mode="mock")
    session.connect()

    session.configure_master_slave(0xFC, 0x00, 0x00, 0x00)

    assert session.driver.master_slave_config == (0xFC, 0x00, 0x00, 0x00)
    assert session.read_state().is_enabled is False
    session.close()
