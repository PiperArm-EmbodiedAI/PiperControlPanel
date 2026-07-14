from __future__ import annotations

import math
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from typing import Callable

from src.gui.view_model import CommandResult, GuiState, PiperGuiViewModel

try:
    from PySide6.QtCore import QSize, QTimer, Qt, Signal
    from PySide6.QtGui import QColor, QPainter, QPen
    from PySide6.QtWidgets import (
        QComboBox,
        QDoubleSpinBox,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSlider,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("PySide6 is required for the GUI. Install with: pip install -e .[gui]") from exc


class JointTrackingBar(QWidget):
    def __init__(self, lower: float, upper: float, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.lower = lower
        self.upper = upper
        self.target = lower
        self.actual = lower
        self.setMinimumWidth(160)
        self.setMinimumHeight(18)

    def set_values(self, target: float, actual: float) -> None:
        self.target = min(max(target, self.lower), self.upper)
        self.actual = min(max(actual, self.lower), self.upper)
        self.update()

    def sizeHint(self) -> QSize:  # noqa: N802
        return QSize(180, 22)

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        margin = 8
        center_y = self.height() // 2
        left = margin
        right = max(margin + 1, self.width() - margin)
        actual_x = self._value_to_x(self.actual, left, right)
        target_x = self._value_to_x(self.target, left, right)

        painter.setPen(QPen(QColor("#777777"), 4, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(left, center_y, right, center_y)
        painter.setPen(QPen(QColor("#d43c3c"), 5, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(actual_x, center_y, target_x, center_y)
        painter.setPen(QPen(QColor("#2f80ed"), 6, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(left, center_y, actual_x, center_y)
        painter.setPen(QPen(QColor("#111111"), 2))
        painter.drawLine(target_x, center_y - 7, target_x, center_y + 7)

    def _value_to_x(self, value: float, left: int, right: int) -> int:
        if self.upper <= self.lower:
            return left
        ratio = (value - self.lower) / (self.upper - self.lower)
        return int(round(left + ratio * (right - left)))


class MainWindow(QMainWindow):
    command_finished = Signal(object)
    command_failed = Signal(str)
    live_finished = Signal(object, int)
    live_failed = Signal(str)

    def __init__(self, view_model: PiperGuiViewModel) -> None:
        super().__init__()
        self.view_model = view_model
        self.setWindowTitle(f"Piper Control Tool ({view_model.mode})")
        self.joint_sliders: list[QSlider] = []
        self.joint_inputs: list[QDoubleSpinBox] = []
        self.radian_labels: list[QLabel] = []
        self.actual_degree_labels: list[QLabel] = []
        self.error_degree_labels: list[QLabel] = []
        self.joint_tracking_bars: list[JointTrackingBar] = []
        self.cartesian_inputs: dict[str, QDoubleSpinBox] = {}
        self.master_slave_offsets: dict[str, QComboBox] = {}
        self._updating_widgets = False
        self._target_dirty = False
        self._live_sending = False
        self._command_running = False
        self._live_target_version = 0
        self._live_continue_tolerance_deg = 0.2
        self._force_sync_target_on_next_render = False
        self._syncing_target_widgets = False
        self._undo_stack: list[tuple[list[float], float]] = []
        self._redo_stack: list[tuple[list[float], float]] = []
        self._buttons_disabled_while_running: list[QPushButton] = []
        self._executor = ThreadPoolExecutor(max_workers=1)
        self.command_finished.connect(self._command_finished)
        self.command_failed.connect(self._command_failed)
        self.live_finished.connect(self._live_finished)
        self.live_failed.connect(self._live_failed)
        self._build_ui()
        self._update_mode_buttons()
        poll_rate_hz = float(view_model.robot_config.get("gui", {}).get("poll_rate_hz", 5.0))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_state)
        self.timer.start(max(100, int(1000.0 / max(poll_rate_hz, 0.1))))

    def _build_ui(self) -> None:
        root = QWidget(self)
        layout = QVBoxLayout(root)

        title_row = QHBoxLayout()
        title_label = QLabel("PiperArm Control Tool V1.0")
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        byline_label = QLabel("by LHM")
        byline_label.setStyleSheet("color: #666666;")
        title_row.addWidget(title_label)
        title_row.addStretch()
        title_row.addWidget(byline_label)
        layout.addLayout(title_row)

        self.status_label = QLabel("Disconnected")
        self.error_label = QLabel("error_code: None")
        self.safety_label = QLabel("safety: OK")
        self.end_pose_label = QLabel("end_pose: unavailable")
        layout.addWidget(self.status_label)
        layout.addWidget(self.error_label)
        layout.addWidget(self.safety_label)
        layout.addWidget(self.end_pose_label)

        button_row = QHBoxLayout()
        for text, handler, disable_while_running in (
            ("Connect", self._run_command(self.view_model.connect), False),
            ("Disconnect", self._run_command(self.view_model.disconnect), False),
            ("Enable", self._run_command(self.view_model.enable), False),
            ("Disable motors", self._run_async_command(self.view_model.disable), True),
            ("Emergency stop", self._run_command(self.view_model.stop), False),
            ("Reset fault", self._run_command(self.view_model.reset_fault), False),
            ("Home target", self._run_async_command(self._send_home), True),
        ):
            button = QPushButton(text)
            button.clicked.connect(handler)
            button_row.addWidget(button)
            if disable_while_running:
                self._buttons_disabled_while_running.append(button)
        layout.addLayout(button_row)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Control mode"))
        self.control_mode = QComboBox()
        self.control_mode.addItems(["Batch: edit then Send target", "Live: slider/value sends immediately"])
        self.control_mode.currentIndexChanged.connect(self._mode_changed)
        mode_row.addWidget(self.control_mode)
        self.undo_button = QPushButton("Undo pose")
        self.undo_button.clicked.connect(self._run_async_command(self._undo_pose))
        self.redo_button = QPushButton("Redo pose")
        self.redo_button.clicked.connect(self._run_async_command(self._redo_pose))
        self._buttons_disabled_while_running.extend([self.undo_button, self.redo_button])
        mode_row.addWidget(self.undo_button)
        mode_row.addWidget(self.redo_button)
        layout.addLayout(mode_row)

        tabs = QTabWidget()
        tabs.addTab(self._build_joint_tab(), "Joint angle control")
        tabs.addTab(self._build_cartesian_tab(), "Cartesian pose control")
        tabs.addTab(self._build_master_slave_tab(), "Master / Slave setup")
        layout.addWidget(tabs)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)
        self.setCentralWidget(root)

    def _build_joint_tab(self) -> QWidget:
        tab = QWidget(self)
        layout = QVBoxLayout(tab)

        joint_grid = QGridLayout()
        joint_grid.addWidget(QLabel("Joint"), 0, 0)
        joint_grid.addWidget(QLabel("Target deg"), 0, 1)
        joint_grid.addWidget(QLabel("Target slider"), 0, 2)
        joint_grid.addWidget(QLabel("Target rad"), 0, 3)
        joint_grid.addWidget(QLabel("Actual deg"), 0, 4)
        joint_grid.addWidget(QLabel("Error deg"), 0, 5)
        joint_grid.addWidget(QLabel("Tracking"), 0, 6)
        limits = self.view_model.robot_config["joint_limits"]
        for index in range(6):
            min_deg = math.degrees(float(limits["min"][index]))
            max_deg = math.degrees(float(limits["max"][index]))
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(min_deg, max_deg)
            spin.setSingleStep(1.0)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(round(min_deg * 100.0)), int(round(max_deg * 100.0)))
            radian_label = QLabel("0.0000")
            actual_label = QLabel("0.00")
            error_label = QLabel("0.00")
            tracking_bar = JointTrackingBar(min_deg, max_deg, self)
            slider.valueChanged.connect(partial(self._slider_to_spin, index))
            spin.valueChanged.connect(partial(self._spin_to_slider, index))
            self.joint_inputs.append(spin)
            self.joint_sliders.append(slider)
            self.radian_labels.append(radian_label)
            self.actual_degree_labels.append(actual_label)
            self.error_degree_labels.append(error_label)
            self.joint_tracking_bars.append(tracking_bar)
            joint_grid.addWidget(QLabel(f"J{index + 1}"), index + 1, 0)
            joint_grid.addWidget(spin, index + 1, 1)
            joint_grid.addWidget(slider, index + 1, 2)
            joint_grid.addWidget(radian_label, index + 1, 3)
            joint_grid.addWidget(actual_label, index + 1, 4)
            joint_grid.addWidget(error_label, index + 1, 5)
            joint_grid.addWidget(tracking_bar, index + 1, 6)
        layout.addLayout(joint_grid)

        gripper_row = QHBoxLayout()
        self.gripper_input = QDoubleSpinBox()
        self.gripper_input.setDecimals(3)
        self.gripper_input.setRange(
            float(self.view_model.robot_config["gripper"]["min"]),
            float(self.view_model.robot_config["gripper"]["max"]),
        )
        self.gripper_input.setSingleStep(0.05)
        self.gripper_input.valueChanged.connect(self._gripper_changed)
        gripper_row.addWidget(QLabel("Gripper target"))
        gripper_row.addWidget(self.gripper_input)
        open_button = QPushButton("Set open target")
        open_button.clicked.connect(partial(self._set_gripper_target, self.view_model.robot_config["gripper"]["open"]))
        close_button = QPushButton("Set close target")
        close_button.clicked.connect(partial(self._set_gripper_target, self.view_model.robot_config["gripper"]["closed"]))
        send_button = QPushButton("Send target")
        send_button.clicked.connect(self._run_async_command(self._send_target_with_history))
        self._buttons_disabled_while_running.extend([open_button, close_button, send_button])
        gripper_row.addWidget(open_button)
        gripper_row.addWidget(close_button)
        gripper_row.addWidget(send_button)
        layout.addLayout(gripper_row)
        return tab

    def _build_cartesian_tab(self) -> QWidget:
        tab = QWidget(self)
        layout = QVBoxLayout(tab)
        layout.addWidget(QLabel("Experimental MoveP Cartesian control. Position is mm; rotation is degrees."))
        self.cartesian_feedback_label = QLabel("current end pose: unavailable")
        layout.addWidget(self.cartesian_feedback_label)
        if not self.view_model.robot_config.get("gui", {}).get("experimental_cartesian_enabled", False):
            layout.addWidget(QLabel("Sending is disabled by config: robot.gui.experimental_cartesian_enabled=false"))
        grid = QGridLayout()
        fields = (
            ("X mm", "x", -500.0, 500.0, 1.0),
            ("Y mm", "y", -500.0, 500.0, 1.0),
            ("Z mm", "z", 0.0, 600.0, 1.0),
            ("RX deg", "rx", -180.0, 180.0, 1.0),
            ("RY deg", "ry", -180.0, 180.0, 1.0),
            ("RZ deg", "rz", -180.0, 180.0, 1.0),
        )
        for row, (label, key, lower, upper, step) in enumerate(fields):
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(lower, upper)
            spin.setSingleStep(step)
            self.cartesian_inputs[key] = spin
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(spin, row, 1)
        layout.addLayout(grid)
        send = QPushButton("Send Cartesian target")
        send.clicked.connect(self._run_async_command(self._send_cartesian_target))
        send.setEnabled(bool(self.view_model.robot_config.get("gui", {}).get("experimental_cartesian_enabled", False)))
        self._buttons_disabled_while_running.append(send)
        layout.addWidget(send)
        return tab

    def _build_master_slave_tab(self) -> QWidget:
        tab = QWidget(self)
        layout = QVBoxLayout(tab)
        gui_config = self.view_model.robot_config.get("gui", {})
        layout.addWidget(
            QLabel(
                "System-level Piper MasterSlaveConfig. This does not enable motors or send motion commands. "
                "Wrong role/offset values can break master/slave routing."
            )
        )
        if not gui_config.get("master_slave_config_enabled", False):
            layout.addWidget(QLabel("Sending is disabled by config: robot.gui.master_slave_config_enabled=false"))

        role_row = QHBoxLayout()
        role_row.addWidget(QLabel("Role"))
        self.master_slave_role = QComboBox()
        self.master_slave_role.addItem("Master / teaching input arm (0xFA)", "master")
        self.master_slave_role.addItem("Slave / motion output arm (0xFC)", "slave")
        role_row.addWidget(self.master_slave_role)
        layout.addLayout(role_row)

        if gui_config.get("master_slave_advanced_offsets_enabled", False):
            grid = QGridLayout()
            for row, (label, key) in enumerate(
                (
                    ("Feedback offset", "feedback"),
                    ("Control offset", "ctrl"),
                    ("Linkage address offset", "linkage"),
                )
            ):
                combo = QComboBox()
                for value in (0x00, 0x10, 0x20):
                    combo.addItem(f"0x{value:02X}", value)
                self.master_slave_offsets[key] = combo
                grid.addWidget(QLabel(label), row, 0)
                grid.addWidget(combo, row, 1)
            layout.addWidget(QLabel("Advanced offsets are deployment-specific. Use non-zero values only with official guidance."))
            layout.addLayout(grid)
        else:
            layout.addWidget(QLabel("Offsets use safe default: feedback=0x00, control=0x00, linkage=0x00"))

        apply_button = QPushButton("Apply Master/Slave Config")
        apply_button.clicked.connect(self._run_async_command(self._apply_master_slave_config))
        apply_button.setEnabled(bool(gui_config.get("master_slave_config_enabled", False)))
        self._buttons_disabled_while_running.append(apply_button)
        layout.addWidget(apply_button)
        return tab

    def _run_command(self, command: Callable[[], CommandResult | None]):
        def wrapped() -> None:
            try:
                result = command()
                if result is not None:
                    self._append_result(result)
                self.refresh_state()
            except Exception as exc:
                self._set_safety_message([str(exc)])
                self._append_log(f"ERROR: {exc}")
        return wrapped

    def _run_async_command(self, command: Callable[[], CommandResult | None]):
        def wrapped() -> None:
            if self._command_running:
                self._append_log("command ignored: another command is still running")
                return
            self._command_running = True
            self._set_command_buttons_enabled(False)
            self.timer.stop()
            self._append_log("command running...")
            future = self._executor.submit(command)
            future.add_done_callback(self._async_done)
        return wrapped

    def _async_done(self, future) -> None:
        try:
            self.command_finished.emit(future.result())
        except Exception as exc:
            self.command_failed.emit(str(exc))

    def _command_finished(self, result: CommandResult | None) -> None:
        self._command_running = False
        self._set_command_buttons_enabled(True)
        if result is not None:
            self._append_result(result)
        self._append_log("command finished")
        self.refresh_state()
        self.timer.start()

    def _command_failed(self, message: str) -> None:
        self._command_running = False
        self._set_command_buttons_enabled(True)
        self._set_safety_message([message])
        self._append_log(f"ERROR: {message}")
        self.refresh_state()
        self.timer.start()

    def _current_target_degrees(self) -> tuple[list[float], float]:
        return [spin.value() for spin in self.joint_inputs], self.gripper_input.value()

    def _current_state_pose_degrees(self) -> tuple[list[float], float]:
        state = self.view_model.read_state()
        return [math.degrees(value) for value in state.joint_position], state.gripper_position

    def _set_target_widgets_degrees(self, joints_deg: list[float], gripper: float, dirty: bool = True) -> None:
        self._updating_widgets = True
        try:
            for index, value in enumerate(joints_deg):
                self.joint_inputs[index].setValue(value)
                self.joint_sliders[index].setValue(int(round(value * 100.0)))
                self.radian_labels[index].setText(f"{math.radians(value):.4f}")
            self.gripper_input.setValue(gripper)
        finally:
            self._updating_widgets = False
        self._target_dirty = dirty

    def _send_target(self) -> CommandResult:
        joints_deg, gripper = self._current_target_degrees()
        result = self.view_model.send_joints_degrees(joints_deg, gripper)
        self._target_dirty = False
        return result

    def _send_target_with_history(self) -> CommandResult:
        if not self._live_mode_enabled():
            self._undo_stack.append(self._current_state_pose_degrees())
            self._redo_stack.clear()
        return self._send_target()

    def _send_home(self) -> CommandResult:
        self._undo_stack.append(self._current_state_pose_degrees())
        self._redo_stack.clear()
        result = self.view_model.home()
        self._target_dirty = False
        self._force_sync_target_on_next_render = True
        return result

    def _undo_pose(self) -> CommandResult:
        if self._live_mode_enabled():
            return CommandResult("Undo is only available in Batch mode")
        if not self._undo_stack:
            return CommandResult("No previous pose to undo")
        self._redo_stack.append(self._current_state_pose_degrees())
        joints_deg, gripper = self._undo_stack.pop()
        self._set_target_widgets_degrees(joints_deg, gripper, dirty=True)
        return self._send_target()

    def _redo_pose(self) -> CommandResult:
        if self._live_mode_enabled():
            return CommandResult("Redo is only available in Batch mode")
        if not self._redo_stack:
            return CommandResult("No next pose to redo")
        self._undo_stack.append(self._current_state_pose_degrees())
        joints_deg, gripper = self._redo_stack.pop()
        self._set_target_widgets_degrees(joints_deg, gripper, dirty=True)
        return self._send_target()

    def _send_cartesian_target(self) -> CommandResult:
        return self.view_model.send_cartesian_pose(
            self.cartesian_inputs["x"].value(),
            self.cartesian_inputs["y"].value(),
            self.cartesian_inputs["z"].value(),
            self.cartesian_inputs["rx"].value(),
            self.cartesian_inputs["ry"].value(),
            self.cartesian_inputs["rz"].value(),
        )

    def _apply_master_slave_config(self) -> CommandResult:
        feedback_offset = self._master_slave_offset_value("feedback")
        ctrl_offset = self._master_slave_offset_value("ctrl")
        linkage_offset = self._master_slave_offset_value("linkage")
        return self.view_model.configure_master_slave(
            self.master_slave_role.currentData(),
            feedback_offset,
            ctrl_offset,
            linkage_offset,
        )

    def _master_slave_offset_value(self, key: str) -> int:
        combo = self.master_slave_offsets.get(key)
        if combo is None:
            return 0x00
        return int(combo.currentData())

    def _set_gripper_target(self, value: float) -> None:
        self.gripper_input.setValue(value)
        self._target_dirty = True
        if self._live_mode_enabled():
            self._send_live_target()

    def _live_mode_enabled(self) -> bool:
        return self.control_mode.currentIndex() == 1

    def _mode_changed(self) -> None:
        try:
            current_pose = self._current_state_pose_degrees()
            self._undo_stack.append(current_pose)
            if self._live_mode_enabled():
                joints_deg, gripper = current_pose
                self._set_target_widgets_degrees(joints_deg, gripper, dirty=False)
        except Exception:
            pass
        self._redo_stack.clear()
        self._target_dirty = False
        self._update_mode_buttons()
        self.refresh_state()

    def _update_mode_buttons(self) -> None:
        batch_enabled = not self._live_mode_enabled() and not self._command_running
        self.undo_button.setEnabled(batch_enabled)
        self.redo_button.setEnabled(batch_enabled)

    def _set_command_buttons_enabled(self, enabled: bool) -> None:
        for button in self._buttons_disabled_while_running:
            button.setEnabled(enabled)
        self._update_mode_buttons()

    def _slider_to_spin(self, index: int, value: int) -> None:
        if self._updating_widgets or self._syncing_target_widgets:
            return
        self._target_dirty = True
        self._mark_live_target_changed()
        self._syncing_target_widgets = True
        try:
            degrees = value / 100.0
            self.joint_inputs[index].setValue(degrees)
            self.radian_labels[index].setText(f"{math.radians(degrees):.4f}")
            self._update_joint_tracking_bar(index)
        finally:
            self._syncing_target_widgets = False
        if self._live_mode_enabled():
            self._send_live_target()

    def _spin_to_slider(self, index: int, value: float) -> None:
        if self._updating_widgets or self._syncing_target_widgets:
            return
        self._target_dirty = True
        self._mark_live_target_changed()
        self._syncing_target_widgets = True
        try:
            self.joint_sliders[index].setValue(int(round(value * 100.0)))
            self.radian_labels[index].setText(f"{math.radians(value):.4f}")
            self._update_joint_tracking_bar(index)
        finally:
            self._syncing_target_widgets = False
        if self._live_mode_enabled():
            self._send_live_target()

    def _gripper_changed(self, value: float) -> None:
        if self._updating_widgets:
            return
        self._target_dirty = True
        self._mark_live_target_changed()
        if self._live_mode_enabled():
            self._send_live_target()

    def _mark_live_target_changed(self) -> None:
        if self._live_mode_enabled():
            self._live_target_version += 1

    def _send_live_target(self) -> None:
        if self._command_running or self._live_sending:
            return
        self._live_sending = True
        version = self._live_target_version
        joints_deg, gripper = self._current_target_degrees()
        future = self._executor.submit(self.view_model.send_joints_degrees, joints_deg, gripper, False)
        future.add_done_callback(partial(self._live_done, version=version))

    def _live_done(self, future, version: int) -> None:
        try:
            result = future.result()
            result.message = "live target sent"
            self.live_finished.emit(result, version)
        except Exception as exc:
            self.live_failed.emit(str(exc))

    def _live_finished(self, result: CommandResult, version: int) -> None:
        self._live_sending = False
        self._append_result(result)
        self.refresh_state()
        if not self._live_mode_enabled():
            return
        if self._live_target_version != version or self._live_target_error_exceeds_tolerance():
            self._send_live_target()

    def _live_failed(self, message: str) -> None:
        self._live_sending = False
        self._set_safety_message([message])
        self._append_log(f"ERROR: {message}")
        self.refresh_state()

    def _live_target_error_exceeds_tolerance(self) -> bool:
        for index, label in enumerate(self.actual_degree_labels):
            try:
                actual = float(label.text())
            except ValueError:
                continue
            if abs(self.joint_inputs[index].value() - actual) > self._live_continue_tolerance_deg:
                return True
        return abs(self.gripper_input.value() - self._last_actual_gripper()) > 0.005

    def _last_actual_gripper(self) -> float:
        status = self.status_label.text()
        marker = "gripper="
        if marker not in status:
            return self.gripper_input.value()
        try:
            return float(status.split(marker, 1)[1].split()[0])
        except (IndexError, ValueError):
            return self.gripper_input.value()

    def refresh_state(self) -> None:
        try:
            state = self.view_model.read_state()
        except Exception as exc:
            self.status_label.setText(f"Disconnected / read failed: {exc}")
            return
        self._render_state(state)

    def _render_state(self, state: GuiState) -> None:
        self.status_label.setText(
            f"connected={state.connected} enabled={state.enabled} gripper={state.gripper_position:.3f}"
        )
        self.error_label.setText(f"error_code: {state.error_code}")
        if state.end_pose is None:
            end_pose_text = "end_pose: unavailable"
        else:
            end_pose_text = (
                "end_pose: "
                f"X={state.end_pose[0]:.1f}mm Y={state.end_pose[1]:.1f}mm Z={state.end_pose[2]:.1f}mm "
                f"RX={math.degrees(state.end_pose[3]):.1f}deg RY={math.degrees(state.end_pose[4]):.1f}deg RZ={math.degrees(state.end_pose[5]):.1f}deg"
            )
        self.end_pose_label.setText(end_pose_text)
        self.cartesian_feedback_label.setText("current " + end_pose_text)
        joints_deg = [math.degrees(value) for value in state.joint_position]
        self._update_actual_joint_feedback(joints_deg)
        if self._force_sync_target_on_next_render:
            self._force_sync_target_on_next_render = False
            self._set_target_widgets_degrees(joints_deg, state.gripper_position, dirty=False)
            self._live_target_version += 1
            self._update_actual_joint_feedback(joints_deg)
            return
        if self._live_mode_enabled() or self._target_dirty:
            return
        self._set_target_widgets_degrees(joints_deg, state.gripper_position, dirty=False)

    def _update_actual_joint_feedback(self, actual_joints_deg: list[float]) -> None:
        for index, actual in enumerate(actual_joints_deg):
            target = self.joint_inputs[index].value()
            error = target - actual
            self.actual_degree_labels[index].setText(f"{actual:.2f}")
            self.error_degree_labels[index].setText(f"{error:+.2f}")
            if abs(error) > 0.1:
                self.error_degree_labels[index].setStyleSheet("color: #d43c3c;")
            else:
                self.error_degree_labels[index].setStyleSheet("color: #2f7d32;")
            self.joint_tracking_bars[index].set_values(target, actual)

    def _update_joint_tracking_bar(self, index: int) -> None:
        if index >= len(self.actual_degree_labels):
            return
        try:
            actual = float(self.actual_degree_labels[index].text())
        except ValueError:
            actual = self.joint_inputs[index].value()
        target = self.joint_inputs[index].value()
        error = target - actual
        self.error_degree_labels[index].setText(f"{error:+.2f}")
        self.joint_tracking_bars[index].set_values(target, actual)

    def _append_result(self, result: CommandResult) -> None:
        self._set_safety_message(result.safety_reasons)
        self._append_log(result.message)
        if result.safety_reasons:
            self._append_log("Safety adjusted: " + "; ".join(result.safety_reasons))
        if result.requested_action is not None:
            self._append_log(
                "requested rad="
                + self._format_joints(result.requested_action.joint_position)
                + f" gripper={result.requested_action.gripper_position:.3f}"
            )
        if result.sent_action is not None:
            self._append_log(
                "sent to driver rad="
                + self._format_joints(result.sent_action.joint_position)
                + f" gripper={result.sent_action.gripper_position:.3f}"
            )
            self._append_log(
                "sent to Piper SDK units joint(deg*1000)="
                + str(result.sdk_joint_units)
                + f" gripper(0.001mm)={result.sdk_gripper_units}"
            )
        if result.cartesian_sdk_units is not None:
            self._append_log("sent cartesian SDK units X/Y/Z/RX/RY/RZ=" + str(result.cartesian_sdk_units))

    def _format_joints(self, joints: list[float]) -> str:
        return "[" + ", ".join(f"{value:.4f}" for value in joints) + "]"

    def _set_safety_message(self, reasons: list[str]) -> None:
        self.safety_label.setText("safety: OK" if not reasons else "safety: " + "; ".join(reasons))

    def _append_log(self, text: str) -> None:
        self.log.append(text)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.timer.stop()
        try:
            self._executor.shutdown(wait=False, cancel_futures=True)
            self.view_model.close()
        finally:
            event.accept()
