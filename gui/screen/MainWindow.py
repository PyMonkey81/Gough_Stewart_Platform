# gui/screen/MainWindow.py
import time
import numpy as np
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QMessageBox, QGroupBox, QComboBox,
    QStackedWidget, QSlider, QDoubleSpinBox,
    QProgressBar, QListWidget
)
from PySide6.QtCore import Qt, QTimer

from gui.graphics.platform import PlatformCanvas
from gui.graphics.actuator import ActuatorCanvas
from gui.dialog.serialconfigdialog import SerialConfigDialog
from gui.dialog.parameterdiag import ParametersDialog, load_parameters
from gui.dialog.axisconfigdialog import AxisConfigDialog
from connection.serial_manager import SerialManager

from kinematics.inverse import inverse_kinematics
from kinematics.pose import pose_to_q
from config.parameters import (
    D, ACTUATOR_MIN, ACTUATOR_MAX, ACTUATOR_HOME_PERCENT,
    ALPHA_BETA_LIMIT_DEG, CARTESIAN_LIMITS, CARTESIAN_STEP
)
from trajectory.generator import TrajectoryConfig, TrajectoryGenerator

BG_MAIN = "#0d1b2a"
BG_CARD = "#1b263b"
TEXT_MAIN = "#e0e6ed"
ACCENT_GREEN = "#2e7d32"
ACCENT_GREEN_BRIGHT = "#43a047"
ACCENT_RED = "#c62828"
ACCENT_RED_BRIGHT = "#ef5350"
AXIS_COLORS = ["#ff6b6b", "#4ecdc4", "#45b7d1", "#96ceb4", "#ffeaa7", "#dfe6e9"]


def length_to_percent(q: np.ndarray) -> np.ndarray:
    """Mapea longitudes de actuador (m) a porcentaje de carrera 0-100%."""
    percent = (q - ACTUATOR_MIN) / (ACTUATOR_MAX - ACTUATOR_MIN) * 100.0
    return np.clip(percent, 0, 100)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stewart Platform - HMI")
        self.setMinimumSize(1280, 800)
        self.resize(1600, 900)

        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background-color: {BG_MAIN}; color: {TEXT_MAIN}; }}
            QPushButton {{
                background-color: #22344a; color: {TEXT_MAIN};
                border: 1px solid #33475e; border-radius: 6px;
                padding: 9px 16px; font-size: 13px; font-weight: 500;
            }}
            QPushButton:hover {{ background-color: #2c4157; border-color: #4a6482; }}
            QPushButton:pressed {{ background-color: #1a2a3a; }}
            QPushButton:disabled {{ color: #667; border-color: #2a3a4a; }}
            QPushButton#btn_start {{ background-color: {ACCENT_GREEN}; border-color: {ACCENT_GREEN_BRIGHT}; }}
            QPushButton#btn_start:hover {{ background-color: {ACCENT_GREEN_BRIGHT}; }}
            QPushButton#btn_stop {{ background-color: {ACCENT_RED}; border-color: {ACCENT_RED_BRIGHT}; }}
            QPushButton#btn_stop:hover {{ background-color: {ACCENT_RED_BRIGHT}; }}
            QPushButton#btn_goto {{ background-color: #0d47a1; border-color: #1565c0; }}
            QPushButton#btn_goto:hover {{ background-color: #1565c0; }}
            QLabel#badge {{ font-size: 13px; font-weight: bold; padding: 4px 10px; }}
            QGroupBox {{
                background-color: {BG_CARD};
                border: 1px solid #2c3e50;
                border-radius: 8px;
                margin-top: 14px;
                font-weight: bold;
                color: #9fb3c8;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }}
            QDoubleSpinBox, QComboBox {{
                background-color: #12202f; color: {TEXT_MAIN};
                border: 1px solid #33475e; border-radius: 4px;
                padding: 4px 6px; min-width: 80px;
            }}
            QSlider::groove:horizontal {{ background: #12202f; height: 6px; border-radius: 3px; }}
            QSlider::handle:horizontal {{
                background: #4fc3f7; width: 14px; margin: -5px 0; border-radius: 7px;
            }}
            QProgressBar {{
                border: 1px solid #33475e; border-radius: 4px;
                text-align: center; background-color: #12202f; color: {TEXT_MAIN};
            }}
            QListWidget {{
                background-color: #12202f; color: #9fb3c8;
                border: 1px solid #2c3e50; border-radius: 4px; font-family: Consolas, monospace;
            }}
            QCheckBox {{ color: {TEXT_MAIN}; }}
        """)

        # ------------------ Estado ------------------
        self.running = False
        self.op_mode = "JOG"             # "JOG" | "TRACK"
        self.motion_mode = "TAREA"       # "TAREA" | "CARTESIAN" (solo aplica en JOG)
        self.t = 0.0
        self.dt = 0.03
        self.demo_duration = 60.0
        self._oor_active = False         # guarda de "pose fuera de carrera" (cartesiano)

        # Pose deseada actual (modo TAREA, usa TIK/PIK sin modificar)
        self.y_desired = np.array([0.0, 0.0])   # [a, b] rad

        # Resultados del último cálculo de cinemática
        self.q_actuator = np.zeros(6)
        self.q_percent = np.full(6, ACTUATOR_HOME_PERCENT, dtype=float)
        self.last_percent = np.full(6, ACTUATOR_HOME_PERCENT, dtype=float)
        self.da = D.copy()
        self.R = np.eye(3)

        self.current_params = load_parameters()

        # Diálogo de jog manual de ejes (no modal)
        self.axis_dialog = AxisConfigDialog(self)

        # Serial
        self.serial_manager = SerialManager(self)
        self.serial_manager.connected_changed.connect(self.on_serial_connected)
        self.serial_manager.message_received.connect(self.on_serial_message)
        self.serial_manager.error_occurred.connect(self.on_serial_error)

        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.setInterval(1000)
        self.heartbeat_timer.timeout.connect(self.send_heartbeat)

        self._last_send = -1.0

        self.setup_ui()
        self.on_parameters_changed(self.current_params)
        self._seed_home_display()
        self.update_status_bar()

    # ------------------------------------------------------------------
    # Construcción de la UI
    # ------------------------------------------------------------------
    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        root.addLayout(self._build_top_bar())
        root.addLayout(self._build_secondary_bar())

        columns = QHBoxLayout()
        columns.setSpacing(10)
        columns.addWidget(self._build_left_column(), stretch=4)
        columns.addWidget(self._build_center_column(), stretch=3)
        columns.addWidget(self._build_right_column(), stretch=3)
        root.addLayout(columns, stretch=1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.setInterval(int(self.dt * 1000))   # 30 ms (~33 Hz)

    def _build_top_bar(self):
        bar = QHBoxLayout()
        bar.setSpacing(16)

        self.lbl_system_state = QLabel("● SYSTEM STOPPED")
        self.lbl_system_state.setObjectName("badge")
        self.lbl_serial_state = QLabel("SERIAL: DESCONECTADO")
        self.lbl_serial_state.setObjectName("badge")
        self.lbl_op_mode_badge = QLabel("JOG")
        self.lbl_op_mode_badge.setObjectName("badge")
        self.lbl_motion_mode_badge = QLabel("TAREA")
        self.lbl_motion_mode_badge.setObjectName("badge")

        bar.addWidget(self.lbl_system_state)
        bar.addWidget(self.lbl_serial_state)
        bar.addWidget(self.lbl_op_mode_badge)
        bar.addWidget(self.lbl_motion_mode_badge)
        bar.addStretch()

        self.btn_home = QPushButton("HOME")
        self.btn_home.clicked.connect(self.go_home)

        self.btn_start = QPushButton("INICIAR")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.clicked.connect(self.start)

        self.btn_stop = QPushButton("PARO")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.clicked.connect(self.stop)

        bar.addWidget(self.btn_home)
        bar.addWidget(self.btn_start)
        bar.addWidget(self.btn_stop)
        return bar

    def _build_secondary_bar(self):
        bar = QHBoxLayout()
        bar.setSpacing(10)

        self.btn_serial = QPushButton("Comunicación")
        self.btn_serial.clicked.connect(self.open_serial_config)

        self.btn_axis = QPushButton("Jog por eje")
        self.btn_axis.clicked.connect(self.open_axis_dialog)

        self.btn_params = QPushButton("Parámetros")
        self.btn_params.clicked.connect(self.open_parameter_dialog)

        bar.addWidget(self.btn_serial)
        bar.addWidget(self.btn_axis)
        bar.addWidget(self.btn_params)
        bar.addStretch()
        return bar

    def _build_left_column(self):
        col = QWidget()
        layout = QVBoxLayout(col)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        gb_platform = QGroupBox("STEWART 6-DOF")
        gb_platform_layout = QVBoxLayout(gb_platform)
        self.platform_canvas = PlatformCanvas()
        gb_platform_layout.addWidget(self.platform_canvas)

        gb_actuators = QGroupBox("ACTUADORES")
        gb_actuators_layout = QVBoxLayout(gb_actuators)
        self.actuator_canvas = ActuatorCanvas()
        gb_actuators_layout.addWidget(self.actuator_canvas)

        layout.addWidget(gb_platform, stretch=3)
        layout.addWidget(gb_actuators, stretch=2)
        return col

    def _build_center_column(self):
        col = QWidget()
        layout = QVBoxLayout(col)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # ---------- CURRENT POSE ----------
        gb_pose = QGroupBox("CURRENT POSE")
        pose_layout = QVBoxLayout(gb_pose)
        self.pose_stack = QStackedWidget()

        big_font = "font-size: 26px; font-weight: bold; color: #4fc3f7;"

        page_tarea = QWidget()
        grid_tarea = QGridLayout(page_tarea)
        self.lbl_pose_alpha = QLabel("0.0 °")
        self.lbl_pose_alpha.setStyleSheet(big_font)
        self.lbl_pose_beta = QLabel("0.0 °")
        self.lbl_pose_beta.setStyleSheet(big_font)
        grid_tarea.addWidget(QLabel("α target:"), 0, 0)
        grid_tarea.addWidget(self.lbl_pose_alpha, 0, 1)
        grid_tarea.addWidget(QLabel("β target:"), 1, 0)
        grid_tarea.addWidget(self.lbl_pose_beta, 1, 1)

        page_cart = QWidget()
        grid_cart = QGridLayout(page_cart)
        self.lbl_pose_x = QLabel("0.0 mm")
        self.lbl_pose_y = QLabel("0.0 mm")
        self.lbl_pose_z = QLabel("0.0 mm")
        self.lbl_pose_roll = QLabel("0.0 °")
        self.lbl_pose_pitch = QLabel("0.0 °")
        self.lbl_pose_yaw = QLabel("0.0 °")
        for i, (name, lbl) in enumerate([
            ("X target:", self.lbl_pose_x), ("Y target:", self.lbl_pose_y),
            ("Z target:", self.lbl_pose_z), ("Roll target:", self.lbl_pose_roll),
            ("Pitch target:", self.lbl_pose_pitch), ("Yaw target:", self.lbl_pose_yaw),
        ]):
            grid_cart.addWidget(QLabel(name), i, 0)
            grid_cart.addWidget(lbl, i, 1)

        page_track = QWidget()
        grid_track = QGridLayout(page_track)
        self.lbl_track_alpha = QLabel("0.0 °")
        self.lbl_track_alpha.setStyleSheet(big_font)
        self.lbl_track_beta = QLabel("0.0 °")
        self.lbl_track_beta.setStyleSheet(big_font)
        self.lbl_track_phase = QLabel("-")
        self.lbl_track_phase.setStyleSheet("font-weight: bold; color: #ffca28;")
        grid_track.addWidget(QLabel("α trayectoria:"), 0, 0)
        grid_track.addWidget(self.lbl_track_alpha, 0, 1)
        grid_track.addWidget(QLabel("β trayectoria:"), 1, 0)
        grid_track.addWidget(self.lbl_track_beta, 1, 1)
        grid_track.addWidget(QLabel("Fase:"), 2, 0)
        grid_track.addWidget(self.lbl_track_phase, 2, 1)

        self.pose_stack.addWidget(page_tarea)
        self.pose_stack.addWidget(page_cart)
        self.pose_stack.addWidget(page_track)
        pose_layout.addWidget(self.pose_stack)
        layout.addWidget(gb_pose)

        # ---------- ACTUATOR STATUS ----------
        gb_status = QGroupBox("ACTUATOR STATUS")
        status_layout = QGridLayout(gb_status)
        self.axis_bars = []
        self.axis_pct_labels = []
        for i in range(6):
            lbl_name = QLabel(f"L{i + 1}")
            lbl_name.setStyleSheet(f"color: {AXIS_COLORS[i]}; font-weight: bold;")
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(int(ACTUATOR_HOME_PERCENT))
            bar.setTextVisible(False)
            pct_label = QLabel(f"{ACTUATOR_HOME_PERCENT:.1f}%")
            pct_label.setMinimumWidth(48)

            status_layout.addWidget(lbl_name, i, 0)
            status_layout.addWidget(bar, i, 1)
            status_layout.addWidget(pct_label, i, 2)

            self.axis_bars.append(bar)
            self.axis_pct_labels.append(pct_label)

        layout.addWidget(gb_status)
        layout.addStretch()
        return col

    def _build_right_column(self):
        col = QWidget()
        layout = QVBoxLayout(col)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # ---------- MOTION CONTROL ----------
        gb_motion = QGroupBox("MOTION CONTROL")
        motion_layout = QVBoxLayout(gb_motion)

        op_row = QHBoxLayout()
        op_row.addWidget(QLabel("Operación:"))
        self.combo_op_mode = QComboBox()
        self.combo_op_mode.addItems(["JOG", "TRACK"])
        self.combo_op_mode.currentIndexChanged.connect(self._on_op_mode_selected)
        op_row.addWidget(self.combo_op_mode)
        op_row.addStretch()
        motion_layout.addLayout(op_row)

        selector_row = QHBoxLayout()
        self.lbl_motion_selector = QLabel("Modo:")
        selector_row.addWidget(self.lbl_motion_selector)
        self.combo_motion_mode = QComboBox()
        self.combo_motion_mode.addItems(["TAREA", "CARTESIANO"])
        self.combo_motion_mode.currentIndexChanged.connect(self._on_motion_mode_selected)
        selector_row.addWidget(self.combo_motion_mode)
        selector_row.addStretch()
        motion_layout.addLayout(selector_row)

        self.motion_stack = QStackedWidget()
        self.motion_stack.addWidget(self._build_tarea_panel())
        self.motion_stack.addWidget(self._build_cartesian_panel())
        motion_layout.addWidget(self.motion_stack)

        self.btn_goto = QPushButton("GOTO")
        self.btn_goto.setObjectName("btn_goto")
        self.btn_goto.clicked.connect(self.on_goto_clicked)
        motion_layout.addWidget(self.btn_goto)

        self.btn_tracking_demo = QPushButton("Tracking demo")
        self.btn_tracking_demo.clicked.connect(self.preset_tracking_demo)
        self.btn_tracking_demo.setVisible(False)   # solo visible en TRACK
        motion_layout.addWidget(self.btn_tracking_demo)

        layout.addWidget(gb_motion)

        # ---------- EVENT LOG ----------
        gb_log = QGroupBox("EVENT LOG")
        log_layout = QVBoxLayout(gb_log)
        self.log_list = QListWidget()
        log_layout.addWidget(self.log_list)
        layout.addWidget(gb_log, stretch=1)

        return col

    def _build_tarea_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        self.slider_alpha, self.spin_alpha = self._linked_control(
            -ALPHA_BETA_LIMIT_DEG, ALPHA_BETA_LIMIT_DEG, 0.5, 1, " °")
        self.slider_beta, self.spin_beta = self._linked_control(
            -ALPHA_BETA_LIMIT_DEG, ALPHA_BETA_LIMIT_DEG, 0.5, 1, " °")

        layout.addWidget(QLabel("α (deg)"))
        layout.addWidget(self.slider_alpha)
        layout.addWidget(self.spin_alpha)
        layout.addWidget(QLabel("β (deg)"))
        layout.addWidget(self.slider_beta)
        layout.addWidget(self.spin_beta)

        layout.addStretch()
        return panel

    def _build_cartesian_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        lin_step = CARTESIAN_STEP["linear"]
        ang_step = CARTESIAN_STEP["angular"]

        self.slider_x, self.spin_x = self._linked_control(
            -CARTESIAN_LIMITS["x"], CARTESIAN_LIMITS["x"], lin_step, 4, " m")
        self.slider_y, self.spin_y = self._linked_control(
            -CARTESIAN_LIMITS["y"], CARTESIAN_LIMITS["y"], lin_step, 4, " m")
        self.slider_z, self.spin_z = self._linked_control(
            -CARTESIAN_LIMITS["z"], CARTESIAN_LIMITS["z"], lin_step, 4, " m")
        self.slider_roll, self.spin_roll = self._linked_control(
            -CARTESIAN_LIMITS["roll"], CARTESIAN_LIMITS["roll"], ang_step, 2, " °")
        self.slider_pitch, self.spin_pitch = self._linked_control(
            -CARTESIAN_LIMITS["pitch"], CARTESIAN_LIMITS["pitch"], ang_step, 2, " °")
        self.slider_yaw, self.spin_yaw = self._linked_control(
            -CARTESIAN_LIMITS["yaw"], CARTESIAN_LIMITS["yaw"], ang_step, 2, " °")

        for label, slider, spin in [
            ("X", self.slider_x, self.spin_x), ("Y", self.slider_y, self.spin_y),
            ("Z", self.slider_z, self.spin_z), ("Roll", self.slider_roll, self.spin_roll),
            ("Pitch", self.slider_pitch, self.spin_pitch), ("Yaw", self.slider_yaw, self.spin_yaw),
        ]:
            layout.addWidget(QLabel(label))
            layout.addWidget(slider)
            layout.addWidget(spin)

        inc_row = QHBoxLayout()
        inc_row.addWidget(QLabel("Incremento (%):"))
        self.spin_increment = QDoubleSpinBox()
        self.spin_increment.setRange(1, 100)
        self.spin_increment.setValue(10)
        self.spin_increment.setSuffix(" %")
        self.spin_increment.valueChanged.connect(self._on_increment_changed)
        inc_row.addWidget(self.spin_increment)
        layout.addLayout(inc_row)

        layout.addStretch()
        return panel

    def _linked_control(self, minv, maxv, step, decimals, suffix=""):
        """Crea un QSlider + QDoubleSpinBox sincronizados."""
        factor = max(1, int(round(1.0 / step))) if step > 0 else 1000

        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(round(minv * factor)), int(round(maxv * factor)))
        slider.setValue(0)

        spin = QDoubleSpinBox()
        spin.setRange(minv, maxv)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        if suffix:
            spin.setSuffix(suffix)

        def slider_to_spin(v):
            spin.blockSignals(True)
            spin.setValue(v / factor)
            spin.blockSignals(False)

        def spin_to_slider(v):
            slider.blockSignals(True)
            slider.setValue(int(round(v * factor)))
            slider.blockSignals(False)

        slider.valueChanged.connect(slider_to_spin)
        spin.valueChanged.connect(spin_to_slider)
        return slider, spin

    def _on_increment_changed(self, pct):
        scale = pct / 10.0
        self.spin_x.setSingleStep(CARTESIAN_STEP["linear"] * scale)
        self.spin_y.setSingleStep(CARTESIAN_STEP["linear"] * scale)
        self.spin_z.setSingleStep(CARTESIAN_STEP["linear"] * scale)
        self.spin_roll.setSingleStep(CARTESIAN_STEP["angular"] * scale)
        self.spin_pitch.setSingleStep(CARTESIAN_STEP["angular"] * scale)
        self.spin_yaw.setSingleStep(CARTESIAN_STEP["angular"] * scale)

    # ------------------------------------------------------------------
    # Cinemática + Visualización + Serial
    # ------------------------------------------------------------------

    def send_heartbeat(self):
        if self.serial_manager.is_connected:
            self.serial_manager.send_raw("ping")

    def _maybe_send(self, percent_vector) -> None:
        """Envía el vector de 6 posiciones respetando un rate-limit >= 100 ms."""
        if self.serial_manager.is_connected and (self.t - self._last_send >= 0.10):
            self.serial_manager.send_positions([int(round(v)) for v in percent_vector])
            self._last_send = self.t

    def update_actuator_status(self, percent_array, oor_mask=None):
        for i in range(6):
            pct = float(percent_array[i])
            bar = self.axis_bars[i]
            bar.setValue(int(round(np.clip(pct, 0, 100))))
            self.axis_pct_labels[i].setText(f"{pct:.1f}%")

            if oor_mask is not None:
                saturated = bool(oor_mask[i])
            else:
                saturated = pct <= 0.5 or pct >= 99.5

            color = ACCENT_RED if saturated else ACCENT_GREEN_BRIGHT
            bar.setStyleSheet(
                f"QProgressBar {{ border: 1px solid #33475e; border-radius: 4px; "
                f"background-color: #12202f; }} "
                f"QProgressBar::chunk {{ background-color: {color}; border-radius: 3px; }}"
            )

    def compute_and_update(self, send_serial: bool = True):
        """
        Núcleo del modo TAREA (MANUAL o AUTO): calcula IK → actualiza gráficos → (opcional) manda a Arduino
        """
        try:
            q_actuator, da, R = inverse_kinematics(self.y_desired)
            self.q_actuator = q_actuator
            self.da = da
            self.R = R

            self.q_percent = length_to_percent(q_actuator)
            self.last_percent = self.q_percent

            self._update_pose_labels_tarea()

            self.platform_canvas.update_platform(self.da, self.R)
            self.actuator_canvas.update_data(self.t, self.q_percent)
            self.update_actuator_status(self.q_percent)

            if send_serial:
                self._maybe_send(self.q_percent)

            self.update_status_bar()

        except Exception as e:
            self.log_event(f"ERROR IK: {e}")
            QMessageBox.warning(self, "Error de Cinemática", str(e))

    def _apply_cartesian_target(self, send_serial: bool):
        x, y, z = self.spin_x.value(), self.spin_y.value(), self.spin_z.value()
        roll_deg, pitch_deg, yaw_deg = self.spin_roll.value(), self.spin_pitch.value(), self.spin_yaw.value()

        try:
            q_actuator, da, R = pose_to_q(
                x, y, z, np.deg2rad(roll_deg), np.deg2rad(pitch_deg), np.deg2rad(yaw_deg)
            )
        except Exception as e:
            self.log_event(f"ERROR IK cartesiano: {e}")
            QMessageBox.warning(self, "Error de Cinemática", str(e))
            return

        in_range = (q_actuator >= ACTUATOR_MIN - 1e-6) & (q_actuator <= ACTUATOR_MAX + 1e-6)
        percent = length_to_percent(q_actuator)

        self.q_actuator = q_actuator
        self.da = da
        self.R = R
        self.q_percent = percent
        self.last_percent = percent

        self._update_pose_labels_cartesian(x, y, z, roll_deg, pitch_deg, yaw_deg)
        self.platform_canvas.update_platform(da, R)
        self.actuator_canvas.update_data(self.t, percent)
        self.update_actuator_status(percent, oor_mask=~in_range)

        if in_range.all():
            self._clear_oor_state()
            if send_serial:
                self._maybe_send(percent)
        else:
            self._handle_out_of_range()

        self.update_status_bar()

    def _handle_out_of_range(self):
        if not self._oor_active:
            self._oor_active = True
            self.log_event("Pose fuera de carrera: comando serial bloqueado")
            QMessageBox.warning(
                self, "Pose fuera de carrera",
                "La pose cartesiana produce longitudes de actuador fuera de "
                "[ACTUATOR_MIN, ACTUATOR_MAX]. No se envía por serial."
            )

    def _clear_oor_state(self):
        self._oor_active = False

    def _update_pose_labels_tarea(self):
        a_deg = np.rad2deg(self.y_desired[0])
        b_deg = np.rad2deg(self.y_desired[1])
        self.lbl_pose_alpha.setText(f"{a_deg:.1f} °")
        self.lbl_pose_beta.setText(f"{b_deg:.1f} °")

        self.lbl_track_alpha.setText(f"{a_deg:.1f} °")
        self.lbl_track_beta.setText(f"{b_deg:.1f} °")
        phase = getattr(self.traj_gen, "phase", "-") if hasattr(self, "traj_gen") else "-"
        self.lbl_track_phase.setText(str(phase).upper())

    def _update_pose_labels_cartesian(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        self.lbl_pose_x.setText(f"{x * 1000:.1f} mm")
        self.lbl_pose_y.setText(f"{y * 1000:.1f} mm")
        self.lbl_pose_z.setText(f"{z * 1000:.1f} mm")
        self.lbl_pose_roll.setText(f"{roll_deg:.1f} °")
        self.lbl_pose_pitch.setText(f"{pitch_deg:.1f} °")
        self.lbl_pose_yaw.setText(f"{yaw_deg:.1f} °")

    def _seed_home_display(self):
        """Muestra un estado inicial en home (50%) en vez de un panel vacío/0%."""
        home = np.full(6, ACTUATOR_HOME_PERCENT, dtype=float)
        self.q_percent = home
        self.last_percent = home
        self.actuator_canvas.update_data(0.0, home)
        self.update_actuator_status(home)
        self._update_pose_labels_tarea()
        self._update_pose_labels_cartesian(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def update_loop(self):
        """Se ejecuta cada tick del timer GUI (20-50 ms)."""
        self.t += self.dt

        if self.op_mode == "JOG":
            if self.axis_dialog.isVisible():
                self._manual_axis_tick()
            elif self.motion_mode == "TAREA":
                self._manual_tarea_tick()
            else:
                self._manual_cartesian_tick()
        else:
            self._auto_tick()

    def _auto_tick(self):
        """Modo AUTO (solo TAREA): sigue el TrajectoryGenerator y manda pos por serial."""
        if hasattr(self, "traj_gen"):
            y_des, yp_des = self.traj_gen.step(self.t)
            self.y_desired = y_des          # [α, β] en radianes

        self.compute_and_update(send_serial=True)

        if self.demo_duration > 0 and self.t >= self.demo_duration:
            self.stop()

    def _manual_tarea_tick(self):
        a_rad = np.deg2rad(self.spin_alpha.value())
        b_rad = np.deg2rad(self.spin_beta.value())
        self.y_desired = np.array([a_rad, b_rad])
        self.compute_and_update(send_serial=True)

    def _manual_cartesian_tick(self):
        self._apply_cartesian_target(send_serial=True)

    def _manual_axis_tick(self):
        """Modo MANUAL con el diálogo 'Jog por eje' abierto: jog directo por pata."""
        vector = self.axis_dialog.get_command_vector()
        self.last_percent = np.array(vector, dtype=float)
        self.actuator_canvas.update_data(self.t, vector)
        self.update_actuator_status(self.last_percent)
        self._maybe_send(vector)
        self.update_status_bar()

    # ------------------------------------------------------------------
    # Controles de UI
    # ------------------------------------------------------------------
    def go_home(self):
        self.spin_alpha.setValue(0.0)
        self.spin_beta.setValue(0.0)
        self.spin_x.setValue(0.0)
        self.spin_y.setValue(0.0)
        self.spin_z.setValue(0.0)
        self.spin_roll.setValue(0.0)
        self.spin_pitch.setValue(0.0)
        self.spin_yaw.setValue(0.0)
        self.y_desired[:] = 0.0
        self.axis_dialog.reset_all_home()
        self._clear_oor_state()
        self.compute_and_update(send_serial=False)
        if self.serial_manager.is_connected:
            self.serial_manager.send_raw("home")
        self.log_event("HOME solicitado")

    def open_serial_config(self):
        dialog = SerialConfigDialog(self.serial_manager, self)
        dialog.exec()

    def open_axis_dialog(self):
        self.axis_dialog.show()
        self.axis_dialog.raise_()
        self.axis_dialog.activateWindow()

    def on_serial_connected(self, connected: bool):
        if connected:
            self.heartbeat_timer.start()
            self.log_event(f"Serial conectado ({self.serial_manager.current_port})")
        else:
            self.heartbeat_timer.stop()
            self.log_event("Serial desconectado")
        self.update_status_bar()

    def on_serial_message(self, msg: str):
        self.log_event(f"Arduino → {msg}")

    def on_serial_error(self, error: str):
        self.log_event(f"ERROR serial: {error}")

    def _on_motion_mode_selected(self, index: int):
        self.set_motion_mode("TAREA" if index == 0 else "CARTESIAN")

    def set_motion_mode(self, mode: str):
        if mode == self.motion_mode:
            return
        # Detener el lazo al cambiar de modo para no mezclar referencias.
        self.stop()
        self.motion_mode = mode

        self.combo_motion_mode.blockSignals(True)
        self.combo_motion_mode.setCurrentIndex(0 if mode == "TAREA" else 1)
        self.combo_motion_mode.blockSignals(False)

        self.motion_stack.setCurrentIndex(0 if mode == "TAREA" else 1)
        self.pose_stack.setCurrentIndex(0 if mode == "TAREA" else 1)

        self.log_event(f"Modo cambiado a {mode}")
        self.update_status_bar()

    def _on_op_mode_selected(self, index: int):
        self.set_operation_mode("JOG" if index == 0 else "TRACK")

    def set_operation_mode(self, mode: str):
        if mode == self.op_mode:
            return
        # Detener el lazo al cambiar de modo para no mezclar referencias.
        self.stop()
        self.op_mode = mode

        self.combo_op_mode.blockSignals(True)
        self.combo_op_mode.setCurrentIndex(0 if mode == "JOG" else 1)
        self.combo_op_mode.blockSignals(False)

        is_jog = mode == "JOG"
        self.lbl_motion_selector.setVisible(is_jog)
        self.combo_motion_mode.setVisible(is_jog)
        self.btn_goto.setVisible(is_jog)
        self.btn_tracking_demo.setVisible(not is_jog)

        # TRACK solo usa TAREA (α, β); bloquea los sliders en vez de ocultarlos.
        self.motion_stack.setCurrentIndex(0 if (not is_jog or self.motion_mode == "TAREA") else 1)
        self.slider_alpha.setEnabled(is_jog)
        self.spin_alpha.setEnabled(is_jog)
        self.slider_beta.setEnabled(is_jog)
        self.spin_beta.setEnabled(is_jog)
        self.pose_stack.setCurrentIndex(2 if not is_jog else (0 if self.motion_mode == "TAREA" else 1))

        self.log_event(f"Modo de operación cambiado a {mode}")
        self.update_status_bar()

    def on_goto_clicked(self):
        if self.motion_mode == "TAREA":
            a_rad = np.deg2rad(self.spin_alpha.value())
            b_rad = np.deg2rad(self.spin_beta.value())
            self.y_desired = np.array([a_rad, b_rad])
            self.compute_and_update(send_serial=self.running and self.op_mode == "JOG")
            self.log_event(
                f"GOTO TAREA α={self.spin_alpha.value():.1f}° β={self.spin_beta.value():.1f}°"
            )
        else:
            self._apply_cartesian_target(send_serial=self.running and self.op_mode == "JOG")
            self.log_event("GOTO CARTESIANO aplicado")

    def preset_tracking_demo(self):
        self.set_operation_mode("TRACK")
        self.start()
        self.log_event("Preset 'tracking demo' iniciado")

    def start(self):
        if self.running:
            return
        self.running = True
        self.t = 0.0
        self._last_send = -1.0
        self._oor_active = False
        self.actuator_canvas.clear()
        self.timer.start()
        self.btn_start.setEnabled(False)
        self.log_event("INICIAR")
        self.update_status_bar()

    def stop(self):
        was_running = self.running
        self.running = False
        self.timer.stop()
        self.btn_start.setEnabled(True)
        if was_running:
            self.log_event("PARO")
        self.update_status_bar()

    def open_parameter_dialog(self):
        dialog = ParametersDialog(current_params=getattr(self, "current_params", None), parent=self)
        dialog.parameters_changed.connect(self.on_parameters_changed)
        dialog.exec()

    def on_parameters_changed(self, params: dict):
        self.current_params = params
        self.demo_duration = float(params.get("DEMO_DURATION", 60.0))
        self.actuator_canvas.set_time_window(self.demo_duration)

        cfg = TrajectoryConfig(
            t_home_end=params["T_HOME_END"],
            t_tracking_end=params["T_TRACKING_END"],
            dt=params["DT"],
            filter_wn=params["FILTER_WN"]
        )
        self.traj_gen = TrajectoryGenerator(cfg)
        traj_points_rad = [[np.deg2rad(float(a)), np.deg2rad(float(b))] for a, b in params["TRAJ_POINTS"]]
        self.traj_gen.set_tracking_points(traj_points_rad)
        self.log_event("Parámetros actualizados")

    def log_event(self, text: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_list.addItem(f"[{stamp}] {text}")
        self.log_list.scrollToBottom()
        if self.log_list.count() > 200:
            self.log_list.takeItem(0)

    def update_status_bar(self):
        if self.running:
            self.lbl_system_state.setText("● SYSTEM ACTIVE")
            self.lbl_system_state.setStyleSheet(f"color: {ACCENT_GREEN_BRIGHT}; font-weight: bold;")
        else:
            self.lbl_system_state.setText("● SYSTEM STOPPED")
            self.lbl_system_state.setStyleSheet(f"color: {ACCENT_RED_BRIGHT}; font-weight: bold;")

        if self.serial_manager.is_connected:
            port = self.serial_manager.current_port or "?"
            self.lbl_serial_state.setText(f"SERIAL: {port}")
            self.lbl_serial_state.setStyleSheet(f"color: {ACCENT_GREEN_BRIGHT}; font-weight: bold;")
        else:
            self.lbl_serial_state.setText("SERIAL: DESCONECTADO")
            self.lbl_serial_state.setStyleSheet(f"color: {ACCENT_RED_BRIGHT}; font-weight: bold;")

        self.lbl_op_mode_badge.setText(f"OP: {self.op_mode}")
        self.lbl_op_mode_badge.setStyleSheet(f"color: {TEXT_MAIN}; font-weight: bold;")
        self.lbl_motion_mode_badge.setText(f"MODO: {self.motion_mode}")
        self.lbl_motion_mode_badge.setStyleSheet(f"color: {TEXT_MAIN}; font-weight: bold;")

    def closeEvent(self, event):
        self.serial_manager.close()
        event.accept()