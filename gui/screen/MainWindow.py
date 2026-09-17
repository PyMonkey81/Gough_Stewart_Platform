# gui/screen/MainWindow.py
import time
import os
import numpy as np
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QMessageBox, QGroupBox, QComboBox,
    QStackedWidget, QSlider, QDoubleSpinBox,
    QProgressBar, QListWidget, QFrame, QSplitter
)
from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QSizePolicy

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
    ALPHA_BETA_LIMIT_DEG, CARTESIAN_LIMITS, CARTESIAN_STEP,
    APPROACH_S, RETURN_S
)
from trajectory.generator import TrajectoryConfig, TrajectoryGenerator

BG_MAIN = "#0E1116"
BG_CARD = "#161B22"
TEXT_MAIN = "#E0E6ED"
ACCENT_OK = "#3DDC97"
ACCENT_OK_BRIGHT = "#4FE5A8"
ACCENT_WARN = "#F5C542"
ACCENT_WARN_BRIGHT = "#F8D96C"
ACCENT_FAULT = "#FF5C5C"
ACCENT_FAULT_BRIGHT = "#FF7777"
AXIS_COLORS = ["#ff6b6b", "#4ecdc4", "#45b7d1", "#96ceb4", "#ffeaa7", "#dfe6e9"]


def length_to_percent(q: np.ndarray) -> np.ndarray:
    """Mapea longitudes de actuador (m) a porcentaje de carrera 0-100%."""
    percent = (q - ACTUATOR_MIN) / (ACTUATOR_MAX - ACTUATOR_MIN) * 100.0
    return np.clip(percent, 0, 100)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Gough-Stewart HMI")
        self.setMinimumSize(1400, 900)
        self.resize(1800, 1000)

        # Load QSS stylesheet
        qss_path = os.path.join(os.path.dirname(__file__), '..', 'theme', 'app.qss')
        if os.path.exists(qss_path):
            with open(qss_path, 'r') as f:
                self.setStyleSheet(f.read())
        
        # Fallback inline stylesheet if QSS not found
        else:
            self.setStyleSheet(f"""
                QMainWindow, QWidget {{ background-color: #0E1116; color: #E0E6ED; }}
                QPushButton {{ background-color: #25333F; color: #E0E6ED; border: 1px solid #404D5C; border-radius: 6px; padding: 8px 14px; }}
                QPushButton:hover {{ background-color: #323D4D; border-color: #505B6A; }}
                QGroupBox {{ background-color: #161B22; border: 2px solid #3A444D; border-radius: 8px; margin-top: 18px; }}
                QComboBox, QDoubleSpinBox, QSpinBox {{ background-color: #0E1116; color: #E0E6ED; border: 1px solid #404D5C; border-radius: 4px; padding: 6px 8px; }}
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
        
        # AUTO sequence state machine (IDLE → APPROACH → TRACK → RETURN → HOME)
        self.auto_state = "idle"        # Current AUTO state
        self.auto_t_start = 0.0         # Timestamp of state entry
        self.auto_q_first = None        # q_percent at first waypoint
        self.auto_q_last = None         # q_percent at last waypoint

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
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Professional 48px header
        root.addLayout(self._build_header(), stretch=0)
        
        # Main content: 3 columns
        columns = QHBoxLayout()
        columns.setContentsMargins(12, 10, 12, 10)
        columns.setSpacing(12)
        columns.addWidget(self._build_left_column(), stretch=1)
        columns.addWidget(self._build_center_column(), stretch=1)
        columns.addWidget(self._build_right_column(), stretch=1)
        root.addLayout(columns, stretch=1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.setInterval(int(self.dt * 1000))   # 30 ms (~33 Hz)

    def _build_header(self):
        """Build professional 48px SCADA header with all key controls."""
        header = QHBoxLayout()
        header.setContentsMargins(14, 6, 14, 6)
        header.setSpacing(16)

        # Title
        title = QLabel("GOUGH–STEWART HMI")
        title.setObjectName("headerTitle")
        title_font = QFont("Segoe UI", 16, QFont.Bold)
        title.setFont(title_font)
        header.addWidget(title)

        # Status indicator (LED)
        self.led_status = QLabel("●")
        self.led_status.setObjectName("statusIndicator")
        self.led_status.setStyleSheet("color: #666; font-size: 18px;")
        header.addWidget(self.led_status, alignment=Qt.AlignCenter)

        # Serial info (port, baud)
        self.lbl_port = QLabel("PORT: —")
        self.lbl_port.setObjectName("portLabel")
        header.addWidget(self.lbl_port, alignment=Qt.AlignCenter)

        self.lbl_baud = QLabel("BAUD: —")
        self.lbl_baud.setObjectName("baudLabel")
        header.addWidget(self.lbl_baud, alignment=Qt.AlignCenter)

        header.addStretch()

        # INICIAR button (large, green)
        self.btn_start = QPushButton("● INICIAR")
        self.btn_start.setObjectName("btnStart")
        self.btn_start.setMinimumWidth(140)
        self.btn_start.setMinimumHeight(36)
        self.btn_start.setStyleSheet(
            "QPushButton#btnStart { background-color: #3DDC97; color: #000; font-weight: bold; font-size: 12px; border: 2px solid #4FE5A8; border-radius: 6px; }"
            "QPushButton#btnStart:hover { background-color: #4FE5A8; }"
            "QPushButton#btnStart:pressed { background-color: #2AC878; }"
        )
        self.btn_start.clicked.connect(self.start)
        header.addWidget(self.btn_start)

        # PARO button (large, red)
        self.btn_stop = QPushButton("● PARO")
        self.btn_stop.setObjectName("btnStop")
        self.btn_stop.setMinimumWidth(140)
        self.btn_stop.setMinimumHeight(36)
        self.btn_stop.setStyleSheet(
            "QPushButton#btnStop { background-color: #FF5C5C; color: #FFF; font-weight: bold; font-size: 12px; border: 2px solid #FF7777; border-radius: 6px; }"
            "QPushButton#btnStop:hover { background-color: #FF7777; }"
            "QPushButton#btnStop:pressed { background-color: #E04040; }"
        )
        self.btn_stop.clicked.connect(self.stop)
        header.addWidget(self.btn_stop)

        # MANUAL / AUTO selector
        self.combo_auto_manual = QComboBox()
        self.combo_auto_manual.addItems(["MANUAL", "AUTO"])
        self.combo_auto_manual.setMinimumWidth(100)
        self.combo_auto_manual.currentIndexChanged.connect(self._on_auto_manual_changed)
        header.addWidget(self.combo_auto_manual)

        return header

    def _on_auto_manual_changed(self, index):
        """Handle MANUAL/AUTO mode selection."""
        mode = "AUTO" if index == 1 else "MANUAL"
        # AUTO only valid in TAREA mode
        if mode == "AUTO" and self.motion_mode != "TAREA":
            self.combo_auto_manual.blockSignals(True)
            self.combo_auto_manual.setCurrentIndex(0)
            self.combo_auto_manual.blockSignals(False)
            self.log_event(f"AUTO solo disponible en modo TAREA")
        else:
            self.log_event(f"Modo: {mode}")

    def _build_top_bar(self):
        """Legacy method - now integrated into header."""
        return QHBoxLayout()
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
        """Legacy method - no longer used."""
        return QHBoxLayout()

    def _build_left_column(self):
        col = QWidget()
        layout = QVBoxLayout(col)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # 3D Visualization (minimum 420x420)
        gb_platform = QGroupBox("HEXÁPODO 3D")
        gb_platform_layout = QVBoxLayout(gb_platform)
        gb_platform_layout.setContentsMargins(6, 6, 6, 6)
        self.platform_canvas = PlatformCanvas()
        self.platform_canvas.setMinimumSize(420, 420)
        gb_platform_layout.addWidget(self.platform_canvas)
        layout.addWidget(gb_platform, stretch=3)

        # Axis legend
        legend_box = QGroupBox("EJES")
        legend_layout = QGridLayout(legend_box)
        legend_layout.setSpacing(6)
        legend_layout.setContentsMargins(8, 8, 8, 8)
        
        for i in range(6):
            lbl_axis = QLabel(f"L{i + 1}")
            lbl_axis.setStyleSheet(f"color: {AXIS_COLORS[i]}; font-weight: bold; font-size: 11px; padding: 4px 8px;")
            lbl_axis.setAlignment(Qt.AlignCenter)
            legend_layout.addWidget(lbl_axis, i // 3, i % 3)
        
        layout.addWidget(legend_box, stretch=0)
        
        # Actuator chart (time series)
        gb_actuators = QGroupBox("TRAYECTORIA")
        gb_actuators_layout = QVBoxLayout(gb_actuators)
        gb_actuators_layout.setContentsMargins(6, 6, 6, 6)
        self.actuator_canvas = ActuatorCanvas()
        gb_actuators_layout.addWidget(self.actuator_canvas)
        layout.addWidget(gb_actuators, stretch=2)
        
        return col

    def _build_center_column(self):
        col = QWidget()
        layout = QVBoxLayout(col)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # Typography definitions
        title_font = QFont("Segoe UI", 18, QFont.Bold)
        label_font = QFont("Segoe UI", 11, QFont.Normal)
        value_font = QFont("Segoe UI Mono", 20, QFont.Bold)

        # ---------- CURRENT POSE ----------
        gb_pose = QGroupBox("CURRENT POSE")
        gb_pose.setFont(title_font)
        pose_layout = QVBoxLayout(gb_pose)
        self.pose_stack = QStackedWidget()

        page_tarea = QWidget()
        grid_tarea = QGridLayout(page_tarea)
        grid_tarea.setSpacing(8)
        grid_tarea.setContentsMargins(4, 4, 4, 4)
        
        self.lbl_pose_alpha = QLabel("0.0 °")
        self.lbl_pose_alpha.setFont(value_font)
        self.lbl_pose_alpha.setStyleSheet(f"color: {ACCENT_OK}; font-weight: bold;")
        self.lbl_pose_beta_tilt = QLabel("0.0 °")
        self.lbl_pose_beta_tilt.setFont(value_font)
        self.lbl_pose_beta_tilt.setStyleSheet(f"color: {ACCENT_OK}; font-weight: bold;")
        self.lbl_pose_beta_elev = QLabel("0.0 °")
        self.lbl_pose_beta_elev.setFont(value_font)
        self.lbl_pose_beta_elev.setStyleSheet(f"color: {ACCENT_OK}; font-weight: bold;")
        
        lbl_alpha_label = QLabel("α target:")
        lbl_alpha_label.setFont(label_font)
        lbl_beta_tilt_label = QLabel("β̄ tilt:")
        lbl_beta_tilt_label.setFont(label_font)
        lbl_beta_elev_label = QLabel("elev. β:")
        lbl_beta_elev_label.setFont(label_font)
        
        grid_tarea.addWidget(lbl_alpha_label, 0, 0)
        grid_tarea.addWidget(self.lbl_pose_alpha, 0, 1)
        grid_tarea.addWidget(lbl_beta_tilt_label, 1, 0)
        grid_tarea.addWidget(self.lbl_pose_beta_tilt, 1, 1)
        grid_tarea.addWidget(lbl_beta_elev_label, 2, 0)
        grid_tarea.addWidget(self.lbl_pose_beta_elev, 2, 1)

        page_cart = QWidget()
        grid_cart = QGridLayout(page_cart)
        grid_cart.setSpacing(8)
        grid_cart.setContentsMargins(4, 4, 4, 4)
        
        self.lbl_pose_x = QLabel("0.0 mm")
        self.lbl_pose_x.setFont(value_font)
        self.lbl_pose_y = QLabel("0.0 mm")
        self.lbl_pose_y.setFont(value_font)
        self.lbl_pose_z = QLabel("0.0 mm")
        self.lbl_pose_z.setFont(value_font)
        self.lbl_pose_roll = QLabel("0.0 °")
        self.lbl_pose_roll.setFont(value_font)
        self.lbl_pose_pitch = QLabel("0.0 °")
        self.lbl_pose_pitch.setFont(value_font)
        self.lbl_pose_yaw = QLabel("0.0 °")
        self.lbl_pose_yaw.setFont(value_font)
        
        cart_labels = [
            ("X target:", self.lbl_pose_x), ("Y target:", self.lbl_pose_y),
            ("Z target:", self.lbl_pose_z), ("Roll target:", self.lbl_pose_roll),
            ("Pitch target:", self.lbl_pose_pitch), ("Yaw target:", self.lbl_pose_yaw),
        ]
        for i, (name, lbl) in enumerate(cart_labels):
            name_widget = QLabel(name)
            name_widget.setFont(label_font)
            grid_cart.addWidget(name_widget, i, 0)
            grid_cart.addWidget(lbl, i, 1)

        page_track = QWidget()
        grid_track = QGridLayout(page_track)
        grid_track.setSpacing(8)
        grid_track.setContentsMargins(4, 4, 4, 4)
        
        self.lbl_track_alpha = QLabel("0.0 °")
        self.lbl_track_alpha.setFont(value_font)
        self.lbl_track_alpha.setStyleSheet(f"color: {ACCENT_WARN}; font-weight: bold;")
        self.lbl_track_beta = QLabel("0.0 °")
        self.lbl_track_beta.setFont(value_font)
        self.lbl_track_beta.setStyleSheet(f"color: {ACCENT_WARN}; font-weight: bold;")
        self.lbl_track_phase = QLabel("-")
        self.lbl_track_phase.setFont(value_font)
        self.lbl_track_phase.setStyleSheet(f"color: {ACCENT_WARN}; font-weight: bold;")
        
        lbl_track_alpha_label = QLabel("α trayectoria:")
        lbl_track_alpha_label.setFont(label_font)
        lbl_track_beta_label = QLabel("β trayectoria:")
        lbl_track_beta_label.setFont(label_font)
        lbl_track_phase_label = QLabel("Fase:")
        lbl_track_phase_label.setFont(label_font)
        
        grid_track.addWidget(lbl_track_alpha_label, 0, 0)
        grid_track.addWidget(self.lbl_track_alpha, 0, 1)
        grid_track.addWidget(lbl_track_beta_label, 1, 0)
        grid_track.addWidget(self.lbl_track_beta, 1, 1)
        grid_track.addWidget(lbl_track_phase_label, 2, 0)
        grid_track.addWidget(self.lbl_track_phase, 2, 1)

        self.pose_stack.addWidget(page_tarea)
        self.pose_stack.addWidget(page_cart)
        self.pose_stack.addWidget(page_track)
        pose_layout.addWidget(self.pose_stack)
        layout.addWidget(gb_pose)

        # ---------- ACTUATOR STATUS ----------
        gb_status = QGroupBox("ACTUATOR STATUS")
        gb_status.setFont(title_font)
        status_layout = QGridLayout(gb_status)
        status_layout.setSpacing(10)
        status_layout.setContentsMargins(4, 10, 4, 4)
        self.axis_bars = []
        self.axis_pct_labels = []
        for i in range(6):
            lbl_name = QLabel(f"L{i + 1}")
            lbl_name.setFont(label_font)
            lbl_name.setStyleSheet(f"color: {AXIS_COLORS[i]}; font-weight: bold;")
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(0)  # HOME = 0%
            bar.setTextVisible(False)
            bar.setMinimumHeight(28)
            pct_label = QLabel("0.0%")  # HOME = 0% (retraído)
            pct_label.setFont(QFont("Segoe UI Mono", 12, QFont.Bold))
            pct_label.setMinimumWidth(60)
            pct_label.setAlignment(Qt.AlignCenter)

            status_layout.addWidget(lbl_name, i, 0, alignment=Qt.AlignCenter)
            status_layout.addWidget(bar, i, 1)
            status_layout.addWidget(pct_label, i, 2, alignment=Qt.AlignRight)

            self.axis_bars.append(bar)
            self.axis_pct_labels.append(pct_label)

        layout.addWidget(gb_status)
        layout.addStretch()
        return col

    def _build_right_column(self):
        col = QWidget()
        main_layout = QVBoxLayout(col)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        title_font = QFont("Segoe UI", 18, QFont.Bold)
        label_font = QFont("Segoe UI", 11, QFont.Normal)

        # ========== TOP WIDGET: MOTION CONTROL + UTILITY BUTTONS ==========
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(10)
        top_layout.addStretch(0)  # No expansion

        # ---------- MOTION CONTROL ----------
        gb_motion = QGroupBox("MOTION CONTROL")
        gb_motion.setFont(title_font)
        gb_motion.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        gb_motion.setMaximumHeight(340)  # Compact TAREA; will resize in CARTESIAN via sizeHint
        
        motion_layout = QVBoxLayout(gb_motion)
        motion_layout.setContentsMargins(8, 8, 8, 8)
        motion_layout.setSpacing(6)

        op_row = QHBoxLayout()
        lbl_op = QLabel("Operación:")
        lbl_op.setFont(label_font)
        op_row.addWidget(lbl_op)
        self.combo_op_mode = QComboBox()
        self.combo_op_mode.addItems(["JOG", "TRACK"])
        self.combo_op_mode.currentIndexChanged.connect(self._on_op_mode_selected)
        op_row.addWidget(self.combo_op_mode)
        op_row.addStretch()
        motion_layout.addLayout(op_row)

        selector_row = QHBoxLayout()
        self.lbl_motion_selector = QLabel("Modo:")
        self.lbl_motion_selector.setFont(label_font)
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

        # GOTO button
        self.btn_goto = QPushButton("GOTO")
        self.btn_goto.setObjectName("btn_goto")
        self.btn_goto.clicked.connect(self.on_goto_clicked)
        self.btn_goto.setMinimumHeight(36)
        motion_layout.addWidget(self.btn_goto)

        # Tracking demo button (hidden in JOG)
        self.btn_tracking_demo = QPushButton("Tracking demo")
        self.btn_tracking_demo.clicked.connect(self.preset_tracking_demo)
        self.btn_tracking_demo.setVisible(False)
        self.btn_tracking_demo.setMinimumHeight(36)
        motion_layout.addWidget(self.btn_tracking_demo)

        # Control buttons row: Jog por eje + HOME
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        self.btn_jog = QPushButton("Jog por eje")
        self.btn_jog.clicked.connect(self.open_axis_dialog)
        self.btn_jog.setMinimumHeight(32)
        btn_row.addWidget(self.btn_jog)
        
        self.btn_home = QPushButton("HOME")
        self.btn_home.clicked.connect(self.go_home)
        self.btn_home.setMinimumHeight(32)
        btn_row.addWidget(self.btn_home)
        motion_layout.addLayout(btn_row)

        top_layout.addWidget(gb_motion)

        # ---------- UTILITY BUTTONS ----------
        util_layout = QHBoxLayout()
        util_layout.setSpacing(8)
        self.btn_serial = QPushButton("Serial")
        self.btn_serial.setMaximumWidth(90)
        self.btn_serial.clicked.connect(self.open_serial_config)
        util_layout.addWidget(self.btn_serial)

        self.btn_params = QPushButton("Parámetros")
        self.btn_params.setMaximumWidth(110)
        self.btn_params.clicked.connect(self.open_parameter_dialog)
        util_layout.addWidget(self.btn_params)
        util_layout.addStretch()
        top_layout.addLayout(util_layout)

        # ========== BOTTOM WIDGET: EVENT LOG ==========
        gb_log = QGroupBox("EVENT LOG")
        gb_log.setFont(title_font)
        gb_log.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        log_layout = QVBoxLayout(gb_log)
        log_layout.setContentsMargins(8, 8, 8, 8)
        self.log_list = QListWidget()
        self.log_list.setMinimumHeight(260)
        self.log_list.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        log_layout.addWidget(self.log_list)

        # ========== SPLITTER ==========
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(top_widget)
        splitter.addWidget(gb_log)
        splitter.setCollapsible(0, False)
        splitter.setCollapsible(1, False)
        splitter.setSizes([320, 400])  # MOTION CONTROL fixed ~320, log gets rest
        main_layout.addWidget(splitter, stretch=1)

        return col

    def _build_tarea_panel(self):
        panel = QWidget()
        panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        
        label_font = QFont("Segoe UI", 11, QFont.Normal)

        self.slider_alpha, self.spin_alpha = self._linked_control(
            -ALPHA_BETA_LIMIT_DEG, ALPHA_BETA_LIMIT_DEG, 0.5, 1, " °")
        self.slider_beta, self.spin_beta = self._linked_control(
            -ALPHA_BETA_LIMIT_DEG, ALPHA_BETA_LIMIT_DEG, 0.5, 1, " °")

        lbl_alpha = QLabel("α (deg)")
        lbl_alpha.setFont(label_font)
        lbl_alpha.setMaximumHeight(22)
        layout.addWidget(lbl_alpha)
        layout.addWidget(self.slider_alpha)
        layout.addWidget(self.spin_alpha)
        
        lbl_beta = QLabel("β̄ tilt (°)")
        lbl_beta.setFont(label_font)
        lbl_beta.setMaximumHeight(22)
        lbl_beta.setToolTip(
            "0° apunta al cenit (artículo 2022).\n"
            "La elevación de estación es 90° − β̄.\n"
            "β̄ = 0°: mira al cenit | β̄ = 90°: mira al horizonte"
        )
        layout.addWidget(lbl_beta)
        layout.addWidget(self.slider_beta)
        layout.addWidget(self.spin_beta)

        return panel

    def _build_cartesian_panel(self):
        panel = QWidget()
        panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        
        label_font = QFont("Segoe UI", 11, QFont.Normal)

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

        for label_text, slider, spin in [
            ("X", self.slider_x, self.spin_x), ("Y", self.slider_y, self.spin_y),
            ("Z", self.slider_z, self.spin_z), ("Roll", self.slider_roll, self.spin_roll),
            ("Pitch", self.slider_pitch, self.spin_pitch), ("Yaw", self.slider_yaw, self.spin_yaw),
        ]:
            lbl = QLabel(label_text)
            lbl.setFont(label_font)
            lbl.setMaximumHeight(22)
            layout.addWidget(lbl)
            layout.addWidget(slider)
            layout.addWidget(spin)

        inc_row = QHBoxLayout()
        lbl_inc = QLabel("Incremento (%):")
        lbl_inc.setFont(label_font)
        lbl_inc.setMaximumHeight(22)
        inc_row.addWidget(lbl_inc)
        self.spin_increment = QDoubleSpinBox()
        self.spin_increment.setRange(1, 100)
        self.spin_increment.setValue(10)
        self.spin_increment.setSuffix(" %")
        self.spin_increment.valueChanged.connect(self._on_increment_changed)
        inc_row.addWidget(self.spin_increment)
        layout.addLayout(inc_row)

        return panel

    def _linked_control(self, minv, maxv, step, decimals, suffix=""):
        """Crea un QSlider + QDoubleSpinBox sincronizados."""
        factor = max(1, int(round(1.0 / step))) if step > 0 else 1000

        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(round(minv * factor)), int(round(maxv * factor)))
        slider.setValue(0)
        slider.setFixedHeight(22)
        slider.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

        spin = QDoubleSpinBox()
        spin.setRange(minv, maxv)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setFixedHeight(24)
        spin.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
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
        """Actualiza barras de estado de actuadores con colores por eje."""
        for i in range(6):
            pct = float(percent_array[i])
            bar = self.axis_bars[i]
            bar.setValue(int(round(np.clip(pct, 0, 100))))
            self.axis_pct_labels[i].setText(f"{pct:.1f}%")

            # Determine if actuator is in fault/warning state
            if oor_mask is not None:
                is_fault = bool(oor_mask[i])
            else:
                is_fault = pct <= 0.5 or pct >= 99.5

            # Use per-axis color, adjusted for fault/warning
            axis_color = AXIS_COLORS[i]
            if is_fault:
                # Red tint for fault, using accent fault color
                bar_color = ACCENT_FAULT
            else:
                # Use axis-specific color
                bar_color = axis_color

            # Enhanced stylesheet with proper SCADA styling
            bar.setStyleSheet(
                f"QProgressBar {{ "
                f"  border: 2px solid {AXIS_COLORS[i]}; "
                f"  border-radius: 4px; "
                f"  background-color: #0E1116; "
                f"  text-align: center; "
                f"}} "
                f"QProgressBar::chunk {{ "
                f"  background-color: {bar_color}; "
                f"  border-radius: 2px; "
                f"}}"
            )
            
            # Update label color to match axis
            self.axis_pct_labels[i].setStyleSheet(
                f"color: {axis_color}; font-weight: bold; font-family: 'Segoe UI Mono';"
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
        b_tilt_deg = np.rad2deg(self.y_desired[1])  # β̄ (tilt)
        b_elev_deg = 90.0 - b_tilt_deg               # β (elevación desde horizonte)
        self.lbl_pose_alpha.setText(f"{a_deg:.1f} °")
        self.lbl_pose_beta_tilt.setText(f"{b_tilt_deg:.1f} °")
        self.lbl_pose_beta_elev.setText(f"{b_elev_deg:.1f} °")

        self.lbl_track_alpha.setText(f"{a_deg:.1f} °")
        self.lbl_track_beta.setText(f"{b_tilt_deg:.1f} °")
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
        """Inicializa displays en HOME: 0% (retraído, cenit), sin IK."""
        home = np.zeros(6, dtype=float)  # 0% = ACTUATOR_MIN (retraído)
        self.q_percent = home
        self.last_percent = home
        self.actuator_canvas.update_data(0.0, home)
        self.update_actuator_status(home)
        self._update_pose_labels_tarea()  # muestra alpha=0, beta_tilt=0, elev.beta=90
        self._update_pose_labels_cartesian(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Dibujar 3D en HOME (sin IK)
        self.platform_canvas.update_platform_at_home()

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
        """AUTO sequence state machine: IDLE → APPROACH → TRACK → RETURN → HOME."""
        if not hasattr(self, "traj_gen"):
            return
        
        # ---- State transitions ----
        dt_elapsed = self.t - self.auto_t_start
        
        if self.auto_state == "idle":
            # Should not reach here in this tick; waiting for start
            pass
        
        elif self.auto_state == "approach":
            # Interpolate from q=[0]*6 to q_first over APPROACH_S seconds
            if dt_elapsed >= APPROACH_S:
                # Phase complete, transition to TRACK
                self.auto_state = "track"
                self.auto_t_start = self.t
                self.log_event("APPROACH → TRACK")
            else:
                # Linear interpolation of q_percent
                progress = dt_elapsed / APPROACH_S
                q_interp = (1 - progress) * np.zeros(6) + progress * self.auto_q_first
                self.q_percent = q_interp
                self.last_percent = q_interp
                self.actuator_canvas.update_data(self.t, q_interp)
                self.update_actuator_status(q_interp)
        
        elif self.auto_state == "track":
            # Follow TrajectoryGenerator (α, β̄) → IK → q_percent
            y_des, yp_des = self.traj_gen.step(self.t)
            self.y_desired = y_des
            
            # Track end time (first check if tracking is over)
            t_track_end = self.traj_gen.cfg.t_tracking_end
            if self.t >= t_track_end:
                # Phase complete, transition to RETURN
                self.auto_state = "return"
                self.auto_t_start = self.t
                self.log_event("TRACK → RETURN")
            else:
                # Normal tracking
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
                    self._maybe_send(self.q_percent)
                except Exception as e:
                    self.log_event(f"ERROR IK (TRACK): {e}")
                    self.stop()
        
        elif self.auto_state == "return":
            # Interpolate from q_last to q=[0]*6 over RETURN_S seconds
            if dt_elapsed >= RETURN_S:
                # Phase complete, transition to HOME
                self.auto_state = "home_final"
                self.auto_t_start = self.t
                self.log_event("RETURN → HOME")
            else:
                # Linear interpolation of q_percent
                progress = dt_elapsed / RETURN_S
                q_interp = (1 - progress) * self.auto_q_last + progress * np.zeros(6)
                self.q_percent = q_interp
                self.last_percent = q_interp
                self.actuator_canvas.update_data(self.t, q_interp)
                self.update_actuator_status(q_interp)
                self._maybe_send(q_interp)
        
        elif self.auto_state == "home_final":
            # Final HOME: send "home", set q_pct=0, PARO
            self.q_percent[:] = 0.0
            self.last_percent[:] = 0.0
            self.actuator_canvas.update_data(self.t, np.zeros(6))
            self.update_actuator_status(np.zeros(6))
            self.platform_canvas.update_platform_at_home()
            if self.serial_manager.is_connected:
                self.serial_manager.send_raw("home")
            ts = time.strftime("%H:%M:%S")
            self.log_event(f"[{ts}] AUTO complete: HOME")
            self.stop()  # PARO

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
        """HOME = ejes retraídos (0%), TAREA α=0° β̄=0° (cenit), sin IK."""
        # 1. Parar el lazo
        self.stop()
        
        # 2. Poner sliders en cero (ambos modos)
        self.spin_alpha.setValue(0.0)
        self.spin_beta.setValue(0.0)
        self.spin_x.setValue(0.0)
        self.spin_y.setValue(0.0)
        self.spin_z.setValue(0.0)
        self.spin_roll.setValue(0.0)
        self.spin_pitch.setValue(0.0)
        self.spin_yaw.setValue(0.0)
        self.y_desired[:] = 0.0
        
        # 3. Poner q_percent a 0% (retraído) directamente sin IK
        self.q_percent[:] = 0.0
        self.last_percent[:] = 0.0
        
        # 4. Actualizar displays (3D, barras, gráfica) - HOME SIN IK
        self.actuator_canvas.update_data(0.0, self.q_percent)
        self.update_actuator_status(self.q_percent)
        self._update_pose_labels_tarea()  # α=0°, β̄=0°, elev.β=90°
        self._update_pose_labels_cartesian(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.platform_canvas.update_platform_at_home()  # Dibuja 3D sin IK
        
        # 5. Enviar comando HOME por serial
        self.axis_dialog.reset_all_home()
        self._clear_oor_state()
        if self.serial_manager.is_connected:
            self.serial_manager.send_raw("home")
        
        # 6. Log con timestamp
        ts = time.strftime("%H:%M:%S")
        self.log_event(f"[{ts}] HOME — ejes retraídos, cenit")

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
        
        # Initialize AUTO state machine if in TRACK mode
        if self.op_mode == "TRACK" and hasattr(self, "traj_gen"):
            # Calculate q_percent for first and last waypoints
            try:
                # First waypoint: [y_start_a, y_start_b]
                y_first = np.array([self.traj_gen.y_start_a, self.traj_gen.y_start_b])
                q_first_actuator, _, _ = inverse_kinematics(y_first)
                self.auto_q_first = length_to_percent(q_first_actuator)
                
                # Last waypoint: evaluate spline at t_tracking_end
                t_end = self.traj_gen.cfg.t_tracking_end
                y_last_a = float(self.traj_gen.spline_a(t_end))
                y_last_b = float(self.traj_gen.spline_b(t_end))
                y_last = np.array([y_last_a, y_last_b])
                q_last_actuator, _, _ = inverse_kinematics(y_last)
                self.auto_q_last = length_to_percent(q_last_actuator)
                
                # Initialize state machine
                self.auto_state = "approach"
                self.auto_t_start = self.t
                self.log_event("APPROACH → TRACK → RETURN → HOME")
            except Exception as e:
                self.log_event(f"ERROR AUTO init: {e}")
                self.stop()
                return
        
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
        """Update header status indicators."""
        # LED status indicator
        if self.serial_manager.is_connected:
            self.led_status.setText("●")
            self.led_status.setStyleSheet("color: #3DDC97; font-size: 18px;")
        else:
            self.led_status.setText("●")
            self.led_status.setStyleSheet("color: #666; font-size: 18px;")

        # Port and baud rate
        if self.serial_manager.is_connected:
            port = self.serial_manager.current_port or "—"
            baud = getattr(self.serial_manager, 'baud_rate', 115200)
            self.lbl_port.setText(f"PORT: {port}")
            self.lbl_baud.setText(f"BAUD: {baud}")
        else:
            self.lbl_port.setText("PORT: —")
            self.lbl_baud.setText("BAUD: —")

    def closeEvent(self, event):
        self.serial_manager.close()
        event.accept()