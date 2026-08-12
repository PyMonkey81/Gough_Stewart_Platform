# gui/screen/MainWindow.py
import sys
from tkinter import dialog
import numpy as np
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QDialog, QFormLayout,
    QDoubleSpinBox, QDialogButtonBox, QMessageBox,
    QGroupBox, QSizePolicy, QGridLayout
)
from PySide6.QtCore import Qt, QTimer

from gui.graphics.platform import PlatformCanvas
from gui.graphics.actuator import ActuatorCanvas
from gui.dialog.serialconfigdialog import SerialConfigDialog
from gui.dialog.parameterdiag import ParametersDialog, load_parameters
from connection.serial_manager import SerialManager

from kinematics.inverse import inverse_kinematics
from config.parameters import (
    D, ACTUATOR_MIN, ACTUATOR_MAX, ACTUATOR_HOME_PERCENT
)
from trajectory.generator import TrajectoryConfig, TrajectoryGenerator


def length_to_percent(q: np.ndarray) -> np.ndarray:
    """
    Versión temporal de depuración.
    Ajusta el rango según lo que realmente esté saliendo de la cinemática.
    """
    # Imprime una vez para ver el rango real
    print(f"[DEBUG] q min={q.min():.4f}, max={q.max():.4f}, mean={q.mean():.4f}")
    
    # Temporal: usa el rango real de los valores actuales + un margen
    q_min = q.min() - 0.01
    q_max = q.max() + 0.01
    
    if q_max - q_min < 1e-6:
        return np.full(6, 50.0)
    
    percent = (q - q_min) / (q_max - q_min) * 100.0
    return np.clip(percent, 0, 100)


class PoseDialog(QDialog):
    def __init__(self, parent=None, current_a_rad=0.0, current_b_rad=0.0):
        super().__init__(parent)
        self.setWindowTitle("Pose Deseada (Orientación)")
        self.setMinimumWidth(340)

        layout = QFormLayout(self)

        # Convertimos de rad → deg para mostrar
        self.spin_a = QDoubleSpinBox()
        self.spin_a.setRange(-90.0, 90.0)
        self.spin_a.setSingleStep(1.0)
        self.spin_a.setDecimals(1)
        self.spin_a.setValue(np.rad2deg(current_a_rad))
        self.spin_a.setSuffix(" °")

        self.spin_b = QDoubleSpinBox()
        self.spin_b.setRange(-90.0, 90.0)
        self.spin_b.setSingleStep(1.0)
        self.spin_b.setDecimals(1)
        self.spin_b.setValue(np.rad2deg(current_b_rad))
        self.spin_b.setSuffix(" °")

        layout.addRow("Ángulo α:", self.spin_a)
        layout.addRow("Ángulo β:", self.spin_b)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def get_pose(self):
        # Convertimos de grados → radianes antes de devolver
        a_rad = np.deg2rad(self.spin_a.value())
        b_rad = np.deg2rad(self.spin_b.value())
        return np.array([a_rad, b_rad])


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stewart Platform - Control Interface")
        self.setMinimumSize(1150, 780)

        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #121212; color: #e0e0e0; }
            QPushButton {
                background-color: #2d2d2d; color: #e0e0e0;
                border: 1px solid #444; border-radius: 6px;
                padding: 10px 18px; font-size: 13px; font-weight: 500;
            }
            QPushButton:hover { background-color: #3a3a3a; border-color: #666; }
            QPushButton:pressed { background-color: #222; }
            QPushButton#btn_start { background-color: #1b5e20; border-color: #2e7d32; }
            QPushButton#btn_start:hover { background-color: #2e7d32; }
            QPushButton#btn_stop { background-color: #b71c1c; border-color: #c62828; }
            QPushButton#btn_stop:hover { background-color: #c62828; }
            QPushButton#btn_mode { background-color: #0d47a1; border-color: #1565c0; }
            QPushButton#btn_mode:hover { background-color: #1565c0; }
            QLabel#status { color: #aaa; font-size: 12px; }
            QGroupBox {
                border: 1px solid #444;
                border-radius: 6px;
                margin-top: 12px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        # ------------------ Estado ------------------
        self.running = False
        self.manual_mode = True
        self.t = 0.0

        # Pose deseada actual (2 parámetros que usa tu TIK)
        self.y_desired = np.array([0.0, 0.0])   # [a, b]

        # Resultados del último cálculo de cinemática
        self.q_actuator = np.zeros(6)
        self.q_percent = np.full(6, ACTUATOR_HOME_PERCENT, dtype=float)
        self.da = D.copy()
        self.R = np.eye(3)

        self.current_params = load_parameters()

        # Serial
        self.serial_manager = SerialManager(self)
        self.serial_manager.connected_changed.connect(self.on_serial_connected)
        self.serial_manager.message_received.connect(self.on_serial_message)
        self.serial_manager.error_occurred.connect(self.on_serial_error)

        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.setInterval(1000)
        self.heartbeat_timer.timeout.connect(self.send_heartbeat)

        self.setup_ui()
        self.on_parameters_changed(self.current_params)
        self.compute_and_update(send_serial=False)

    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # ---------- Botones superiores ----------
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.btn_pose = QPushButton("Pose Deseada")
        self.btn_pose.clicked.connect(self.open_pose_dialog)

        self.btn_home = QPushButton("Home (50%)")
        self.btn_home.clicked.connect(self.go_home)

        self.btn_serial = QPushButton("Comunicación")
        self.btn_serial.clicked.connect(self.open_serial_config)

        self.btn_mode = QPushButton("Modo: MANUAL")
        self.btn_mode.setObjectName("btn_mode")
        self.btn_mode.clicked.connect(self.toggle_mode)

        self.btn_params = QPushButton("Parámetros")
        self.btn_params.setObjectName("btn_params")
        self.btn_params.clicked.connect(self.open_parameter_dialog)

        self.btn_start = QPushButton("Iniciar")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.clicked.connect(self.start)

        self.btn_stop = QPushButton("Paro")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.clicked.connect(self.stop)

        btn_layout.addWidget(self.btn_pose)
        btn_layout.addWidget(self.btn_home)
        btn_layout.addWidget(self.btn_serial)
        btn_layout.addWidget(self.btn_mode)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_params)
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)

        main_layout.addLayout(btn_layout)

        # ---------- Status ----------
        self.status_label = QLabel("Estado: Detenido  |  Modo: Manual  |  Serial: Desconectado")
        self.status_label.setObjectName("status")
        main_layout.addWidget(self.status_label)

        # ---------- Info de cinemática ----------
        info_group = QGroupBox("Cinemática Inversa (último cálculo)")
        info_layout = QGridLayout(info_group)

        self.lbl_pose = QLabel("y = [0.000, 0.000]")
        self.lbl_q = QLabel("q = [—, —, —, —, —, —]")
        self.lbl_percent = QLabel("% = [50, 50, 50, 50, 50, 50]")

        info_layout.addWidget(QLabel("Pose deseada:"), 0, 0)
        info_layout.addWidget(self.lbl_pose, 0, 1)
        info_layout.addWidget(QLabel("Longitudes q:"), 1, 0)
        info_layout.addWidget(self.lbl_q, 1, 1)
        info_layout.addWidget(QLabel("Porcentaje Arduino:"), 2, 0)
        info_layout.addWidget(self.lbl_percent, 2, 1)

        main_layout.addWidget(info_group)

        # ---------- Gráficos ----------
        self.platform_canvas = PlatformCanvas()
        self.actuator_canvas = ActuatorCanvas()
        main_layout.addWidget(self.platform_canvas, stretch=3)
        main_layout.addWidget(self.actuator_canvas, stretch=2)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.setInterval(20)   # 50 Hz (suficiente para demo)

    # ------------------------------------------------------------------
    # Cinemática + Visualización + Serial
    # ------------------------------------------------------------------

    def send_heartbeat(self):
        if self.serial_manager.is_connected:
            self.serial_manager.send_raw("ping")

    def compute_and_update(self, send_serial: bool = True):
        """
        Núcleo: calcula IK → actualiza gráficos → (opcional) manda a Arduino
        """
        try:
            q_actuator, da, R = inverse_kinematics(self.y_desired)
            self.q_actuator = q_actuator
            self.da = da
            self.R = R

            # Mapeo a porcentaje
            self.q_percent = length_to_percent(q_actuator)

            # Actualizar labels
            self.lbl_pose.setText(f"y = [{np.rad2deg(self.y_desired[0]):.1f}°, {np.rad2deg(self.y_desired[1]):.1f}°]"
)
            self.lbl_q.setText(
                "q = [" + ", ".join(f"{v:.4f}" for v in self.q_actuator) + "]"
            )
            self.lbl_percent.setText(
                "% = [" + ", ".join(f"{v:.1f}" for v in self.q_percent) + "]"
            )

            # Visualización 3D
            self.platform_canvas.update_platform(self.da, self.R)
            self.actuator_canvas.update_data(self.t, self.q_percent)

            # Enviar a Arduino
            # solo manda serial cada cierto tiempo
            if not hasattr(self, "_last_send"):
                self._last_send = 0.0

            now = self.t
            if send_serial and self.serial_manager.is_connected and (now - self._last_send > 0.10):
                self.serial_manager.send_positions(self.q_percent.tolist())
                # print(f"[MainWindow] Enviando posiciones a Arduino: {self.q_percent.tolist()}")
                self._last_send = now

        except Exception as e:
            print(f"[ERROR IK] {e}")
            QMessageBox.warning(self, "Error de Cinemática", str(e))

    def update_loop(self):
        """Se ejecuta cada 50 ms cuando está en modo AUTO o animación."""
        self.t += 0.05   # o el dt que estés usando en la GUI

        if not self.manual_mode and hasattr(self, "traj_gen"):
            y_des, yp_des = self.traj_gen.step(self.t)
            self.y_desired = y_des          # [α, β] en radianes

        self.compute_and_update(send_serial=True)

    # ------------------------------------------------------------------
    # Controles de UI
    # ------------------------------------------------------------------
    def open_pose_dialog(self):
        dialog = PoseDialog(self, self.y_desired[0], self.y_desired[1])
        if dialog.exec() == QDialog.Accepted:
            self.y_desired = dialog.get_pose()
            self.compute_and_update(send_serial=True)

    def go_home(self):
        self.y_desired[:] = 0.0
        self.compute_and_update(send_serial=True)
        if self.serial_manager.is_connected:
            self.serial_manager.send_raw("home")

    def open_serial_config(self):
        dialog = SerialConfigDialog(self.serial_manager, self)
        dialog.exec()

    def on_serial_connected(self, connected: bool):
        mode = "Manual" if self.manual_mode else "Auto"
        serial_txt = "Conectado" if connected else "Desconectado"
        state = "Ejecutando" if self.running else "Detenido"
        self.status_label.setText(f"Estado: {state}  |  Modo: {mode}  |  Serial: {serial_txt}")
        if connected:
            self.heartbeat_timer.start()
            # Opcional: mandar home o un pos inicial
            self.serial_manager.send_raw("home")
        else:
            self.heartbeat_timer.stop()

    def on_serial_message(self, msg: str):
        print(f"Arduino → {msg}")

    def on_serial_error(self, error: str):
        print(f"Error serial: {error}")

    def toggle_mode(self):
        self.manual_mode = not self.manual_mode
        self.btn_mode.setText("Modo: MANUAL" if self.manual_mode else "Modo: AUTO")
        self.on_serial_connected(self.serial_manager.is_connected)

    def start(self):
        if not self.running:
            self.running = True
            self.timer.start()
            self.btn_start.setEnabled(False)
            self.on_serial_connected(self.serial_manager.is_connected)

    def stop(self):
        self.running = False
        self.timer.stop()
        self.btn_start.setEnabled(True)
        self.on_serial_connected(self.serial_manager.is_connected)

    def open_parameter_dialog(self):
        dialog = ParametersDialog(current_params=getattr(self, "current_params", None), parent=self)
        dialog.parameters_changed.connect(self.on_parameters_changed)
        dialog.exec()

    def open_parameters(self):
        self.open_parameter_dialog()

    def on_parameters_changed(self, params: dict):
        self.current_params = params
        total_duration = float(params.get("T_TRACKING_END", 653.0)) + float(params.get("T_HOME_END", 60.0))
        self.actuator_canvas.set_time_window(total_duration)

        cfg = TrajectoryConfig(
            t_home_end=params["T_HOME_END"],
            t_tracking_end=params["T_TRACKING_END"],
            dt=params["DT"],
            filter_wn=params["FILTER_WN"]
        )
        self.traj_gen = TrajectoryGenerator(cfg)
        traj_points_rad = [[np.deg2rad(float(a)), np.deg2rad(float(b))] for a, b in params["TRAJ_POINTS"]]
        self.traj_gen.set_tracking_points(traj_points_rad)


    def closeEvent(self, event):
        self.serial_manager.close()
        event.accept()