#gui/screen/MainWindow.py
import sys
import numpy as np
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QDialog, QFormLayout,
    QDoubleSpinBox, QDialogButtonBox, QMessageBox,
    QComboBox, QGroupBox, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot, QThread, QObject, QMetaObject, Q_ARG




from gui.graphics.platform import PlatformCanvas
from gui.graphics.actuator import ActuatorCanvas
from gui.dialog.serialconfigdialog import SerialConfigDialog
from connection.serial_manager import SerialManager
from config.parameters import D



# ============================================================
# MAIN WINDOW
# ============================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stewart Platform - Control Interface")
        self.setMinimumSize(1100, 750)

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
        """)

        # Estado
        self.running = False
        self.manual_mode = True
        self.t = 0.0
        self.actuator_lengths = np.array([50.0] * 6)

        # Serial
        self.serial_manager = SerialManager(self)
        self.serial_manager.connected_changed.connect(self.on_serial_connected)
        self.serial_manager.message_received.connect(self.on_serial_message)
        self.serial_manager.error_occurred.connect(self.on_serial_error)

        self.setup_ui()
    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # Botones superiores
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.btn_params = QPushButton("Parámetros")
        self.btn_serial = QPushButton("Comunicación")
        self.btn_serial.clicked.connect(self.open_serial_config)

        self.btn_mode = QPushButton("Modo: MANUAL")
        self.btn_mode.setObjectName("btn_mode")
        self.btn_mode.clicked.connect(self.toggle_mode)

        self.btn_start = QPushButton("Iniciar")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.clicked.connect(self.start)

        self.btn_stop = QPushButton("Paro")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.clicked.connect(self.stop)

        # Botones superiores
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.btn_params = QPushButton("Parámetros")
        self.btn_serial = QPushButton("Comunicación")
        self.btn_serial.clicked.connect(self.open_serial_config)

        self.btn_mode = QPushButton("Modo: MANUAL")
        self.btn_mode.setObjectName("btn_mode")
        self.btn_mode.clicked.connect(self.toggle_mode)

        self.btn_start = QPushButton("Iniciar")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.clicked.connect(self.start)

        self.btn_stop = QPushButton("Paro")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.clicked.connect(self.stop)

        btn_layout.addWidget(self.btn_params)
        btn_layout.addWidget(self.btn_serial)
        btn_layout.addWidget(self.btn_mode)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)

        main_layout.addLayout(btn_layout)

        self.status_label = QLabel("Estado: Detenido  |  Modo: Manual  |  Serial: Desconectado")
        self.status_label.setObjectName("status")
        main_layout.addWidget(self.status_label)

        # Gráficos
        self.platform_canvas = PlatformCanvas()
        self.actuator_canvas = ActuatorCanvas()
        main_layout.addWidget(self.platform_canvas, stretch=3)
        main_layout.addWidget(self.actuator_canvas, stretch=2)

        # Timer simulación
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.setInterval(50)

    def open_serial_config(self):
        dialog = SerialConfigDialog(self.serial_manager, self)
        dialog.exec()

    def on_serial_connected(self, connected: bool):
        mode = "Manual" if self.manual_mode else "Auto"
        serial_txt = "Conectado" if connected else "Desconectado"
        state = "Ejecutando" if self.running else "Detenido"
        self.status_label.setText(f"Estado: {state}  |  Modo: {mode}  |  Serial: {serial_txt}")

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

    def update_simulation(self):
        self.t += 0.05
        for i in range(6):
            self.actuator_lengths[i] = 50 + 25 * np.sin(self.t * 1.2 + i * 1.05)

        self.actuator_canvas.update_data(self.t, self.actuator_lengths)

        yaw = 0.15 * np.sin(self.t * 0.8)
        pitch = 0.08 * np.sin(self.t * 0.5)
        roll = 0.06 * np.cos(self.t * 0.6)

        cz, sz = np.cos(yaw), np.sin(yaw)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cx, sx = np.cos(roll), np.sin(roll)

        rotation = np.array([
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ])

        translation = D + np.array([
            0.04 * np.sin(self.t * 0.4),
            0.03 * np.cos(self.t * 0.5),
            0.05 * np.sin(self.t * 0.6),
        ])

        self.platform_canvas.update_platform(translation, rotation)

        # Enviar posiciones al Arduino si está conectado
        if self.serial_manager.is_connected:
            self.serial_manager.send_positions(self.actuator_lengths.tolist())

    def closeEvent(self, event):
        self.serial_manager.close()
        event.accept()