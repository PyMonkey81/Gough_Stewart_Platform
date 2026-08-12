from PySide6.QtCore import Qt, Slot
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
)


class SerialConfigDialog(QDialog):
    def __init__(self, serial_manager, parent=None):
        super().__init__(parent)
        self.serial_manager = serial_manager

        self.setWindowTitle("Configuración de Comunicación Serial")
        self.setMinimumWidth(460)
        self.setModal(True)

        self.setStyleSheet("""
            QDialog {
                background-color: #1e1e1e;
                color: #e0e0e0;
            }
            QLabel {
                color: #e0e0e0;
            }
            QGroupBox {
                color: #aaaaaa;
                font-weight: bold;
                border: 1px solid #444;
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }
            QComboBox {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 7px 10px;
                min-height: 20px;
            }
            QComboBox:hover {
                border-color: #777;
            }
            QComboBox::drop-down {
                border: none;
            }
            QPushButton {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #555;
                border-radius: 6px;
                padding: 9px 18px;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
                border-color: #777;
            }
            QPushButton:disabled {
                background-color: #252525;
                color: #666;
                border-color: #333;
            }
            QPushButton#btn_connect {
                background-color: #1b5e20;
                border-color: #2e7d32;
            }
            QPushButton#btn_connect:hover {
                background-color: #2e7d32;
            }
            QPushButton#btn_disconnect {
                background-color: #b71c1c;
                border-color: #c62828;
            }
            QPushButton#btn_disconnect:hover {
                background-color: #c62828;
            }
        """)

        self._setup_ui()
        self._connect_signals()
        self.refresh_ports()
        self.update_ui_state()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(14)
        layout.setContentsMargins(18, 18, 18, 18)

        group = QGroupBox("Puerto Serial")
        form = QFormLayout(group)
        form.setSpacing(10)

        self.port_combo = QComboBox()
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.btn_refresh = QPushButton("Actualizar")
        self.btn_refresh.setFixedWidth(110)
        self.btn_refresh.clicked.connect(self.refresh_ports)

        port_row = QHBoxLayout()
        port_row.addWidget(self.port_combo)
        port_row.addWidget(self.btn_refresh)

        form.addRow("Seleccionar puerto:", port_row)
        layout.addWidget(group)

        self.status_label = QLabel("Estado: Desconectado")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("""
            font-size: 14px;
            font-weight: bold;
            padding: 10px;
            border-radius: 6px;
            background-color: #2a2a2a;
        """)
        layout.addWidget(self.status_label)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(12)

        self.btn_connect = QPushButton("Conectar")
        self.btn_connect.setObjectName("btn_connect")
        self.btn_connect.clicked.connect(self.on_connect)

        self.btn_disconnect = QPushButton("Desconectar")
        self.btn_disconnect.setObjectName("btn_disconnect")
        self.btn_disconnect.clicked.connect(self.on_disconnect)

        btn_layout.addWidget(self.btn_connect)
        btn_layout.addWidget(self.btn_disconnect)
        layout.addLayout(btn_layout)

        self.btn_close = QPushButton("Cerrar")
        self.btn_close.clicked.connect(self.accept)
        layout.addWidget(self.btn_close)

    def _connect_signals(self):
        self.serial_manager.connected_changed.connect(self.on_connection_changed)
        self.serial_manager.error_occurred.connect(self.on_error)

    @Slot()
    def on_connect(self):
        if self.serial_manager.is_connected:
            QMessageBox.information(
                self,
                "Ya conectado",
                f"Ya estás conectado a {self.serial_manager.current_port}",
            )
            return

        port_name = self.port_combo.currentData()
        if not port_name:
            QMessageBox.warning(self, "Puerto no válido", "Selecciona un puerto serial válido.")
            return

        self.btn_connect.setEnabled(False)
        self.btn_connect.setText("Conectando...")
        self.status_label.setText(f"Estado: Conectando a {port_name}...")
        self.status_label.setStyleSheet("""
            font-size: 14px; font-weight: bold; padding: 10px;
            border-radius: 6px; background-color: #2a2a2a; color: #ffcc00;
        """)

        self.serial_manager.connect_port(port_name, 115200)

    @Slot()
    def on_disconnect(self):
        self.serial_manager.disconnect_port()

    @Slot(bool)
    def on_connection_changed(self, connected: bool):
        self.update_ui_state()

        if connected:
            port = self.serial_manager.current_port or self.port_combo.currentData() or "desconocido"
            QMessageBox.information(self, "Conectado", f"Conectado correctamente a:\n{port}")

    @Slot(str)
    def on_error(self, message: str):
        self.update_ui_state()
        QMessageBox.critical(self, "Error de comunicación", message)

    def refresh_ports(self):
        self.port_combo.blockSignals(True)
        self.port_combo.clear()

        ports = self.serial_manager.available_ports()

        if not ports:
            self.port_combo.addItem("No se encontraron puertos")
            self.port_combo.blockSignals(False)
            self.update_ui_state()
            return

        for port in ports:
            display_text = f"{port['name']}  —  {port['description']}"
            self.port_combo.addItem(display_text, port['name'])

        target = self.serial_manager.current_port or "COM8"
        index = self.port_combo.findData(target)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)
        else:
            preferred_keywords = ["com8", "giga", "arduino", "usb serial", "com"]
            found = False
            for i in range(self.port_combo.count()):
                text = self.port_combo.itemText(i).lower()
                if any(keyword in text for keyword in preferred_keywords):
                    self.port_combo.setCurrentIndex(i)
                    found = True
                    break
            if not found:
                self.port_combo.setCurrentIndex(0)

        self.port_combo.blockSignals(False)
        self.update_ui_state()

    def update_ui_state(self):
        connected = self.serial_manager.is_connected
        port = self.serial_manager.current_port or "—"

        if connected:
            self.status_label.setText(f"Estado: ● Conectado a {port}")
            self.status_label.setStyleSheet("""
                font-size: 14px; font-weight: bold; padding: 10px;
                border-radius: 6px; background-color: #1b3d1b; color: #4caf50;
            """)
            self.btn_connect.setEnabled(False)
            self.btn_connect.setText("Conectar")
            self.btn_disconnect.setEnabled(True)
            self.port_combo.setEnabled(False)
            self.btn_refresh.setEnabled(False)
        else:
            self.status_label.setText("Estado: ● Desconectado")
            self.status_label.setStyleSheet("""
                font-size: 14px; font-weight: bold; padding: 10px;
                border-radius: 6px; background-color: #3d1b1b; color: #ff6b6b;
            """)
            has_valid_port = self.port_combo.count() > 0 and self.port_combo.currentData() is not None
            self.btn_connect.setEnabled(has_valid_port)
            self.btn_connect.setText("Conectar")
            self.btn_disconnect.setEnabled(False)
            self.port_combo.setEnabled(True)
            self.btn_refresh.setEnabled(True)

