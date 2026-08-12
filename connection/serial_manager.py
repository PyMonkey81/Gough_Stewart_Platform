from PySide6.QtCore import Q_ARG, QMetaObject, QObject, QThread, QTimer, Qt, Signal
from PySide6.QtSerialPort import QSerialPortInfo
from connection.serial_worker import SerialWorker


class SerialManager(QObject):
    connected_changed = Signal(bool)
    message_received = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)

        self.read_timer = QTimer(self)
        self.read_timer.setInterval(20)
        self.read_timer.timeout.connect(self.worker.process_incoming)

        self.worker.connected.connect(self.connected_changed)
        self.worker.message_received.connect(self.message_received)
        self.worker.error.connect(self.error_occurred)
        self.worker.finished.connect(self.thread.quit)
        self.connected_changed.connect(self._update_connected_state)

        self._is_connected = False
        self._current_port = None
        self.thread.start()

    def _update_connected_state(self, state: bool):
        self._is_connected = state
        if state:
            if not self.read_timer.isActive():
                self.read_timer.start()
        else:
            self.read_timer.stop()
            self._current_port = None

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def current_port(self) -> str | None:
        return self._current_port

    def preferred_port(self) -> str | None:
        ports = self.available_ports()
        port_names = [p["name"].upper() for p in ports]
        if "COM8" in port_names:
            return "COM8"
        if not ports:
            return None
        return ports[0]["name"]

    def available_ports(self):
        ports = []
        for info in sorted(
            QSerialPortInfo.availablePorts(),
            key=lambda port: (0 if port.portName().upper() == "COM8" else 1, port.portName().lower())
        ):
            ports.append({
                "name": info.portName(),
                "description": info.description() or "Puerto serial",
                "manufacturer": info.manufacturer() or "Desconocido"
            })
        return ports

    def connect_port(self, port_name: str | None = None, baudrate: int = 115200):
        port_name = port_name or self.preferred_port()
        if not port_name:
            return

        self._current_port = port_name
        QMetaObject.invokeMethod(
            self.worker,
            "connect_port",
            Qt.QueuedConnection,
            Q_ARG(str, port_name),
            Q_ARG(int, baudrate),
        )

    def disconnect_port(self):
        QMetaObject.invokeMethod(self.worker, "disconnect_port", Qt.QueuedConnection)

    def send_positions(self, positions: list):
        if len(positions) != 6:
            self.error_occurred.emit("Se requieren exactamente 6 posiciones")
            return

        cmd = "pos " + ",".join(str(int(round(p))) for p in positions)
        print(f"[SerialManager] Enviando comando: {cmd}")
        self.send_raw(cmd)

    def send_raw(self, command: str):
        QMetaObject.invokeMethod(
            self.worker,
            "send_command",
            Qt.QueuedConnection,
            Q_ARG(str, command),
        )

    def close(self):
        self.read_timer.stop()
        if self.thread.isRunning():
            QMetaObject.invokeMethod(self.worker, "stop", Qt.QueuedConnection)
            self.thread.quit()
            self.thread.wait(1000)

        if self.worker is not None:
            self.worker.deleteLater()

        if self.thread is not None:
            self.thread.deleteLater()
