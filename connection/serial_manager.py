import sys
from pathlib import Path

from PySide6.QtCore import Q_ARG, QMetaObject, QObject, QThread, QTimer, Qt, Signal
from PySide6.QtSerialPort import QSerialPortInfo
from connection.serial_worker import SerialWorker


_LINUX_SKIP = ("ttyS", "ttyprintk")
_LINUX_PREF = ("ttyACM", "ttyUSB", "ttyAMA")


def _canonical_port_name(info: QSerialPortInfo) -> str:
    location = info.systemLocation() or ""
    name = info.portName() or ""
    if sys.platform.startswith("linux"):
        if location.startswith("/dev/"):
            return location
        if name and not name.startswith("/dev/"):
            return f"/dev/{name}"
    return name or location


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
        if not ports:
            return None
        return ports[0]["name"]

    def available_ports(self):
        ports = []
        for info in QSerialPortInfo.availablePorts():
            name = _canonical_port_name(info)
            if not name:
                continue

            short = Path(name).name
            if sys.platform.startswith("linux") and short.startswith(_LINUX_SKIP):
                continue

            ports.append({
                "name": name,
                "short": short,
                "description": info.description() or "Puerto serial",
                "manufacturer": info.manufacturer() or "Desconocido"
            })

        def rank(port):
            short = port["short"]
            for index, prefix in enumerate(_LINUX_PREF):
                if short.startswith(prefix):
                    return (0, index, short)
            return (1, 99, short)

        ports.sort(key=rank)
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
