#connection/serial_manager.py
from PySide6.QtCore import QObject, Signal, QThread, QTimer, Slot
from PySide6.QtSerialPort import QSerialPortInfo
from connection.serial_worker import SerialWorker


class SerialManager(QObject):
    connected_changed = Signal(bool)
    message_received = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.thread = QThread()
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)

        # Conexiones de señales
        self.worker.connected.connect(self.connected_changed)
        self.worker.message_received.connect(self.message_received)
        self.worker.error.connect(self.error_occurred)
        self.worker.finished.connect(self.thread.quit)

        # Timer para leer datos entrantes (corre en el hilo principal, pero es liviano)
        self.read_timer = QTimer()
        self.read_timer.setInterval(20)  # 50 Hz
        self.read_timer.timeout.connect(self.worker.process_incoming)

        self.thread.start()
        self._is_connected = False
        self.connected_changed.connect(self._update_connected_state)

    def _update_connected_state(self, state: bool):
        self._is_connected = state
        if state:
            self.read_timer.start()
        else:
            self.read_timer.stop()

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def available_ports(self):
        ports = []
        for info in QSerialPortInfo.availablePorts():
            ports.append({
                "name": info.portName(),
                "description": info.description(),
                "manufacturer": info.manufacturer()
            })
        return ports

    def connect_port(self, port_name: str, baudrate: int = 115200):
        # Llamamos al slot del worker de forma thread-safe
        from PySide6.QtCore import QMetaObject, Qt, Q_ARG
        QMetaObject.invokeMethod(
            self.worker,
            "connect_port",
            Qt.QueuedConnection,
            Q_ARG(str, port_name),
            Q_ARG(int, baudrate)
        )

    def disconnect_port(self):
        from PySide6.QtCore import QMetaObject, Qt
        QMetaObject.invokeMethod(self.worker, "disconnect_port", Qt.QueuedConnection)

    def send_positions(self, positions: list):
        from PySide6.QtCore import QMetaObject, Qt, Q_ARG
        QMetaObject.invokeMethod(
            self.worker,
            "send_positions",
            Qt.QueuedConnection,
            Q_ARG(list, positions)
        )

    def send_raw(self, command: str):
        from PySide6.QtCore import QMetaObject, Qt, Q_ARG
        QMetaObject.invokeMethod(
            self.worker,
            "send_command",
            Qt.QueuedConnection,
            Q_ARG(str, command)
        )

    def close(self):
        self.read_timer.stop()
        self.worker.stop()
        self.thread.quit()
        self.thread.wait(1000)