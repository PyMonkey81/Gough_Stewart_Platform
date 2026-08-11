#connection/serial_worker.py

from PySide6.QtCore import QObject, Signal, Slot, QThread
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
import time

class SerialWorker(QObject):
    """Trabajador que corre dentro de un QThread"""
    
    connected = Signal(bool)
    message_received = Signal(str)
    error = Signal(str)
    finished = Signal()

    def __init__(self):
        super().__init__()
        self.serial = None
        self._running = True
        self.port_name = ""
        self.baudrate = 115200

    @Slot(str, int)
    def connect_port(self, port_name: str, baudrate: int = 115200):
        self.port_name = port_name
        self.baudrate = baudrate

        if self.serial and self.serial.isOpen():
            self.serial.close()

        self.serial = QSerialPort()
        self.serial.setPortName(port_name)
        self.serial.setBaudRate(baudrate)
        self.serial.setDataBits(QSerialPort.Data8)
        self.serial.setParity(QSerialPort.NoParity)
        self.serial.setStopBits(QSerialPort.OneStop)
        self.serial.setFlowControl(QSerialPort.NoFlowControl)

        if self.serial.open(QSerialPort.ReadWrite):
            # Esperar a que el Arduino se reinicie (muy común)
            time.sleep(1.8)
            self.serial.clear()
            self.connected.emit(True)
        else:
            self.error.emit(f"No se pudo abrir el puerto {port_name}")
            self.connected.emit(False)

    @Slot()
    def disconnect_port(self):
        if self.serial and self.serial.isOpen():
            self.serial.close()
        self.connected.emit(False)

    @Slot(str)
    def send_command(self, command: str):
        if self.serial and self.serial.isOpen():
            if not command.endswith("\n"):
                command += "\n"
            self.serial.write(command.encode("utf-8"))
        else:
            self.error.emit("Puerto no conectado")

    @Slot(list)
    def send_positions(self, positions: list):
        if len(positions) != 6:
            self.error.emit("Se requieren exactamente 6 posiciones")
            return
        cmd = "pos " + ",".join(str(int(p)) for p in positions)
        self.send_command(cmd)

    def process_incoming(self):
        """Llamar periódicamente desde el hilo"""
        if self.serial and self.serial.isOpen() and self.serial.canReadLine():
            while self.serial.canReadLine():
                data = self.serial.readLine().data().decode("utf-8", errors="ignore").strip()
                if data:
                    self.message_received.emit(data)

    @Slot()
    def stop(self):
        self._running = False
        self.disconnect_port()
        self.finished.emit()