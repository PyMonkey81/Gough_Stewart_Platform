# gui/dialog/axisconfigdialog.py
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGroupBox,
    QSlider, QSpinBox, QCheckBox, QPushButton
)

from config.parameters import ACTUATOR_HOME_PERCENT


class AxisConfigDialog(QDialog):
    """Diálogo no modal para jog manual de los 6 ejes (Modo MANUAL)."""

    axes_changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Ejes / Manual")
        self.setMinimumWidth(760)
        self.setModal(False)

        self.setStyleSheet("""
            QDialog { background-color: #1e1e1e; color: #e0e0e0; }
            QLabel { color: #e0e0e0; }
            QGroupBox {
                color: #aaa; font-weight: bold;
                border: 1px solid #444; border-radius: 6px;
                margin-top: 10px; padding-top: 10px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton {
                background-color: #2d2d2d; color: #e0e0e0;
                border: 1px solid #555; border-radius: 6px; padding: 8px 14px;
            }
            QPushButton:hover { background-color: #3a3a3a; }
            QSpinBox {
                background-color: #2d2d2d; color: #e0e0e0;
                border: 1px solid #555; border-radius: 4px; padding: 3px;
            }
        """)

        self.sliders = []
        self.spins = []
        self.checks = []

        layout = QVBoxLayout(self)

        axes_layout = QHBoxLayout()
        for i in range(6):
            axes_layout.addWidget(self._build_axis_group(i))
        layout.addLayout(axes_layout)

        btn_layout = QHBoxLayout()
        btn_all = QPushButton("Habilitar todos")
        btn_all.clicked.connect(self.enable_all)
        btn_none = QPushButton("Ninguno")
        btn_none.clicked.connect(self.disable_all)
        btn_home = QPushButton("Todos 50%")
        btn_home.clicked.connect(self.home_enabled)
        btn_layout.addWidget(btn_all)
        btn_layout.addWidget(btn_none)
        btn_layout.addWidget(btn_home)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def _build_axis_group(self, index: int) -> QGroupBox:
        gb = QGroupBox(f"Eje {index + 1}")
        v = QVBoxLayout(gb)

        slider = QSlider(Qt.Vertical)
        slider.setRange(0, 100)
        slider.setValue(int(ACTUATOR_HOME_PERCENT))
        slider.setMinimumHeight(140)

        spin = QSpinBox()
        spin.setRange(0, 100)
        spin.setValue(int(ACTUATOR_HOME_PERCENT))
        spin.setSuffix(" %")

        check = QCheckBox("Habilitar")
        check.setChecked(False)

        slider.valueChanged.connect(spin.setValue)
        spin.valueChanged.connect(slider.setValue)
        # Enviar al soltar el slider (no en cada cambio de valor)
        slider.sliderReleased.connect(lambda _=None: self.axes_changed.emit())
        # También enviar cuando cambies manualmente el spinbox
        spin.valueChanged.connect(lambda _=None: self.axes_changed.emit())
        check.toggled.connect(lambda enabled, i=index: self._on_toggle(i, enabled))

        v.addWidget(slider, alignment=Qt.AlignHCenter)
        v.addWidget(spin)
        v.addWidget(check)

        self.sliders.append(slider)
        self.spins.append(spin)
        self.checks.append(check)

        self._on_toggle(index, False)
        return gb

    def _on_toggle(self, index: int, enabled: bool):
        self.sliders[index].setEnabled(enabled)
        self.spins[index].setEnabled(enabled)
        self.axes_changed.emit()

    def enable_all(self):
        for c in self.checks:
            c.setChecked(True)

    def disable_all(self):
        for c in self.checks:
            c.setChecked(False)

    def home_enabled(self):
        """Pone 50% solo en los ejes actualmente habilitados."""
        for i, c in enumerate(self.checks):
            if c.isChecked():
                self.spins[i].setValue(50)

    def reset_all_home(self):
        """Fuerza los 6 ejes a home (50%), sin importar si están habilitados."""
        for spin in self.spins:
            spin.setValue(int(ACTUATOR_HOME_PERCENT))

    def get_command_vector(self):
        """Vector de 6 enteros a enviar. Los ejes deshabilitados conservan
        su último valor (o home, si nunca se habilitaron), nunca 0."""
        return [s.value() for s in self.spins]

    def get_enabled(self):
        return [c.isChecked() for c in self.checks]
