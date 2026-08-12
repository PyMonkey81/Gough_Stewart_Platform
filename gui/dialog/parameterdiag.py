import json
from copy import deepcopy
from pathlib import Path

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
    QDoubleSpinBox, QLabel, QPushButton, QScrollArea, QWidget,
    QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView,
    QMessageBox, QAbstractItemView
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
import numpy as np

SETTINGS_PATH = Path(__file__).resolve().parents[2] / "config" / "parameters.json"

DEFAULT_PARAMETERS = {
    "OFFSET_ACTUADOR": 1.4895,
    "STROKE": 0.100,
    "ACTUATOR_HOME_PERCENT": 50.0,
    "ALPHA_0": 0.0,
    "D": [0.0, 0.0, 2.1],
    "RT": 0.9,
    "ALPHA_PI": 150.0,
    "KP_S": 15000.0,
    "KI_S": 300.0,
    "T_HOME_END": 60.0,
    "T_TRACKING_END": 653.0,
    "DT": 0.001,
    "FILTER_WN": 3.4,
    "TRAJ_POINTS": [
        [0.0, 0.0],
        [15.0, 10.0],
        [0.0, 0.0]
    ],
    "Az": [
        [0.19087453, 0.23866804, -0.35236387],
        [0.11125532, 0.28463621, -0.35236387],
        [-0.30212985, 0.04596817, -0.35236387],
        [-0.30212985, -0.04596817, -0.35236387],
        [0.11125532, -0.28463621, -0.35236387],
        [0.19087453, -0.23866804, -0.35236387]
    ],
    "Bz": [
        [0.62321826, 0.09836858, 0.35695029],
        [-0.22641944, 0.58890714, 0.35695029],
        [-0.39679882, 0.49053856, 0.35695029],
        [-0.39679882, -0.49053856, 0.35695029],
        [-0.22641944, -0.58890714, 0.35695029],
        [0.62321826, -0.09836858, 0.35695029]
    ]
}


def load_parameters() -> dict:
    if SETTINGS_PATH.exists():
        try:
            with open(SETTINGS_PATH, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            merged = deepcopy(DEFAULT_PARAMETERS)
            if isinstance(data, dict):
                merged.update(data)
            return merged
        except Exception:
            pass
    return deepcopy(DEFAULT_PARAMETERS)


def save_parameters(params: dict):
    settings = deepcopy(DEFAULT_PARAMETERS)
    if isinstance(params, dict):
        settings.update(params)

    SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(SETTINGS_PATH, "w", encoding="utf-8") as fh:
        json.dump(settings, fh, indent=2, ensure_ascii=False)
    return settings


class ParametersDialog(QDialog):
    """
    Diálogo de configuración de parámetros del Stewart Platform.
    Incluye geometría, actuadores, controlador y puntos de trayectoria.
    """
    parameters_changed = Signal(dict)   # Emite el diccionario completo al aceptar

    def __init__(self, current_params: dict = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Parámetros del Sistema")
        self.setMinimumSize(780, 620)
        self.resize(820, 680)

        self.setStyleSheet("""
            QDialog { background-color: #1e1e1e; color: #e0e0e0; }
            QLabel { color: #e0e0e0; }
            QGroupBox {
                color: #aaa; font-weight: bold;
                border: 1px solid #444; border-radius: 6px;
                margin-top: 14px; padding-top: 12px;
            }
            QGroupBox::title {
                subcontrol-origin: margin; left: 12px; padding: 0 6px;
            }
            QDoubleSpinBox, QSpinBox {
                background-color: #2d2d2d; color: #e0e0e0;
                border: 1px solid #555; border-radius: 4px;
                padding: 5px 8px; min-width: 110px;
            }
            QDoubleSpinBox:focus { border-color: #4fc3f7; }
            QPushButton {
                background-color: #2d2d2d; color: #e0e0e0;
                border: 1px solid #555; border-radius: 6px;
                padding: 8px 18px;
            }
            QPushButton:hover { background-color: #3a3a3a; }
            QPushButton#btn_ok { background-color: #1b5e20; border-color: #2e7d32; }
            QPushButton#btn_ok:hover { background-color: #2e7d32; }
            QTableWidget {
                background-color: #252525; color: #e0e0e0;
                gridline-color: #444; border: 1px solid #444;
            }
            QHeaderView::section {
                background-color: #333; color: #ccc;
                padding: 6px; border: 1px solid #444;
            }
            QTabWidget::pane { border: 1px solid #444; background: #1e1e1e; }
            QTabBar::tab {
                background: #2d2d2d; color: #ccc;
                padding: 8px 18px; margin-right: 2px;
                border-top-left-radius: 4px; border-top-right-radius: 4px;
            }
            QTabBar::tab:selected { background: #3a3a3a; color: #fff; }
        """)

        self.defaults = load_parameters()

        if current_params:
            self.defaults.update(current_params)

        self._build_ui()
        self._load_values()

    def _build_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(14, 14, 14, 14)

        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        # -------------------- Pestaña 1: Actuadores & Geometría --------------------
        tab1 = QWidget()
        tab1_layout = QVBoxLayout(tab1)
        scroll1 = QScrollArea()
        scroll1.setWidgetResizable(True)
        scroll1.setWidget(tab1)
        tabs.addTab(scroll1, "Actuadores y Geometría")

        # --- Actuadores ---
        gb_act = QGroupBox("Actuadores (L16)")
        form_act = QFormLayout(gb_act)

        self.sp_offset = self._spin(1.0, 2.0, 4, 0.0001)
        self.sp_stroke = self._spin(0.01, 0.30, 3, 0.001)
        self.sp_home_pct = self._spin(0, 100, 1, 1)

        form_act.addRow("OFFSET_ACTUADOR (m):", self.sp_offset)
        form_act.addRow("STROKE (m):", self.sp_stroke)
        form_act.addRow("HOME %:", self.sp_home_pct)

        self.lbl_min_max = QLabel("")
        self.lbl_min_max.setStyleSheet("color: #81c784; font-size: 12px;")
        form_act.addRow("Rango calculado:", self.lbl_min_max)

        self.sp_offset.valueChanged.connect(self._update_min_max)
        self.sp_stroke.valueChanged.connect(self._update_min_max)

        tab1_layout.addWidget(gb_act)

        # --- Geometría plataforma ---
        gb_geo = QGroupBox("Geometría de la plataforma")
        form_geo = QFormLayout(gb_geo)

        self.sp_alpha0 = self._spin(-1.0, 1.0, 4, 0.001)
        self.sp_rt = self._spin(0.1, 2.0, 3, 0.01)

        self.sp_dx = self._spin(-1.0, 1.0, 4, 0.001)
        self.sp_dy = self._spin(-1.0, 1.0, 4, 0.001)
        self.sp_dz = self._spin(0.5, 4.0, 3, 0.01)

        form_geo.addRow("ALPHA_0 (rad):", self.sp_alpha0)
        form_geo.addRow("RT:", self.sp_rt)
        form_geo.addRow("D.x:", self.sp_dx)
        form_geo.addRow("D.y:", self.sp_dy)
        form_geo.addRow("D.z:", self.sp_dz)

        tab1_layout.addWidget(gb_geo)

        # --- Matrices Az / Bz (solo visualización por ahora) ---
        gb_mat = QGroupBox("Matrices Az y Bz (puntos de anclaje)")
        mat_layout = QVBoxLayout(gb_mat)

        info = QLabel("Estas matrices vienen de la medición / SolidWorks.\n"
                      "Por ahora son de solo lectura. Más adelante se podrán editar.")
        info.setStyleSheet("color: #aaa; font-size: 12px;")
        mat_layout.addWidget(info)

        self.table_az = self._create_matrix_table("Az (base)")
        self.table_bz = self._create_matrix_table("Bz (plataforma)")

        mat_layout.addWidget(QLabel("Az – Puntos base:"))
        mat_layout.addWidget(self.table_az)
        mat_layout.addWidget(QLabel("Bz – Puntos plataforma:"))
        mat_layout.addWidget(self.table_bz)

        tab1_layout.addWidget(gb_mat)
        tab1_layout.addStretch()

        # -------------------- Pestaña 2: Controlador --------------------
        tab2 = QWidget()
        tab2_layout = QVBoxLayout(tab2)
        tabs.addTab(tab2, "Controlador")

        gb_ctrl = QGroupBox("Parámetros del controlador")
        form_ctrl = QFormLayout(gb_ctrl)

        self.sp_alpha_pi = self._spin(1, 500, 1, 1)
        self.sp_kp = self._spin(100, 50000, 1, 100)
        self.sp_ki = self._spin(1, 2000, 1, 10)

        form_ctrl.addRow("ALPHA_PI:", self.sp_alpha_pi)
        form_ctrl.addRow("KP_S:", self.sp_kp)
        form_ctrl.addRow("KI_S:", self.sp_ki)

        tab2_layout.addWidget(gb_ctrl)
        tab2_layout.addStretch()

        # -------------------- Pestaña 3: Trayectoria --------------------
        tab3 = QWidget()
        tab3_layout = QVBoxLayout(tab3)
        tabs.addTab(tab3, "Trayectoria")

        # Tiempos
        gb_time = QGroupBox("Tiempos de la trayectoria")
        form_time = QFormLayout(gb_time)

        self.sp_t_home = self._spin(1, 300, 1, 1)
        self.sp_t_track = self._spin(10, 2000, 1, 1)
        self.sp_dt = self._spin(0.0001, 0.01, 4, 0.0001)
        self.sp_filter = self._spin(0.1, 20.0, 2, 0.1)

        form_time.addRow("T_HOME_END (s):", self.sp_t_home)
        form_time.addRow("T_TRACKING_END (s):", self.sp_t_track)
        form_time.addRow("DT (s):", self.sp_dt)
        form_time.addRow("FILTER_WN:", self.sp_filter)

        tab3_layout.addWidget(gb_time)

        # Puntos del spline (3 pares α, β) en grados
        gb_traj = QGroupBox("Puntos de trayectoria (α, β) en grados – Spline cúbico")
        traj_layout = QVBoxLayout(gb_traj)

        note = QLabel("Define 3 puntos (α, β) en grados que se usarán para generar la trayectoria\n"
                      "mediante el spline cúbico. Los valores se convierten internamente a radianes.")
        note.setStyleSheet("color: #aaa; font-size: 12px;")
        traj_layout.addWidget(note)

        self.table_traj = QTableWidget(3, 2)
        self.table_traj.setHorizontalHeaderLabels(["Alpha (°)", "Beta (°)"])
        self.table_traj.verticalHeader().setVisible(True)
        self.table_traj.setVerticalHeaderLabels(["Punto 1", "Punto 2", "Punto 3"])
        self.table_traj.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_traj.setMaximumHeight(140)

        # Hacer las celdas editables con spinboxes en grados
        for row in range(3):
            for col in range(2):
                spin = QDoubleSpinBox()
                spin.setRange(-180.0, 180.0)
                spin.setDecimals(2)
                spin.setSingleStep(0.5)
                spin.setStyleSheet("background-color: #2d2d2d; color: #e0e0e0; border: none;")
                self.table_traj.setCellWidget(row, col, spin)

        traj_layout.addWidget(self.table_traj)
        tab3_layout.addWidget(gb_traj)
        tab3_layout.addStretch()

        # -------------------- Botones finales --------------------
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()

        btn_reset = QPushButton("Restaurar defaults")
        btn_reset.clicked.connect(self._load_values)

        btn_cancel = QPushButton("Cancelar")
        btn_cancel.clicked.connect(self.reject)

        btn_ok = QPushButton("Aplicar y Guardar")
        btn_ok.setObjectName("btn_ok")
        btn_ok.clicked.connect(self._on_accept)

        btn_layout.addWidget(btn_reset)
        btn_layout.addWidget(btn_cancel)
        btn_layout.addWidget(btn_ok)
        main_layout.addLayout(btn_layout)

    def _spin(self, minv, maxv, decimals, step):
        sp = QDoubleSpinBox()
        sp.setRange(minv, maxv)
        sp.setDecimals(decimals)
        sp.setSingleStep(step)
        return sp

    def _create_matrix_table(self, title: str):
        table = QTableWidget(6, 3)
        table.setHorizontalHeaderLabels(["X", "Y", "Z"])
        table.verticalHeader().setVisible(True)
        table.setVerticalHeaderLabels([f"P{i+1}" for i in range(6)])
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)  # solo lectura
        table.setMaximumHeight(200)
        return table

    def _update_min_max(self):
        offset = self.sp_offset.value()
        stroke = self.sp_stroke.value()
        amin = offset - stroke / 2
        amax = offset + stroke / 2
        self.lbl_min_max.setText(f"MIN = {amin:.4f} m    |    MAX = {amax:.4f} m")

    def _load_values(self):
        p = self.defaults

        self.sp_offset.setValue(p["OFFSET_ACTUADOR"])
        self.sp_stroke.setValue(p["STROKE"])
        self.sp_home_pct.setValue(p["ACTUATOR_HOME_PERCENT"])
        self._update_min_max()

        self.sp_alpha0.setValue(p["ALPHA_0"])
        self.sp_rt.setValue(p["RT"])
        self.sp_dx.setValue(p["D"][0])
        self.sp_dy.setValue(p["D"][1])
        self.sp_dz.setValue(p["D"][2])

        self.sp_alpha_pi.setValue(p["ALPHA_PI"])
        self.sp_kp.setValue(p["KP_S"])
        self.sp_ki.setValue(p["KI_S"])

        self.sp_t_home.setValue(p["T_HOME_END"])
        self.sp_t_track.setValue(p["T_TRACKING_END"])
        self.sp_dt.setValue(p["DT"])
        self.sp_filter.setValue(p["FILTER_WN"])

        # Matrices
        for i, row in enumerate(p["Az"]):
            for j, val in enumerate(row):
                self.table_az.setItem(i, j, QTableWidgetItem(f"{val:.6f}"))
        for i, row in enumerate(p["Bz"]):
            for j, val in enumerate(row):
                self.table_bz.setItem(i, j, QTableWidgetItem(f"{val:.6f}"))

        # Puntos de trayectoria
        for i, (a, b) in enumerate(p["TRAJ_POINTS"]):
            self.table_traj.cellWidget(i, 0).setValue(a)
            self.table_traj.cellWidget(i, 1).setValue(b)

    def get_parameters(self) -> dict:
        """Devuelve todos los parámetros actuales del diálogo"""
        traj = []
        for i in range(3):
            alpha = self.table_traj.cellWidget(i, 0).value()
            beta = self.table_traj.cellWidget(i, 1).value()
            traj.append([alpha, beta])

        return {
            "OFFSET_ACTUADOR": self.sp_offset.value(),
            "STROKE": self.sp_stroke.value(),
            "ACTUATOR_HOME_PERCENT": self.sp_home_pct.value(),
            "ACTUATOR_MIN": self.sp_offset.value() - self.sp_stroke.value() / 2,
            "ACTUATOR_MAX": self.sp_offset.value() + self.sp_stroke.value() / 2,
            "ALPHA_0": self.sp_alpha0.value(),
            "RT": self.sp_rt.value(),
            "D": [self.sp_dx.value(), self.sp_dy.value(), self.sp_dz.value()],
            "ALPHA_PI": self.sp_alpha_pi.value(),
            "KP_S": self.sp_kp.value(),
            "KI_S": self.sp_ki.value(),
            "T_HOME_END": self.sp_t_home.value(),
            "T_TRACKING_END": self.sp_t_track.value(),
            "DT": self.sp_dt.value(),
            "FILTER_WN": self.sp_filter.value(),
            "TRAJ_POINTS": traj,
            "Az": self.defaults["Az"],   # por ahora no editables
            "Bz": self.defaults["Bz"]
        }

    def _on_accept(self):
        params = self.get_parameters()
        save_parameters(params)

        self.parameters_changed.emit(params)
        self.accept()

