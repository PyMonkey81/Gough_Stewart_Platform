import numpy as np


# config/parameters.py

# Longitud de referencia (home) del modelo
OFFSET_ACTUADOR = 1.4895          # valor actual que tienes

# Carrera real del Actuonix LP16
STROKE = 0.100                    # 100 mm = 0.1 m

# Rango físico completo
ACTUATOR_MIN = OFFSET_ACTUADOR - (STROKE / 2)   # ≈ 1.4395 m
ACTUATOR_MAX = OFFSET_ACTUADOR + (STROKE / 2)   # ≈ 1.5395 m

ACTUATOR_HOME_PERCENT = 100


# ====================== Geometría (escala 1:1 actual) ======================
Az = np.array([
    [ 0.19087453,  0.23866804, -0.35236387],
    [ 0.11125532,  0.28463621, -0.35236387],
    [-0.30212985,  0.04596817, -0.35236387],
    [-0.30212985, -0.04596817, -0.35236387],
    [ 0.11125532, -0.28463621, -0.35236387],
    [ 0.19087453, -0.23866804, -0.35236387]
])

Bz = np.array([
    [ 0.62321826,  0.09836858,  0.35695029],
    [-0.22641944,  0.58890714,  0.35695029],
    [-0.39679882,  0.49053856,  0.35695029],
    [-0.39679882, -0.49053856,  0.35695029],
    [-0.22641944, -0.58890714,  0.35695029],
    [ 0.62321826, -0.09836858,  0.35695029]
])

# ====================== Parámetros de la plataforma ======================
ALPHA_0 = 0.0
D = np.array([0.0, 0.0, 2.1])
RT = 0.9                    # ← pon el valor real de rt
OFFSET_ACTUADOR = 1.4895    # medido de SolidWorks (cambiará en el prototipo)

# ====================== Controlador ======================
ALPHA_PI = 150.0
KP_S = 15000.0
KI_S = 300.0

# ====================== Trayectoria / Tiempos ======================
T_HOME_END = 60.0
T_TRACKING_END = 653.0
DT = 0.001
FILTER_WN = 3.4