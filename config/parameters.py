import numpy as np


# config/parameters.py

# Longitud de referencia (home) del prototipo L16-100
OFFSET_ACTUADOR = 247.41          # ||A-B|| medido en HOME (cenit, patas retraídas), mm

# Carrera real del Actuonix L16-100
STROKE = 100.0                    # 100 mm

# Rango físico completo: HOME es 0% (todo retraído)
ACTUATOR_MIN = 247.41             # 0% retraído
ACTUATOR_MAX = 347.41             # 100% = min + carrera
ACTUATOR_HOME_PERCENT = 0         # HOME = 0% (retraído, cenit)


# ====================== Geometría (escala prototipo, mm) ======================
# Az: puntos de anclaje en marco plataforma (origen = centro del agujero)
# Bz: puntos de anclaje en marco base (fijo)
Az = np.array([
    [ 46.95, -39.63, -44.33],
    [ 10.84, -60.48, -44.33],
    [-57.79, -20.85, -44.33],
    [-57.79,  20.85, -44.33],
    [ 10.84,  60.48, -44.33],
    [ 46.95,  39.63, -44.33],
])  # mm

Bz = np.array([
    [ 87.79, -20.85, 44.32],
    [-25.84, -86.46, 44.32],
    [-61.95, -65.61, 44.32],
    [-61.95,  65.61, 44.32],
    [-25.84,  86.46, 44.32],
    [ 87.79,  20.85, 44.32],
])  # mm

# ====================== Parámetros de la plataforma (en mm) ======================
ALPHA_0 = 0.0
D = np.array([0.0, 0.0, 331.94])  # Centro del agujero, mm
RT = 0.0                           # Prototipo: RT solo usada para antena (no aplica aquí)

# ====================== Controlador ======================
ALPHA_PI = 150.0
KP_S = 15000.0
KI_S = 300.0

# ====================== Trayectoria / Tiempos ======================
T_HOME_END = 60.0
T_TRACKING_END = 653.0
DT = 0.001
FILTER_WN = 3.4

# AUTO sequence timing: HOME → APPROACH → TRACK → RETURN → HOME
APPROACH_S = 3.0    # Tiempo para interpolar de q=0 al primer waypoint
RETURN_S = 3.0      # Tiempo para interpolar del último waypoint a q=0

# Duración de la demo GUI en modo AUTO (no confundir con T_TRACKING_END)
DEMO_DURATION = 60.0

# ====================== Modo CARTESIANO (MOTION CONTROL) ======================
# Límites de los sliders cartesianos, en mm y grados
ALPHA_BETA_LIMIT_DEG = 90.0

CARTESIAN_LIMITS = {
    "x": 30.0, "y": 30.0, "z": 40.0,            # mm (alrededor de 331.94 ± 40)
    "roll": 15.0, "pitch": 15.0, "yaw": 20.0,   # deg
}

CARTESIAN_STEP = {
    "linear": 1.0,     # mm
    "angular": 0.5,    # deg
}