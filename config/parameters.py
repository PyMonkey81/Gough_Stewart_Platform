import numpy as np


# config/parameters.py

# Carrera real del Actuonix L16-100
STROKE = 100.0                    # 100 mm

# Longitud muerta ancla a ancla con el émbolo retraído (equivalente al 1.4895 m de MATLAB):
# junta B + cuerpo retraído + junta M. Se resta una sola vez, al pasar a porcentaje.
L0 = 249.4        # 188.65 mm

# Rango físico completo: HOME es 0% (todo retraído)
ACTUATOR_MIN = L0                 # 0% retraído
ACTUATOR_MAX = L0 + STROKE        # 100% = L0 + carrera
ACTUATOR_HOME_PERCENT = 0         # HOME = 0% (retraído, cenit)


# ====================== Geometría (escala prototipo, mm) ======================
# Az: puntos de anclaje en marco plataforma (origen = centro del agujero)
# Bz: puntos de anclaje en marco base (fijo)
# Reordenados para alinear con CAD: B1/P1 = Pin01 (eje 1)
Az = np.array([
    [ 46.95,  39.63, -44.33],     # fila 0 (nuevo): eje 1 (Pin01)
    [ 10.84,  60.48, -44.33],     # fila 1 (nuevo): eje 2
    [-57.79,  20.85, -44.33],     # fila 2 (nuevo): eje 3
    [-57.79, -20.85, -44.33],     # fila 3 (nuevo): eje 4
    [ 10.84, -60.48, -44.33],     # fila 4 (nuevo): eje 5
    [ 46.95, -39.63, -44.33],     # fila 5 (nuevo): eje 6
])  # mm

Bz = np.array([
    [ 87.79,  20.85, 44.32],      # fila 0 (nuevo): eje 1 (Pin01)
    [-25.84,  86.46, 44.32],      # fila 1 (nuevo): eje 2
    [-61.95,  65.61, 44.32],      # fila 2 (nuevo): eje 3
    [-61.95, -65.61, 44.32],      # fila 3 (nuevo): eje 4
    [-25.84, -86.46, 44.32],      # fila 4 (nuevo): eje 5
    [ 87.79, -20.85, 44.32],      # fila 5 (nuevo): eje 6
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