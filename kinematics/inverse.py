import numpy as np
from config.parameters import Az, Bz, ALPHA_0, D, RT, OFFSET_ACTUADOR

def TIK(y, a0=ALPHA_0, D=D, rt=RT):
    """
    Task Inverse Kinematics (MATLAB)
    """
    a, b = y
    R = np.array([
        # Fila 1
        [np.sin(a)*np.sin(a - a0) + np.cos(a)*np.cos(a - a0)*np.sin(b),
         np.cos(a - a0)*np.sin(a) - np.cos(a)*np.sin(a - a0)*np.sin(b),
         np.cos(a)*np.cos(b)],

        # Fila 2
        [np.cos(a)*np.sin(a - a0) - np.cos(a - a0)*np.sin(a)*np.sin(b),
         np.cos(a - a0)*np.cos(a) + np.sin(a)*np.sin(a - a0)*np.sin(b),
         -np.cos(b)*np.sin(a)],

        # Fila 3
        [-np.cos(a - a0)*np.cos(b),
          np.cos(b)*np.sin(a - a0),
          np.sin(b)]
    ])

    da = D + rt * R[:, 2]
    return da, R

def PIK(da, R, Az=Az, Bz=Bz):
    """
    Platform Inverse Kinematics (Gough-Stewart)
    ------------------------------------------
    Az : puntos de anclaje en la plataforma móvil (6x3) o (3x6)
    Bz : puntos de anclaje en la base fija     (6x3) o (3x6)
    da : posición deseada del origen de la plataforma
    R  : matriz de rotación deseada 3x3
    
    Returns
    -------
    q : longitudes de las 6 piernas (6,)
    """
    q = np.zeros(6)
    for i in range(6):
        leg = da + R @ Az[i] - Bz[i]
        q[i] = np.linalg.norm(leg)
    return q

def inverse_kinematics(y):
    """Función principal: y → q_actuator"""
    da, R = TIK(y)
    q_geom = PIK(da, R)
    q_actuator = q_geom - OFFSET_ACTUADOR
    return q_actuator, da, R