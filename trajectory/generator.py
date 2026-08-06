import numpy as np
from logic.spline import SimpleCubicSpline as CubicSpline

from dataclasses import dataclass
from typing import Callable, Optional

@dataclass
class TrajectoryConfig:
    t_home_end: float = 60.0          # Fin de la fase Home → Inicio de tracking
    t_tracking_end: float = 653.0     # Fin del tracking → Inicio de retorno a cero
    dt: float = 0.001                 # Tiempo de muestreo
    filter_wn: float = 3.4            # Frecuencia natural del filtro de 2º orden

class SecondOrderFilter:
    """Equivalente al Filter2 de Simulink"""
    def __init__(self, wn: float = 3.4, dt: float = 0.001):
        self.wn = wn
        self.dt = dt
        self.x1 = 0.0  # posición filtrada
        self.x2 = 0.0  # velocidad filtrada

    def reset(self, x1: float = 0.0, x2: float = 0.0):
        self.x1 = x1
        self.x2 = x2

    def step(self, ur: float) -> tuple[float, float]:
        error1 = ur - self.x1
        error2 = self.wn * error1 - 2.0 * self.x2
        x2_dot = self.wn * error2
        x1_dot = self.x2

        self.x2 += x2_dot * self.dt
        self.x1 += x1_dot * self.dt
        return self.x1, self.x2

class TrajectoryGenerator:
    """
    Generador de trayectoria completo (Task Space).
    Reemplaza: Home block + Spline + Switches + Filter2
    """
    def __init__(self, config: TrajectoryConfig = TrajectoryConfig()):
        self.cfg = config
        self.filter = SecondOrderFilter(wn=config.filter_wn, dt=config.dt)
        
        # --- Define aquí tu trayectoria de seguimiento ---
        # Ejemplo: puedes reemplazar esto por tus puntos reales
        self.tracking_times = None
        self.tracking_positions = None
        self.spline: Optional[CubicSpline] = None
        
        # Posición inicial de la fase de tracking (la que alcanzas en t=60)
        self.y_start_tracking = 0.0
        
        # Estado interno
        self.phase = "home"
        self.y_current = 0.0
        self.yp_current = 0.0

    def set_tracking_trajectory(self, times: np.ndarray, positions: np.ndarray):
        """
        Define la trayectoria de la fase de seguimiento (60s → 653s).
        
        times: vector de tiempos absolutos (debe empezar cerca de 60 y terminar cerca de 653)
        positions: valores de yd correspondientes
        """
        self.tracking_times = times
        self.tracking_positions = positions
        self.spline = CubicSpline(times, positions, bc_type='natural')
        
        # La posición a la que debe llegar el Home (en t=60)
        self.y_start_tracking = float(self.spline(self.cfg.t_home_end))

    def set_tracking_function(self, func: Callable[[float], float]):
        """
        Alternativa: pasar una función yd = f(t) para la fase de tracking.
        """
        self.spline = None
        self._tracking_func = func
        self.y_start_tracking = float(func(self.cfg.t_home_end))

    def _get_tracking_reference(self, t: float) -> tuple[float, float]:
        """Devuelve (yd, ypd) durante la fase de tracking"""
        if self.spline is not None:
            yd = float(self.spline(t))
            ypd = float(self.spline(t, 1))  # primera derivada
            return yd, ypd
        else:
            # Si usaste set_tracking_function
            yd = self._tracking_func(t)
            # Derivada numérica simple (mejor si tu función es analítica)
            eps = 1e-5
            ypd = (self._tracking_func(t + eps) - self._tracking_func(t - eps)) / (2 * eps)
            return yd, ypd

    def step(self, t: float) -> tuple[float, float]:
        """
        Genera la referencia en el instante t.
        
        Returns:
            yd  : posición/orientación deseada
            ypd : velocidad deseada
        """
        t_home = self.cfg.t_home_end
        t_end  = self.cfg.t_tracking_end

        # -------------------------------------------------
        # FASE 1: Home → Posición inicial de tracking
        # -------------------------------------------------
        if t < t_home:
            # Referencia cruda: rampa o escalón hacia y_start_tracking

            progress = t / t_home
            ur = self.y_start_tracking * progress          # rampa lineal simple
            # ur = self.y_start_tracking                    # escalón (el filtro se encarga)

            yd, ypd = self.filter.step(ur)
            self.phase = "home"

        # -------------------------------------------------
        # FASE 2: Tracking (spline)
        # -------------------------------------------------
        elif t < t_end:
            # Reiniciar el filtro cuando entramos a tracking (opcional)
            if self.phase != "tracking":
                self.filter.reset(x1=self.y_start_tracking, x2=0.0)
                self.phase = "tracking"

            yd, ypd = self._get_tracking_reference(t)

        # -------------------------------------------------
        # FASE 3: Retorno a cero
        # -------------------------------------------------
        else:
            if self.phase != "return":
                # Guardamos la posición actual para bajar suavemente
                self.filter.reset(x1=self.y_current, x2=self.yp_current)
                self.phase = "return"

            ur = 0.0   # queremos ir a cero
            yd, ypd = self.filter.step(ur)

        # Guardar estado actual
        self.y_current = yd
        self.yp_current = ypd

        return yd, ypd

    def generate(self, t_final: float = 800.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Genera toda la trayectoria de una vez (útil para plots o offline).
        """
        t = np.arange(0, t_final, self.cfg.dt)
        yd = np.zeros_like(t)
        ypd = np.zeros_like(t)

        self.filter.reset()
        self.phase = "home"

        for i, ti in enumerate(t):
            yd[i], ypd[i] = self.step(ti)

        return t, yd, ypd