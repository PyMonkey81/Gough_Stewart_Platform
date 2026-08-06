import numpy as np
from kinematics.inverse import inverse_kinematics
from kinematics.jacobian import InvJac
from control.sliding_pi import SlidingPIController
from config.parameters import DT

def main():
    controller = SlidingPIController()
    
    # Ejemplo de loop básico
    t = 0.0
    while t < 100.0:
        # 1. Generar referencia (aquí irá el TrajectoryGenerator)
        y_des = np.array([0.1, 0.2])          # ejemplo
        yp_des = np.array([0.0, 0.0])

        # 2. Cinemática inversa
        qd, da, R = inverse_kinematics(y_des)

        # 3. Jacobiano + twist (cuando lo terminemos)
        # A, _ = InvJac(da, R)
        # twist = ...
        # qpd = A @ twist

        # 4. Errores (cuando tengamos la planta)
        # delta_q  = qd - q_actual
        # delta_qp = qpd - qp_actual
        # Tau = controller.step(delta_q, delta_qp)

        t += DT

if __name__ == "__main__":
    main()