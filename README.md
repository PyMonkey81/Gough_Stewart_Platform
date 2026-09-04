# Gough-Stewart Platform — HMI

Dashboard industrial oscuro (PySide6) para controlar y visualizar una plataforma
Gough-Stewart 6-DOF: vista 3D, gráfica de actuadores, estado por eje, control de
movimiento (TAREA/CARTESIANO) y bitácora de eventos.

## Instalación

```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux/macOS
source .venv/bin/activate

pip install -r requirements.txt
```

## Ejecución

```bash
python main.py
```

## Modos de movimiento (MOTION CONTROL)

La columna **MOTION CONTROL** tiene un selector con dos modos de pose, cada uno con
sus propios sliders/spin boxes y botón **GOTO**:

| Modo           | Controles                                                                                    | Cinemática usada                                                                      | Notas                                                                                                                                                                                                                                                                      |
| -------------- | -------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **TAREA**      | α, β (deg), límite ±`ALPHA_BETA_LIMIT_DEG`                                                   | `kinematics/inverse.py` (`TIK` + `PIK`, sin modificar)                                | Único modo donde **AUTO** (`TrajectoryGenerator`) tiene sentido. Preset **Tracking demo**.                                                                                                                                                                                 |
| **CARTESIANO** | X, Y, Z (m) y Roll, Pitch, Yaw (deg), límites en `config/parameters.py` (`CARTESIAN_LIMITS`) | `kinematics/pose.py` (`rpy_to_R` + `pose_to_q`, arma `da`/`R` y llama al mismo `PIK`) | Siempre **MANUAL**; si se selecciona estando en AUTO, se fuerza MANUAL. Si la pose produce longitudes fuera de `[ACTUATOR_MIN, ACTUATOR_MAX]`, no se envía por serial: el eje afectado se marca en rojo en **ACTUATOR STATUS** y aparece un aviso "pose fuera de carrera". |

En ambos modos, **GOTO** aplica la pose de los sliders; en MANUAL solo sale por serial
si **INICIAR** está activo (igual que el jog de ejes).

## Run / Iniciar / Paro

- **MANUAL** (checkbox "AUTO (trayectoria)" desmarcado): **INICIAR** empieza a enviar
  periódicamente (≥100 ms) la pose/ejes actuales; **PARO** deja de enviar y congela la
  pose mostrada.
- **AUTO** (checkbox marcado, solo disponible en TAREA): **INICIAR** corre el
  `TrajectoryGenerator` (α, β) y se detiene solo tras `DEMO_DURATION` segundos (60 s por
  defecto, configurable en **Parámetros → Trayectoria**); **PARO** congela el lazo.
- Cambiar de modo (TAREA↔CARTESIANO) o alternar AUTO/MANUAL detiene el lazo
  automáticamente para no mezclar referencias.

### Jog por eje

El botón **Jog por eje** abre el diálogo existente de jog directo (0-100 % por pata,
con checkbox **Habilitar**). Mientras ese diálogo está abierto, el lazo MANUAL envía el
vector de 6 ejes en vez de la pose TAREA/CARTESIANO; no sustituye el jog cartesiano, son
dos formas de generar el mismo vector de 6 posiciones.

## Cómo joggear ejes manualmente

1. Abrir **Jog por eje** (funciona en cualquier modo, mientras el diálogo esté visible
   tiene prioridad sobre TAREA/CARTESIANO).
2. Marcar **Habilitar** en los ejes deseados y mover su slider/spin box.
3. Pulsar **INICIAR** en la ventana principal para comenzar a enviar el vector de 6
   posiciones por serial (si hay conexión activa).
4. Pulsar **PARO** para detener el envío sin perder los valores configurados.

## Serial

- Puertos soportados: Windows (`COM*`), Linux (`/dev/ttyACM*`, `/dev/ttyUSB*`). El primer
  puerto disponible se sugiere por defecto; no hay puerto fijo predeterminado.
- Protocolo de texto (una línea por comando):
  - `pos i,i,i,i,i,i` — 6 enteros de 0 a 100 (%).
  - `home` — solo se envía al pulsar el botón **HOME**; no se envía automáticamente
    al conectar.
  - `ping` — heartbeat periódico mientras hay conexión activa.
- Envío limitado a máximo 1 comando `pos` cada ≥100 ms.

## Notas de implementación

- La cinemática inversa (`kinematics/inverse.py`, `TIK`/`PIK`) y la fórmula del
  controlador (`control/sliding_pi.py`) no se modifican.
- `kinematics/pose.py` es un helper nuevo para el modo CARTESIANO: arma `da`/`R` a
  partir de (x, y, z, roll, pitch, yaw) y llama al mismo `PIK`, sin tocar `TIK`.
- El lazo de la GUI corre en modo IK abierta (sin controlador en el lazo serial): la
  pose deseada se convierte a longitudes de actuador y estas a porcentaje de carrera.
- El log de eventos (conexión, home, pos, errores de IK) se muestra en el panel
  **EVENT LOG** de la columna derecha.
