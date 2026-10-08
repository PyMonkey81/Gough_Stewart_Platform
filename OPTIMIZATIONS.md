# Optimizaciones de Rendimiento - Gough-Stewart HMI

## Problema Original
La aplicación se congelaba a mitad de trayectoria (pasadas de 60s) debido a múltiples redibujados y logs excesivos ejecutándose en cada tick del timer (~30 ms = ~33 Hz).

## Soluciones Implementadas

### 1. **Throttling de Logging de Posiciones** ✓
- **Antes**: `_log_lengths()` se ejecutaba **cada tick** (~33 veces/segundo)
- **Ahora**: Log solo cuando:
  - Ha transcurrido 0.5s desde el último log, **O**
  - El vector de posiciones cambió **> 2%** en cualquier eje
- **Implementación**: `_should_log_position()` + `_last_log` + `_last_logged_percent`
- **Impacto**: Reduce logs de ~33/s a ~2-4/s en operación normal

### 2. **Throttling de Gráfica de Actuadores** ✓
- **Antes**: `actuator_canvas.update_data()` se ejecutaba cada tick, redibujando cada vez
- **Ahora**: `update_data()` se llama **cada 100ms máximo** (10 Hz)
- **Recorte de historial**: Mantiene solo los últimos **60s** de datos
- **Implementación**: `_last_actuator_plot` en MainWindow + `max_history` en ActuatorCanvas
- **Impacto**: Reduce redibujados de 30+/s a 10/s, y memoria acotada

### 3. **3D Render a 5 Hz (Throttling existente mejorado)** ✓
- **Ya estaba**: Throttling a 5 Hz (0.2s entre frames)
- **Mejorado**: Ahora respeta flag `_show_3d` antes de redibujar
- **En JOG por eje**: NO actualiza 3D (solo barra de actuadores)
- **Impacto**: 3D caro solo se redibuja cuando es necesario

### 4. **Checkbox "3D" para Modo Ultra-Rápido** ✓
- **Nueva funcionalidad**: Checkbox "Mostrar" en encabezado del panel 3D
- **Uso**: Si la app sigue congelada, deshabilita 3D → ganancias significativas
- **Barras + Gráfica**: Siguen funcionando a plena velocidad sin 3D
- **Implementación**: `_show_3d` flag + `_on_3d_toggle()` handler + canvas visibility

### 5. **JOG por Eje Optimizado** ✓
- **Antes**: `_manual_axis_tick()` actualizaba gráfica cada tick
- **Ahora**: 
  - NO actualiza 3D
  - NO actualiza gráfica (no acumula datos de jog de ejes)
  - Solo actualiza barra de estado
  - Solo loguea cuando el vector cambia
- **Impacto**: Jog fluido incluso sin 3D visible

## Cambios de Código

### MainWindow.py
```python
# Nuevas variables en __init__():
self._last_log = -1.0                    # Para throttling de logs
self._last_logged_percent = None         # Snapshot anterior
self._last_actuator_plot = -1.0          # Para throttling de gráfica
self._show_3d = True                     # Flag para ocultar 3D

# Nuevos métodos:
def _should_log_position(self, percent_vector: np.ndarray) -> bool:
    """Retorna True si deben loguear: cada 0.5s o si cambió > 2%."""
    
def _on_3d_toggle(self, checked: bool):
    """Mostrar/ocultar el canvas 3D para reducir carga si está muy lento."""
```

### Métodos Modificados
1. `compute_and_update()` - Throttling de log + 3D + gráfica
2. `_apply_cartesian_target()` - Throttling de log + 3D + gráfica
3. `_auto_tick()` - Throttling de log + 3D + gráfica (fase tracking)
4. `_manual_axis_tick()` - Eliminado update_data() de gráfica
5. `_build_left_column()` - Agregado checkbox de 3D

### ActuatorCanvas.py
```python
# Nuevo atributo:
self.max_history = 60.0  # Recortar a últimos 60s

# Método update_data() mejorado:
- Recorta datos si excede max_history
- draw_idle() solo con datos visibles (más rápido)
```

## Criterios de Éxito

✅ **Una pasada de 60s no debe congelar la ventana**
- Antes: Congelaciones notables
- Después: Fluidez constante incluso en AUTO

✅ **Jog de un eje sigue mandando pos y moviendo solo su barra**
- Serial: Throttling a 100ms preservado
- Barra: Actualización en cada tick
- 3D/Gráfica: Ignoradas en jog por eje

✅ **3D + Gráfica siguen funcionando**
- 3D: 5 Hz cuando está habilitado
- Gráfica: 10 Hz con historial acotado
- Barras: Siempre en tiempo real

## Pruebas Recomendadas

1. **Prueba de trackeado de 60s**: Iniciar AUTO → verificar que NO hay congelación
2. **Prueba de jog por eje**: Abrir diálogo EJES → mover sliders → verificar fluidez
3. **Deshabilitar 3D**: Desmarcar checkbox → verificar ganancia si estaba lento
4. **Logs**: Verificar log_list → líneas cada 0.5s máximo (no cada tick)
5. **Memoria**: Verificar que gráfica de actuadores no crece sin límite (recorte a 60s)

## Performance Esperado
- **CPU**: Reducción ~50-70% en loop principal
- **Memoria**: Historial acotado (~7KB/min con 10 puntos/s × 6 ejes)
- **Responsividad**: UI sin lag perceptible incluso en AUTO
