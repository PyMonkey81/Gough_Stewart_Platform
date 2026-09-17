# gui/graphics/platform.py
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import numpy as np

from config.parameters import Az, Bz, D

class PlatformCanvas(FigureCanvas):
    def __init__(self, parent=None):
        # Figura con márgenes cero y fondo igual que dashboard
        self.fig = Figure(figsize=(5, 4), facecolor="#161B22", edgecolor="none", linewidth=0)
        self.fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
        
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_facecolor("#0E1116")
        self.ax.set_axis_off()
        
        # Cámara isométrica: elev=30, azim=-45 aproxima bien a isométrico
        self.ax.view_init(elev=30, azim=-45)
        self.ax.set_box_aspect((1, 1, 0.95))

        # ========== Ejes XYZ del mundo (origen en base) ==========
        axis_length = 0.5
        self.ax.plot([0, axis_length], [0, 0], [0, 0], 
                    color='#FF6B6B', linewidth=2.8, alpha=0.9)
        self.ax.text(axis_length + 0.06, 0, 0, 'X', color='#FF6B6B', fontsize=11, weight='bold')
        
        self.ax.plot([0, 0], [0, axis_length], [0, 0], 
                    color='#43A047', linewidth=2.8, alpha=0.9)
        self.ax.text(0, axis_length + 0.06, 0, 'Y', color='#43A047', fontsize=11, weight='bold')
        
        self.ax.plot([0, 0], [0, 0], [0, axis_length], 
                    color='#42A5F5', linewidth=2.8, alpha=0.9)
        self.ax.text(0, 0, axis_length + 0.06, 'Z', color='#42A5F5', fontsize=11, weight='bold')

        # ========== Puntos de geometría ==========
        self.base_points = Bz.copy()
        self.platform_reference_points = Az.copy()

        # ========== Grid sutil en el piso (z = z_base) ==========
        z_base = self.base_points[0, 2]
        grid_spacing = 0.15
        grid_range = 0.8
        grid_lines = []
        for x in np.arange(-grid_range, grid_range + grid_spacing, grid_spacing):
            grid_lines.append([(x, -grid_range, z_base), (x, grid_range, z_base)])
        for y in np.arange(-grid_range, grid_range + grid_spacing, grid_spacing):
            grid_lines.append([(-grid_range, y, z_base), (grid_range, y, z_base)])
        
        self.grid_collection = Line3DCollection(
            grid_lines, colors='#2a3a4a', linewidths=0.5, alpha=0.3
        )
        self.ax.add_collection3d(self.grid_collection)

        # ========== Hexágono base (B1..B6) ==========
        base_loop = np.vstack([self.base_points, self.base_points[0]])
        self.base_outline, = self.ax.plot(
            base_loop[:, 0], base_loop[:, 1], base_loop[:, 2],
            color="#00D4FF", linewidth=2.2, alpha=1.0, zorder=100
        )
        self.base_scatter = self.ax.scatter(
            self.base_points[:, 0], self.base_points[:, 1], self.base_points[:, 2],
            c="#00D4FF", s=60, depthshade=False, alpha=1.0, zorder=101
        )
        
        # Labels B1..B6 en puntos base
        for i, p in enumerate(self.base_points):
            self.ax.text(p[0]*1.15, p[1]*1.15, p[2] - 0.08,
                        f"B{i+1}", color="#00D4FF", fontsize=9, weight="bold",
                        ha='center', va='center')

        # ========== Hexágono plataforma (P1..P6) ==========
        initial_translation = D.copy()
        self.platform_points = initial_translation + self.platform_reference_points
        platform_loop = np.vstack([self.platform_points, self.platform_points[0]])
        
        # Polígono semi-opaco
        self.platform_poly = Poly3DCollection(
            [platform_loop], facecolors="#FF6B6B", alpha=0.35,
            edgecolors="#FF9999", linewidths=2.5, zorder=50
        )
        self.ax.add_collection3d(self.platform_poly)
        
        self.platform_outline, = self.ax.plot(
            platform_loop[:, 0], platform_loop[:, 1], platform_loop[:, 2],
            color="#FF9999", linewidth=2.5, alpha=1.0, zorder=100
        )
        
        self.platform_scatter = self.ax.scatter(
            self.platform_points[:, 0], self.platform_points[:, 1], self.platform_points[:, 2],
            c="#FF9999", s=70, depthshade=False, alpha=1.0, marker='s', zorder=101
        )
        
        # Labels P1..P6 en puntos plataforma
        self.platform_labels = []
        for i in range(6):
            txt = self.ax.text(0, 0, 0, f"P{i+1}", color="#FF9999", fontsize=9,
                              weight="bold", ha='center', va='center')
            self.platform_labels.append(txt)

        # ========== Segmentos actuadores (6 líneas de color distinto) ==========
        self.leg_lines = []
        self.leg_colors = ["#FF6B6B", "#4ECDC4", "#45B7D1", "#96CEB4", "#FFEAA7", "#DFE6E9"]
        for i in range(6):
            line, = self.ax.plot(
                [], [], [],
                color=self.leg_colors[i], linewidth=3.5, alpha=0.95, zorder=75
            )
            self.leg_lines.append(line)

        # ========== Flecha de orientación (eje Z local de la plataforma) ==========
        # Muestra la dirección "hacia arriba" de la plataforma
        self.orientation_arrow = None
        
        # ========== Límites iniciales ==========
        all_points = np.vstack([self.base_points, self.platform_points])
        margin = 0.25
        mins = all_points.min(axis=0) - margin
        maxs = all_points.max(axis=0) + margin
        self.ax.set_xlim(mins[0], maxs[0])
        self.ax.set_ylim(mins[1], maxs[1])
        self.ax.set_zlim(mins[2], maxs[2])
        
        # Inicializar con primer frame
        self._update_leg_lines(self.platform_points)
        self._update_orientation_arrow(np.eye(3))

    def _update_leg_lines(self, platform_points):
        """Actualiza líneas de actuadores (B_i -> P_i) eficientemente."""
        for i, line in enumerate(self.leg_lines):
            line.set_data(
                [self.base_points[i, 0], platform_points[i, 0]],
                [self.base_points[i, 1], platform_points[i, 1]],
            )
            line.set_3d_properties([self.base_points[i, 2], platform_points[i, 2]])

    def _update_orientation_arrow(self, rotation_matrix):
        """Dibuja/actualiza flecha de orientación (eje Z local de la plataforma).
        
        La flecha sale del centroide de la plataforma y muestra hacia dónde 
        apunta el eje Z local (up-vector) después de la rotación.
        """
        # Centro de la plataforma
        platform_center = self.platform_points.mean(axis=0)
        
        # Eje Z local después de rotación (tercera columna de R)
        z_local = rotation_matrix[:, 2]  # vector normalizado del eje Z
        arrow_length = 0.15
        arrow_tip = platform_center + z_local * arrow_length
        
        # Eliminar flecha anterior si existe
        if self.orientation_arrow is not None:
            self.orientation_arrow.remove()
        
        # Dibujar nueva flecha (línea gruesa + punta)
        self.orientation_arrow, = self.ax.plot(
            [platform_center[0], arrow_tip[0]],
            [platform_center[1], arrow_tip[1]],
            [platform_center[2], arrow_tip[2]],
            color="#FFD700", linewidth=4.0, alpha=0.9, zorder=110
        )
        
        # Punta de flecha (pequeño scatter)
        self.ax.scatter(
            [arrow_tip[0]], [arrow_tip[1]], [arrow_tip[2]],
            c="#FFD700", s=100, depthshade=False, alpha=0.9,
            marker='^', zorder=111
        )

    def update_platform(self, translation, rotation):
        """Actualiza posición y orientación de la plataforma.
        
        Args:
            translation: array [x, y, z] - posición del centro de plataforma
            rotation: array 3x3 - matriz de rotación R de la plataforma
        
        Eficiencia: solo actualiza lo necesario (sin recálculos IK).
        """
        self.platform_points = translation + self.platform_reference_points @ rotation.T
        platform_loop = np.vstack([self.platform_points, self.platform_points[0]])

        # Actualizar polígono (cara de la plataforma)
        self.platform_poly.set_verts([platform_loop])

        # Actualizar outline (perímetro)
        self.platform_outline.set_data(platform_loop[:, 0], platform_loop[:, 1])
        self.platform_outline.set_3d_properties(platform_loop[:, 2])
        
        # Actualizar puntos scatter
        self.platform_scatter._offsets3d = (
            self.platform_points[:, 0],
            self.platform_points[:, 1],
            self.platform_points[:, 2],
        )
        
        # Actualizar labels P1..P6 con nueva posición
        for i, txt in enumerate(self.platform_labels):
            txt.set_position((self.platform_points[i, 0], self.platform_points[i, 1]))
            txt.set_3d_properties(z=self.platform_points[i, 2], zdir='z')

        # Actualizar segmentos actuadores
        self._update_leg_lines(self.platform_points)
        
        # Actualizar flecha de orientación
        self._update_orientation_arrow(rotation)
        
        # Renderizar cambios
        self.draw_idle()

    def update_platform_at_home(self):
        """HOME especial: q=0 (6 actuadores a ACTUATOR_MIN), sin IK.
        
        Dibuja plataforma plana sin rotación, ejes retraídos (todos a MIN),
        flecha apuntando +Z (cenit), sin marcadores amarillos en waypoints.
        """
        # HOME: d_a = D (sin traslación extra), R = I (sin rotación)
        translation = D.copy()
        rotation = np.eye(3)
        
        self.platform_points = translation + self.platform_reference_points @ rotation.T
        platform_loop = np.vstack([self.platform_points, self.platform_points[0]])

        # Actualizar polígono (cara de la plataforma)
        self.platform_poly.set_verts([platform_loop])

        # Actualizar outline (perímetro)
        self.platform_outline.set_data(platform_loop[:, 0], platform_loop[:, 1])
        self.platform_outline.set_3d_properties(platform_loop[:, 2])
        
        # Actualizar puntos scatter
        self.platform_scatter._offsets3d = (
            self.platform_points[:, 0],
            self.platform_points[:, 1],
            self.platform_points[:, 2],
        )
        
        # Actualizar labels P1..P6
        for i, txt in enumerate(self.platform_labels):
            txt.set_position((self.platform_points[i, 0], self.platform_points[i, 1]))
            txt.set_3d_properties(z=self.platform_points[i, 2], zdir='z')

        # Actualizar segmentos actuadores (todos de igual longitud a ACTUATOR_MIN)
        self._update_leg_lines(self.platform_points)
        
        # Actualizar flecha de orientación (sin marcadores amarillos, solo línea)
        self._update_orientation_arrow_home(rotation)
        
        # Renderizar cambios
        self.draw_idle()

    def _update_orientation_arrow_home(self, rotation_matrix):
        """Dibuja flecha de orientación en HOME SIN marcadores amarillos."""
        # Centro de la plataforma
        platform_center = self.platform_points.mean(axis=0)
        
        # Eje Z local después de rotación
        z_local = rotation_matrix[:, 2]
        arrow_length = 0.15
        arrow_tip = platform_center + z_local * arrow_length
        
        # Eliminar flecha anterior si existe
        if self.orientation_arrow is not None:
            self.orientation_arrow.remove()
        
        # Dibujar SOLO la línea, sin punta amarilla
        self.orientation_arrow, = self.ax.plot(
            [platform_center[0], arrow_tip[0]],
            [platform_center[1], arrow_tip[1]],
            [platform_center[2], arrow_tip[2]],
            color="#FFD700", linewidth=4.0, alpha=0.9, zorder=110
        )
