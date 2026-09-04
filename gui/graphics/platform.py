# gui/graphics/platform.py
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from config.parameters import Az, Bz, D

class PlatformCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(5, 4), facecolor="#1b263b")
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_facecolor("#1b263b")
        # Sin caja/paneles matplotlib: solo dibujamos los ejes XYZ propios abajo.
        self.ax.set_axis_off()
        self.ax.view_init(elev=26, azim=-58)
        self.ax.set_box_aspect((1, 1, 0.9))

        
        axis_length = 0.5
        self.ax.plot([0, axis_length], [0, 0], [0, 0], color='red', linewidth=2.5, linestyle='-', alpha=0.8)
        self.ax.text(axis_length + 0.05, 0, 0, 'X', color='red', fontsize=12)
        self.ax.plot([0, 0], [0, axis_length], [0, 0], color='green', linewidth=2.5, linestyle='-', alpha=0.8)
        self.ax.text(0, axis_length + 0.05, 0, 'Y', color='green', fontsize=12)
        self.ax.plot([0, 0], [0, 0], [0, axis_length], color='blue', linewidth=2.5, linestyle='-', alpha=0.8)
        self.ax.text(0, 0, axis_length + 0.05, 'Z', color='blue', fontsize=12)


        self.base_points = Bz.copy()
        self.platform_reference_points = Az.copy()

        base_loop = np.vstack([self.base_points, self.base_points[0]])
        self.base_outline, = self.ax.plot(
            base_loop[:, 0],
            base_loop[:, 1],
            base_loop[:, 2],
            color="#00bfff",
            linewidth=1.8,
            linestyle="--",
            alpha=0.95,
        )
        self.base_scatter = self.ax.scatter(
            self.base_points[:, 0],
            self.base_points[:, 1],
            self.base_points[:, 2],
            c="#00bfff",
            s=40,
            depthshade=False,
        )

        initial_translation = D.copy()
        self.platform_points = initial_translation + self.platform_reference_points
        platform_loop = np.vstack([self.platform_points, self.platform_points[0]])
        self.platform_poly = Poly3DCollection(
            [platform_loop],
            facecolors="#ff4444",
            alpha=0.25,
            edgecolors="#ff6666",
            linewidths=2,
        )
        self.ax.add_collection3d(self.platform_poly)
        self.platform_outline, = self.ax.plot(
            platform_loop[:, 0],
            platform_loop[:, 1],
            platform_loop[:, 2],
            color="#ff6666",
            linewidth=2,
        )
        self.platform_scatter = self.ax.scatter(
            self.platform_points[:, 0],
            self.platform_points[:, 1],
            self.platform_points[:, 2],
            c="#ff6666",
            s=50,
            depthshade=False,
        )

        self.leg_lines = []
        self.leg_colors = ["#ff6b6b", "#4ecdc4", "#45b7d1", "#96ceb4", "#ffeaa7", "#dfe6e9"]
        for i in range(6):
            line, = self.ax.plot(
                [], [], [],
                color=self.leg_colors[i],
                linewidth=3.0,
                alpha=1.0
            )
            self.leg_lines.append(line)

        # Numerar las 6 patas junto a su anclaje en la base (punto fijo)
        for i, p in enumerate(self.base_points):
            self.ax.text(p[0], p[1], p[2] - 0.05, str(i + 1), color=self.leg_colors[i], fontsize=10, weight="bold")

        # Dibujar actuadores desde el primer frame para que sean visibles aun sin iniciar simulacion.
        self._update_leg_lines(self.platform_points)

        # Encuadre ajustado a la geometría real (base + plataforma home), no una caja fija de 2.5 m.
        all_points = np.vstack([self.base_points, self.platform_points])
        margin = 0.15
        mins = all_points.min(axis=0) - margin
        maxs = all_points.max(axis=0) + margin
        self.ax.set_xlim(mins[0], maxs[0])
        self.ax.set_ylim(mins[1], maxs[1])
        self.ax.set_zlim(mins[2], maxs[2])

        self.fig.tight_layout()

    def _update_leg_lines(self, platform_points):
        for i, line in enumerate(self.leg_lines):
            line.set_data(
                [self.base_points[i, 0], platform_points[i, 0]],
                [self.base_points[i, 1], platform_points[i, 1]],
            )
            line.set_3d_properties([self.base_points[i, 2], platform_points[i, 2]])

    def update_platform(self, translation, rotation):
        self.platform_points = translation + self.platform_reference_points @ rotation.T
        platform_loop = np.vstack([self.platform_points, self.platform_points[0]])

        self.platform_poly.set_verts([platform_loop])

        # Asegúrate de que ambas partes de la actualización 3D se llamen
        self.platform_outline.set_data(platform_loop[:, 0], platform_loop[:, 1])
        self.platform_outline.set_3d_properties(platform_loop[:, 2])
        
        self.platform_scatter._offsets3d = (
            self.platform_points[:, 0],
            self.platform_points[:, 1],
            self.platform_points[:, 2],
        )

        self._update_leg_lines(self.platform_points)
        self.draw_idle()
