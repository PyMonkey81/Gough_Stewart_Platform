# gui/graphics/platform.py
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from config.parameters import Az, Bz, D

class PlatformCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(5, 4), facecolor="#1e1e1e")
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_facecolor("#1e1e1e")
        self.ax.set_title("Plataforma Gough-Stewart", color="#e0e0e0", fontsize=11)
        self.ax.set_xlabel("X", color="#aaa")
        self.ax.set_ylabel("Y", color="#aaa")
        self.ax.set_zlabel("Z", color="#aaa")
        self.ax.tick_params(colors="#888")
        self.ax.view_init(elev=26, azim=-58)
        self.ax.set_xlim(-0.8, 0.8)
        self.ax.set_ylim(-0.8, 0.8)
        self.ax.set_zlim(0.0, 2.8)
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
        for _ in range(6):
            # ===============================================
            # CAMBIOS AQUI: ACTUADORES MAS VISIBLES
            # ===============================================
            line, = self.ax.plot(
                [], [], [],
                color="#ffff00",      # Un color brillante que contraste
                linewidth=3.0,       # Mayor grosor para que destaquen
                alpha=1.0            # Completamente opaco
            )
            self.leg_lines.append(line)

        # Dibujar actuadores desde el primer frame para que sean visibles aun sin iniciar simulacion.
        self._update_leg_lines(self.platform_points)

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
