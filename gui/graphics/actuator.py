# gui/graphics/actuator.py


from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
class ActuatorCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(6, 3.2), facecolor="#1e1e1e")
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#1e1e1e")
        self.ax.set_title("Posiciones de actuadores (%)", color="#e0e0e0", fontsize=11)
        self.ax.set_xlabel("Tiempo (s)", color="#aaa")
        self.ax.set_ylabel("Posición (%)", color="#aaa")
        self.ax.tick_params(colors="#888")
        for spine in self.ax.spines.values():
            spine.set_color("#555")
        self.ax.set_ylim(0, 100)
        self.ax.grid(True, color="#333", linestyle="--", alpha=0.6)

        self.colors = ["#ff6b6b", "#4ecdc4", "#45b7d1", "#96ceb4", "#ffeaa7", "#dfe6e9"]
        self.lines = []
        self.data = [[] for _ in range(6)]
        self.time = []

        for i in range(6):
            line, = self.ax.plot([], [], color=self.colors[i], linewidth=1.8, label=f"A{i+1}")
            self.lines.append(line)

        self.ax.legend(loc="upper right", ncol=3, facecolor="#2d2d2d", edgecolor="#555", labelcolor="#ccc", fontsize=8)
        self.fig.tight_layout()

    def update_data(self, t, lengths):
        self.time.append(t)
        for i in range(6):
            self.data[i].append(lengths[i])
            self.lines[i].set_data(self.time, self.data[i])

        max_points = 200
        if len(self.time) > max_points:
            self.time = self.time[-max_points:]
            for i in range(6):
                self.data[i] = self.data[i][-max_points:]
                self.lines[i].set_data(self.time, self.data[i])

        self.ax.set_xlim(max(0, t - 15), t + 0.5)
        self.draw_idle()