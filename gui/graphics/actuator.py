# gui/graphics/actuator.py


from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
class ActuatorCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(6, 3.2), facecolor="#161B22")
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#161B22")
        self.ax.set_title("ACTUADORES", color="#e0e6ed", fontsize=12, fontweight="bold")
        self.ax.set_xlabel("Tiempo (s)", color="#aaa")
        self.ax.set_ylabel("Posición (%)", color="#aaa")
        self.ax.tick_params(colors="#888")
        for spine in self.ax.spines.values():
            spine.set_color("#555")
        self.ax.set_ylim(0, 100)
        self.ax.grid(True, color="#333", linestyle="--", alpha=0.6)

        self.time_window = 653.0
        self.max_history = 60.0  # Recortar a últimos 60s
        self.colors = ["#ff6b6b", "#4ecdc4", "#45b7d1", "#96ceb4", "#ffeaa7", "#dfe6e9"]
        self.lines = []
        self.data = [[] for _ in range(6)]
        self.time = []

        for i in range(6):
            line, = self.ax.plot([], [], color=self.colors[i], linewidth=1.8, label=f"A{i+1}")
            self.lines.append(line)

        self.ax.legend(loc="upper right", ncol=3, facecolor="#2d2d2d", edgecolor="#555", labelcolor="#ccc", fontsize=8)
        self.fig.tight_layout()

    def set_time_window(self, total_time: float):
        self.time_window = max(float(total_time), 0.0)
        if self.time_window > 0:
            self.ax.set_xlim(0.0, self.time_window)
            ticks = [0.0, self.time_window * 0.25, self.time_window * 0.5, self.time_window * 0.75, self.time_window]
            self.ax.set_xticks(ticks)
        self.draw_idle()

    def update_data(self, t, lengths):
        """Agrega un punto a la gráfica. Recorta a los últimos 60s y redibuja."""
        self.time.append(t)
        for i in range(6):
            self.data[i].append(lengths[i])
        
        # Recortar a últimos 60s
        if len(self.time) > 0:
            t_min = t - self.max_history
            while len(self.time) > 0 and self.time[0] < t_min:
                self.time.pop(0)
                for i in range(6):
                    self.data[i].pop(0)
        
        # Actualizar líneas
        for i in range(6):
            self.lines[i].set_data(self.time, self.data[i])
        
        self.ax.set_xlim(0.0, self.time_window)
        self.draw_idle()

    def clear(self):
        """Reinicia el historial para arrancar una nueva corrida desde t=0."""
        self.time = []
        self.data = [[] for _ in range(6)]
        for line in self.lines:
            line.set_data([], [])
        self.draw_idle()