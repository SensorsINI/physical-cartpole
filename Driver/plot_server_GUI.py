import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider
from PyQt6.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from plot_server import LivePlotter

class LivePlotterGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Live Plotter Control Panel')

        layout = QVBoxLayout()

        # Create matplotlib figure and axes
        self.fig, self.axs = plt.subplots(5, 2, figsize=(16, 9), gridspec_kw={'width_ratios': [3, 1]})
        self.fig.subplots_adjust(hspace=0.8)

        # Embed the matplotlib figure in the PyQt6 window
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        # Add navigation toolbar
        self.toolbar = NavigationToolbar(self.canvas, self)
        layout.addWidget(self.toolbar)

        # Slider to control keep_samples
        self.label = QLabel('Number of Samples to Keep:', self)
        layout.addWidget(self.label)

        self.slider = QSlider(Qt.Orientation.Horizontal, self)
        self.slider.setMinimum(10)
        self.slider.setMaximum(1000)
        self.slider.setValue(100)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.slider.valueChanged.connect(self.update_samples)
        layout.addWidget(self.slider)

        self.setLayout(layout)

        self.plotter = LivePlotter(self.fig, self.axs, keep_samples=self.slider.value())
        self.ani = None

        self.start_animation()

    def update_samples(self, value):
        if self.plotter:
            self.plotter.set_keep_samples(value)

    def start_animation(self):
        self.ani = animation.FuncAnimation(self.fig, self.plotter.animate, interval=200)
        self.canvas.draw()

    def resizeEvent(self, event):
        # Adjust the layout and elements on window resize
        self.canvas.draw()
        super().resizeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = LivePlotterGUI()
    gui.show()  # Show the GUI in windowed mode
    sys.exit(app.exec())


# TODO: Here removing self.setGeometry(100, 100, 800, 600) helped. Maybe in Cartpole it would help to for shifted x axis
