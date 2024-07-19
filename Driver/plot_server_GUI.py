import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QComboBox
from PyQt6.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from plot_server import LivePlotter

class LivePlotterGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.headers = []
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

        # Dropdowns to select features
        self.feature_selectors = []
        for i in range(5):
            selector = QComboBox(self)
            selector.addItem("None")
            selector.currentIndexChanged.connect(self.update_feature_selection)
            layout.addWidget(selector)
            self.feature_selectors.append(selector)

        self.setLayout(layout)

        self.plotter = LivePlotter(self.fig, self.axs, keep_samples=self.slider.value(), header_callback=self.update_headers)
        self.ani = None

        self.start_animation()

    def update_samples(self, value):
        if self.plotter:
            self.plotter.set_keep_samples(value)

    def update_feature_selection(self):
        selected_features = [selector.currentText() for selector in self.feature_selectors]
        self.plotter.update_selected_features(selected_features)

    def start_animation(self):
        self.ani = animation.FuncAnimation(self.fig, self.plotter.animate, interval=200)
        self.canvas.draw()

    def update_headers(self, headers, selected_features):
        self.headers = headers
        for i, selector in enumerate(self.feature_selectors):
            selector.clear()
            selector.addItem("None")
            selector.addItems(headers)
            selector.setCurrentText(selected_features[i])

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
