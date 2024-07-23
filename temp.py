import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


class DynamicSubplotsAnimation:
    def __init__(self):
        self.fig, axs = plt.subplots(2, 1)
        if isinstance(axs, np.ndarray):
            self.axs = axs.tolist()
        else:
            self.axs = [axs]

        self.n_subplots = 1

        # Initialize data
        self.x = np.linspace(0, 2 * np.pi, 100)
        self.y = np.sin(self.x)

        # Create the animation
        self.ani = animation.FuncAnimation(self.fig, self.update, frames=range(1, 100), interval=100, repeat=False)

    def update_subplots(self, num):
        self.n_subplots = num
        self.fig.clf()  # Clear the current figure
        self.axs = self.fig.subplots(self.n_subplots, 1)  # Create new subplots
        if self.n_subplots == 1:
            self.axs = [self.axs]  # Ensure axs is always a list for consistency

    def update(self, frame):
        # Change number of subplots every second (assuming 10 frames per second)
        new_n_subplots = (frame // 10 % 4) + 1
        if new_n_subplots != self.n_subplots:
            self.update_subplots(new_n_subplots)

        for idx, ax in enumerate(self.axs):
            ax.clear()  # Clear each subplot
            ax.plot(self.x, self.y * (frame / 10))  # Update the plot with new data
            ax.set_title(f'Subplot {idx + 1}')
        plt.tight_layout()

    def show(self):
        plt.show()


# Create and display the animation
dynamic_animation = DynamicSubplotsAnimation()
dynamic_animation.show()
