import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Initialize data
x = np.linspace(0, 2 * np.pi, 100)
y = np.sin(x)

# Global variables to manage subplots
fig, axs = plt.subplots(1, 1)
axs = [axs]  # Ensure axs is always a list
n_subplots = 1


# Function to update the number of subplots dynamically
def update_subplots(num):
    global fig, axs, n_subplots
    n_subplots = num
    fig.clf()  # Clear the current figure
    axs = fig.subplots(n_subplots, 1)  # Create new subplots
    if n_subplots == 1:
        axs = [axs]  # Ensure axs is always a list for consistency


# Animation update function
def update(frame):
    global n_subplots
    # Change number of subplots every second (assuming 10 frames per second)
    new_n_subplots = (frame // 10 % 4) + 1
    if new_n_subplots != n_subplots:
        update_subplots(new_n_subplots)

    for idx, ax in enumerate(axs):
        ax.clear()  # Clear each subplot
        ax.plot(x, y * (frame / 10))  # Update the plot with new data
        ax.set_title(f'Subplot {idx + 1}')
    plt.tight_layout()


# Create the animation
ani = animation.FuncAnimation(fig, update, frames=range(1, 100), interval=100, repeat=False)

plt.show()
