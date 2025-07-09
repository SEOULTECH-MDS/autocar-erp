import matplotlib.pyplot as plt
import numpy as np

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """Draw arrow for pose visualization."""
    plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
              head_length=width, head_width=width, fc=fc, ec=ec)