import numpy as np

def angle_mod(x, zero_2_2pi=False):
    if zero_2_2pi:
        return x % (2 * np.pi)
    else:
        return (x + np.pi) % (2 * np.pi) - np.pi

def rot_mat_2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])