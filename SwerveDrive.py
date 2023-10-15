import numpy as np


def get_pythag_C(x, y):
    return np.sqrt(x * x + y * y)


class SwerveDrive:
    def __init__(self, length, width):
        self.length = length
        self.width = width

        self.radius = get_pythag_C(length, width)

        self.cos01 = self.width / (2 * self.radius)
        self.cos02 = self.length / (2 * self.radius)

    def get_module_setpoints(self, x0, y0, x1):
        m1x = x0 + x1 * self.cos02
        m1y = y0 - x1 * self.cos01

        m2x = x0 - x1 * self.cos01
        m2y = y0 - x1 * self.cos02

        m3x = x0 - x1 * self.cos02
        m3y = y0 + x1 * self.cos01

        m4x = x0 + x1 * self.cos01
        m4y = y0 + x1 * self.cos02

        m1s = get_pythag_C(m1x, m1y)
        m2s = get_pythag_C(m2x, m2y)
        m3s = get_pythag_C(m3x, m3y)
        m4s = get_pythag_C(m4x, m4y)

        m1a = np.arctan2(m1y, m1x) / np.pi * 180
        m2a = np.arctan2(m1y, m1x) / np.pi * 180
        m3a = np.arctan2(m1y, m1x) / np.pi * 180
        m4a = np.arctan2(m1y, m1x) / np.pi * 180

        return [[m1s, m1a], [m2s, m2a], [m3s, m3a], [m4s, m4a]]
