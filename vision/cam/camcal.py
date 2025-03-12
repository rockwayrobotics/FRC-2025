import numpy as np

class CamCal:
    def __init__(self, fx, fy, cx, cy):
        self._array = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], np.float64).reshape((3, 3))
        self._value = { "fx":fx, "fy":fy, "cx":cx, "cy": cy }
    def as_array(self):
        return self._array
    def as_dict(self):
        return self._value

CALS = {
    # images2 640x480
    '640x480': CamCal(fx=813.002665, fy=814.367913, cx=340.340811, cy=248.727651),

    # images4 1456x1088
    '1456x1088': CamCal(fx=1757.669488, fy=1762.782233, cx=736.690867, cy=557.428635),
}