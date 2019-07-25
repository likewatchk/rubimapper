"""Initialize an ellipsoid"""
from math import *
import numpy as np
import sys


class Geoid:

    def __init__(self, a, f):
        """
        Parameters
        ----------
        a : float, radius in meters
        f : float, flattening
        """
        self.a = a
        self.f = f
        self.e2 = f * (2 - f)
        self.e2m = (1 - f)**2
        self.e2a = abs(self.e2)
        self.e4a = self.e2**2
        self.max_rad = 2. * self.a / sys.float_info.epsilon

wgs84_a = 6378137
wgs84_f = 1 / (np.uint64(298257223563) / 1000000000)
wgs84_geoid = Geoid(wgs84_a, wgs84_f)
