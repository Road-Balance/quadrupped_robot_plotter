import numpy as np

from math import cos, pi

class StanceCurveMaker(object):

    def __init__(self, delta = 0.05, L=5):
        super().__init__()

        self.delta = delta
        self.L = L

    """
    t = 0 will be (L/2, 0), and
    t = 1 will be (-L/2, 0)

    And middle point will be (0, -delta)
    """
        
    def stanceCurve(self, t): 
        output = np.array([0.0, 0.0])

        output[0] = self.L * (0.5 - t)
        output[1] = -self.delta * cos(pi * (0.5 - t))

        return output