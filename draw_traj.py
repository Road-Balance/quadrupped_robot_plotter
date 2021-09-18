import numpy as np
import matplotlib.pyplot as plt

from BezierCurve import BezierCurveMaker
from StanceCurve import StanceCurveMaker


class TrajMaker(object):

    def __init__(self, swing_len, points, stance_len, delta, L):
        super().__init__()

        self.swing_len = swing_len
        self.points = points
        self.stance_len = stance_len
        self.delta = delta
        self.L = L

        # Bezier Curve Maker
        self.bc_maker = BezierCurveMaker(len(points) - 1, points)
        # Stance Curve Maker    
        self.sc_maker = StanceCurveMaker(delta=self.delta, L=self.L)


    def calcBezierCurve(self):

        sample_points = np.linspace(0, 1, self.swing_len)
        traj_points = []

        for t in sample_points:
            my_point = self.bc_maker.getBezierPoint(t)
            traj_points.append(my_point)

        return traj_points 

    def calcStanceCurve(self):

        sample_points = np.linspace(0, 1, self.stance_len)
        traj_points = []

        for t in sample_points:
            my_point = self.sc_maker.stanceCurve(t)
            traj_points.append(my_point)

        return traj_points

    def getTrajPoints(self):
        traj_points = []

        traj_points += self.calcBezierCurve()
        traj_points += self.calcStanceCurve()

        return traj_points

points = [
        [-5,  0],
        [-5, 2.5],
        [-5, 5],
        [5,  5],
        [5,  2.5],
        [5,  0],
    ]


num_sample = 25
sample_points = np.linspace(0, 1, num_sample)

# Attaching 3D axis to the figure
# referenced from https://wikidocs.net/14604
fig = plt.figure()
ax = fig.subplots()

traj_maker = TrajMaker(
    swing_len=25, points=points, 
    stance_len=25, delta=0.5, L=10
    )

traj_points = traj_maker.getTrajPoints()

for p in traj_points:
    ax.plot(p[0], p[1],'r-*',lw=1)

# # X and Y swap
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.axis([-6, 6, -1, 6])

plt.show()