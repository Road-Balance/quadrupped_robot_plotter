import numpy as np
import matplotlib.pyplot as plt

from BezierCurve import BezierCurveMaker
from StanceCurve import StanceCurveMaker


class TrajMaker(object):

    def __init__(self, swing_len, points, stance_len, delta, L, z_offset):
        super().__init__()

        self.swing_len = swing_len
        self.points = points
        self.stance_len = stance_len
        self.delta = delta
        self.L = L

        self.z_offset = z_offset

        # Bezier Curve Maker
        self.bc_maker = BezierCurveMaker(len(points) - 1, points)
        # Stance Curve Maker    
        self.sc_maker = StanceCurveMaker(delta=self.delta, L=self.L)


    def calcBezierCurve(self, z_offset=0):

        sample_points = np.linspace(0, 1, self.swing_len)
        traj_points = []

        for t in sample_points:
            my_point = self.bc_maker.getBezierPoint(t)
            my_point[1] += self.z_offset
            traj_points.append(my_point)

        return traj_points 

    def calcStanceCurve(self, z_offset=0):

        sample_points = np.linspace(0, 1, self.stance_len)
        traj_points = []

        for t in sample_points:
            my_point = self.sc_maker.stanceCurve(t)
            my_point[1] += self.z_offset
            traj_points.append(my_point)

        return traj_points

    def getTrajPoints(self, z_offset=0):
        traj_points = []

        traj_points += self.calcBezierCurve(z_offset)
        traj_points += self.calcStanceCurve(z_offset)

        return traj_points

    def get3DTrajPoints(self, y_offset=0, z_offset=0, phase=1):
        traj_points = []
        bezier_traj_3d = []
        stance_traj_3d = []

        for bezier_point in self.calcBezierCurve(z_offset):
            bezier_traj_3d.append([bezier_point[0], y_offset, bezier_point[1]])

        for stance_point in self.calcStanceCurve(z_offset):
            stance_traj_3d.append([stance_point[0], y_offset, stance_point[1]])

        if phase == 1:
            traj_points += bezier_traj_3d
            traj_points += stance_traj_3d
        elif phase ==2:
            traj_points += stance_traj_3d
            traj_points += bezier_traj_3d

        return traj_points


if __name__=="__main__":
    points = [
            [-0.1,  0],
            [-0.1, 0.05],
            [-0.1, 0.1],
            [0.1,  0.1],
            [0.1,  0.05],
            [0.1,  0],
        ]


    num_sample = 25
    sample_points = np.linspace(0, 1, num_sample)

    # Attaching 3D axis to the figure
    # referenced from https://wikidocs.net/14604
    fig = plt.figure()
    ax = fig.subplots()

    traj_maker = TrajMaker(
        swing_len=25, points=points, 
        stance_len=25, delta=0.02, L=0.2,
        z_offset=-0
        )

    traj_points = traj_maker.getTrajPoints()

    for p in traj_points:
        ax.plot(p[0], p[1],'r-*',lw=1)

    # # X and Y swap
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.axis([-0.2, 0.2, -0.1, 0.2])

    plt.show()