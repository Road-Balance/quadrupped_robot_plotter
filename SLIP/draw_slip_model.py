import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

from math import pi, cos, sin

d2r = pi/180
r2d = 180/pi

def plotBody(ax, x, y, lenLeg, thLeg, phi):
    _x, _y = [x, x + lenLeg*sin(thLeg)], [y, y - lenLeg*cos(thLeg)]
    ax.plot(_x, _y, marker = 'o')
    plotEllipse(ax, x,y, 0.5, 0.25, phi)

def rotation(theta):
    R = np.array(
        [
            [cos(theta), -sin(theta)],
            [sin(theta),  cos(theta)]
        ]
    )
    return R

def plotEllipse(ax, x_ctr,y_ctr,rx,ry,phi):
    t = np.linspace(0, 2*pi, 100)
    R = rotation(phi)
    point = np.array(
        [
            rx * np.cos(t),
            ry * np.sin(t)
        ]
    )

    rotated_point = R @ point
    rotated_point += [[x_ctr], [y_ctr]]
    print(rotated_point.T)

    ax.plot( rotated_point[0], rotated_point[1] )


if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.subplots()

    points  = plotBody(ax, 0,0,0.75,0,d2r*30)

    # # X and Y swap
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.axis([-1, 1, -1, 1])

    plt.show()