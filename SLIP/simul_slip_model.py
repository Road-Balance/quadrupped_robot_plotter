import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.animation as animation

from math import pi, cos, sin
import csv

X1=[]
Y1=[]
X2=[]
Y2=[]

phi=[]
thLen=[]

d2r = pi/180
r2d = 180/pi

lenLeg = 0.75

def readXparams():
    file = open('X.csv', 'r')
    reader = csv.reader(file)

    for line in reader:
        x1 = float(line[0])
        y1 = float(line[1])
        _phi = float(line[2])
        _thLeg = float(line[3])

        x2 = x1 + lenLeg * sin(_thLeg)
        y2 = y1 - lenLeg * cos(_thLeg)

        X1.append(x1)
        Y1.append(y1)
        X2.append(x2)
        Y2.append(y2)
        phi.append(_phi)
        thLen.append(_thLeg)

    file.close()

def sethalfData(X1, X2, Y1, Y2, phi, thLen):
    X1 = X1[::2]
    Y1 = Y1[::2]
    X2 = X2[::2]
    Y2 = Y2[::2]
    phi = phi[::2]
    thLen = thLen[::2]

def plotBody(ax, x, y, lenLeg, thLeg, phi):
    _x, _y = [x, x + lenLeg*sin(thLeg)], [y, y - lenLeg*cos(thLeg)]
    plot = ax.plot(_x, _y, marker = 'o')
    return plot
    # plotEllipse(ax, x,y, 0.5, 0.25, phi)

def plotEllipse(ax, x_ctr, y_ctr, rx, ry, phi):
    t = np.linspace(0, 2*pi, 100)
    R = rotation(phi)
    point = np.array(
        [
            x_ctr + rx * np.cos(t),
            y_ctr + ry * np.sin(t)
        ]
    )

    rotated_point = R @ point
    # print(rotated_point.T)

    plot = ax.plot( rotated_point[0], rotated_point[1] )
    return plot


def rotation(theta):
    R = np.array(
        [
            [cos(theta),  -sin(theta)],
            [sin(theta),  cos(theta)]
        ]
    )
    return R

def init():
    body_plot.set_data([], [])
    ellipse_plot.set_data([], [])

def update_plots(i, X1, X2, Y1, Y2, phi, thLen, plots):
    x_points = [X1[i], X2[i]]
    y_points = [Y1[i], Y2[i]]

    body_plots, ellipse_plots = plots

    # # print(i)
    # # print(type(body_plots), type(body_plots[0]), body_plots)
    # # print(type(ellipse_plot), type(ellipse_plot[0]), ellipse_plot)

    t = np.linspace(0, 2*pi, 100)
    R = rotation(phi[i])
    point = np.array(
        [
            0.5 * np.cos(t),
            0.25 * np.sin(t)
        ]
    )
    rotated_point = R @ point
    rotated_point += np.array([[X1[i]], [Y1[i]]])

    # print(rotated_point)

    # body_plots[0].set_data(x_points, y_points)
    ellipse_plots[0].set_data(rotated_point[0], rotated_point[1])

    body_plot[0].set_data(x_points, y_points)

    return plots

if __name__ == "__main__":
    fig = plt.figure()

    ax = fig.subplots()
    readXparams()

    print(len(X1))

    # points  = plotBody(ax, 0, 0, lenLeg, 0,d2r*60)

    # # # X and Y swap
    # ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    ax.axis([-1, 1, 0, 2])

    # plt.show()

    body_plot = plotBody(ax, 0, 0, lenLeg, 0, 0)
    ellipse_plot = plotEllipse(ax, X1[0], Y1[0], 0.5, 0.25, phi[0])
    
    plots = []
    plots.append(body_plot)
    plots.append(ellipse_plot)

    line_ani = animation.FuncAnimation(
        fig,
        update_plots,
        # init_func=init,
        frames=len(X1[::2]),
        fargs=(X1[::2], X2[::2], Y1[::2], Y2[::2], phi, thLen, plots),
        interval=1,
        blit=False, # 이걸 False로 해야 body, ellipse를 따로 관리할 수 있다.
    )

    # line_ani.save('./animation.gif', writer='imagemagick', fps=60)
    # FFwriter = animation.FFMpegWriter(fps=1)
    # line_ani.save('animation.mp4', writer = FFwriter)

    # writergif = animation.PillowWriter(fps=30) 
    # line_ani.save('./animation.gif', writer=writergif)

    plt.show()
