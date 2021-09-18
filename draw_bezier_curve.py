import numpy as np
import matplotlib.pyplot as plt

from BezierCurve import BezierCurveMaker

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


bc_maker = BezierCurveMaker(5, points)

bc_result = []
for t in sample_points:
    my_point = bc_maker.getBezierPoint(t)
    bc_result.append(my_point)

for p in bc_result:
    print(p)
    ax.plot(p[0], p[1],'r-*',lw=1)

# # X and Y swap
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.axis([-6, 6, -1, 6])

plt.show()