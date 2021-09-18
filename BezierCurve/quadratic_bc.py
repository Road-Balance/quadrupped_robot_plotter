import numpy as np
import matplotlib.pyplot as plt

class Point(object):

    def __init__(self, x_in=0, y_in=0):
        super().__init__()

        self.x = x_in
        self.y = y_in

    def __str__(self):
        return f"({self.x}, {self.y})"

def quadraticBC(t, p_0, p_1, p_2):
    output = Point(0, 0)

    output.x = (1 - t ) ** 2 * p_0.x + \
                2 * (1 - t) * t * p_1.x + \
                t ** 2 * p_2.x

    output.y = (1 - t ) ** 2 * p_0.y + \
                2 * (1 - t) * t * p_1.y + \
                t ** 2 * p_2.y

    return output

num_points = 25
sample_points = np.linspace(0,1,num_points)

# Attaching 3D axis to the figure
# referenced from https://wikidocs.net/14604
fig = plt.figure()
ax = fig.subplots()

p_0 = Point(0, 10)
p_1 = Point(0,  0)
p_2 = Point(10, 0)

bc_result = []

for t in sample_points:
    my_point = quadraticBC(t, p_0, p_1, p_2)
    bc_result.append(my_point)

for p in bc_result:
    ax.plot(p.x, p.y,'r-*',lw=1)

# # X and Y swap
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.axis([-1, 11, -1, 11])

plt.show()