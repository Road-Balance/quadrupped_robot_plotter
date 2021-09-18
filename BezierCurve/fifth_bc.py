import numpy as np
import matplotlib.pyplot as plt

class Point(object):

    def __init__(self, x_in=0, y_in=0):
        super().__init__()

        self.x = x_in
        self.y = y_in

    def __str__(self):
        return f"({self.x}, {self.y})"

def fifthBC_raw(t, p_0, p_1, p_2, p_3, p_4, p_5):
    output = Point(0, 0)

    output.x =  1  * (1 - t ) ** 5 * p_0.x + \
                5  * (1 - t) ** 4 * t * p_1.x + \
                10 * (1 - t) ** 3 * t ** 2 * p_2.x + \
                10 * (1 - t) ** 2 * t ** 3 * p_3.x + \
                5  * (1 - t) ** 1 * t ** 4 * p_4.x + \
                t ** 5 * p_5.x

    output.y =  1  * (1 - t ) ** 5 * p_0.y + \
                5  * (1 - t) ** 4 * t * p_1.y + \
                10 * (1 - t) ** 3 * t ** 2 * p_2.y + \
                10 * (1 - t) ** 2 * t ** 3 * p_3.y + \
                5  * (1 - t) ** 1 * t ** 4 * p_4.y + \
                t ** 5 * p_5.y

    return output

def factorial(num):
    output = 1

    if num == 0:
        return output
    elif num == 1:
        return output
    else: 
        output = output * num * factorial(num - 1)

    return output

def binomialFactor(n, i):
    return factorial(n) / (factorial(i) * factorial(n - i))

def generalBC(t, n, points):
    output = Point(0, 0)

    for i in range(len(points)):
        output.x += binomialFactor(n, i) * pow(t, i) * pow(1-t, n-i) * points[i].x
        output.y += binomialFactor(n, i) * pow(t, i) * pow(1-t, n-i) * points[i].y

    return output

num_points = 25
sample_points = np.linspace(0,1,num_points)

# Attaching 3D axis to the figure
# referenced from https://wikidocs.net/14604
fig = plt.figure()
ax = fig.subplots()

p_0 = Point(-5,  0)
p_1 = Point(-5, 2.5)
p_2 = Point(-5, 5)
p_3 = Point(5,  5)
p_4 = Point(5,  2.5)
p_5 = Point(5,  0)

points = [
    p_0,
    p_1,
    p_2,
    p_3,
    p_4,
    p_5,
    ]

bc_result = []

for t in sample_points:
    my_point = fifthBC_raw(t, p_0, p_1, p_2, p_3, p_4, p_5)
    bc_result.append(my_point)

# # make BC as general function
# for t in sample_points:
#     my_point = generalBC(t, 5, points)
#     bc_result.append(my_point)

for p in bc_result:
    ax.plot(p.x, p.y,'r-*',lw=1)

# # X and Y swap
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.axis([-6, 6, -1, 6])

plt.show()