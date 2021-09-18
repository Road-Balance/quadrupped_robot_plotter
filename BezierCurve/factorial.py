import matplotlib.pyplot as plt

from math import pow

class Point(object):

    def __init__(self, x_in=0, y_in=0):
        super().__init__()

        self.x = x_in
        self.y = y_in

    def __str__(self):
        return f"({self.x}, {self.y})"

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

# print(factorial(4))
# print(binomialFactor(6, 4))

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
    my_point = generalBC(t, 5, points)
    bc_result.append(my_point)

for p in bc_result:
    ax.plot(p.x, p.y,'r-*',lw=1)

# # X and Y swap
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.axis([-6, 6, -1, 6])

plt.show()