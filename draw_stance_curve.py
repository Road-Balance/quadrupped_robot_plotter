import numpy as np
import matplotlib.pyplot as plt

from StanceCurve import StanceCurveMaker

num_points = 25
sample_points = np.linspace(0,1,num_points)

# Attaching 3D axis to the figure
# referenced from https://wikidocs.net/14604
fig = plt.figure()
ax = fig.subplots()

delta = 0.1
L = 5
sc_maker = StanceCurveMaker(delta=delta)

# stance curve result
sc_result = []
for t in sample_points:
    my_point = sc_maker.stanceCurve(t)
    sc_result.append(my_point)

# for point in sc_result: 
#     print(sc_result)

for p in sc_result:
    ax.plot(p[0], p[1],'r-*',lw=1)

# # X and Y swap
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.axis([-6, 6, -1, 1])

plt.show()