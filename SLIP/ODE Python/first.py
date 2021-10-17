import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def firstorder(y, t, K, u):
    tau = 5.0 
    dydt = (-y + K * u ) / tau
    return dydt

t = np.linspace(0, 10, 11)
K = 2.0
u = np.zeros(len(t))
u[3:] = 1.0
y0 = 0

for i in range(len(t) - 1):
    ts = [t[i], t[i+1]]
    y = odeint(firstorder, y0, ts, args=(K,u[i]))
    y0 = y[1]
    print(y[1])

plt.plot(t, y)
plt.show()