import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

pred = [3.6914e-3, 1.5555e-2, 3.1638e-2]
actu = [0.0756351, 0.1041822, 0.1364571]

slope = 2.166615229199018
yintr = 0.0686758280104485

for i in range(3):
    fix = slope * pred[i] + yintr
    print(fix)
    print(actu[i])