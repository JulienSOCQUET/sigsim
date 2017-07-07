# coding: utf-8 

import numpy as np
import sigsim
import random

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import math

    noise_ampl = .5
    deg = 3
    size = 1000
    bound = 30
    window_duration = 5
    i = 0

    t = np.array(sorted([random.uniform(0,bound) for i in range(size)]))
    y = np.cos(t) + noise_ampl*(2*np.random.rand(size)-1)

    Y0 = []
    Y1 = []
    Y2 = []

    signal = sigsim.Smoothed(lambda me : y[i], 2, deg, window_duration)
    
    for i in range(1,len(t)) :
        dt = t[i] - t[i-1]
        signal.next(dt)
        Y0.append(signal.values[0])
        Y1.append(signal.values[1])
        Y2.append(signal.values[2])
    

    plt.figure()
    plt.scatter(t[1:],y[1:], alpha=.5)
    plt.plot(t[1:],Y0, label="Y")
    plt.plot(t[1:],Y1, label="Y'")
    plt.plot(t[1:],Y2, label="Y''")
    plt.legend()
    plt.show()
    
