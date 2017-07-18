# coding: utf-8 

import numpy as np
import sigsim
import random

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import math

    noise_ampl = .05
    deg = 4
    size = 800
    bound = 30
    window_duration = 2
    i = 0

    t = np.array(sorted([random.uniform(0,bound) for i in range(size)]))
    # t = np.linspace(0,bound,size) # for regular sampling
    y = np.cos(t) + noise_ampl*(2*np.random.rand(size)-1)

    Y0 = []
    Y1 = []
    Y2 = []

    signal = sigsim.Smoothed(lambda me : y[i], 2, deg, window_duration)
    
    for i in range(1,len(t)) :
        dt = t[i] - t[i-1]
        signal.next(dt)
        Y0.append(signal[0])
        Y1.append(signal[1])
        Y2.append(signal[2])
    

    plt.figure()
    plt.scatter(t[1:],y[1:], alpha=.5)
    plt.plot(t[1:],Y0, label="Y")
    plt.plot(t[1:],Y1, label="Y'")
    # plt.plot(t[1:],Y2, label="Y''")  # very noisy... increase deg
    plt.legend()
    plt.show()
    
