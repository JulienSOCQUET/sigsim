# coding: utf-8 

import numpy as np
import sigsim
import device

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import math
    
    speed_scale = 1.0
    dev = device.CmdVel()


    # this is the positions we want to reach.
    target  = sigsim.Forced(lambda t : float(t > 1.0) - 2*float(t > 5.0) + (1 - max(math.cos(x),-.5))*float(t > 6.28), 0, 0)

    # this is the error signal. Switch comments to see the instability due to delays.
    error   = sigsim.Computed(lambda me : target.value[0] - dev.position.value[0], 0, 1)
    # error   = sigsim.Computed(lambda me : target.value[0] - dev.position_sensor.value[0], 0, 1)

    # Let us set up a corrector
    Kp = 5
    Ki = 1
    cmd_vel = sigsim.Computed(lambda me : Kp*error.value[0] + Ki*error.value[1], 0, 0)

    # this sets the command to the device.
    dev.cmd_vel = cmd_vel

    dt = 0.01
    X = np.arange(0,20,dt)
    COMMAND  = [speed_scale*cmd_vel.value[0]]
    SPEED    = [speed_scale*dev.position.value[1]]
    POSITION = [dev.position.value[0]]
    TARGET   = [target.value[0]]

    for x in X[1:]:
        target.next(dt)
        error.next(dt)
        cmd_vel.next(dt)
        dev.next(dt)
        COMMAND.append(speed_scale*cmd_vel.value[0])
        SPEED.append(speed_scale*dev.position.value[1])
        POSITION.append(dev.position.value[0])
        TARGET.append(target.value[0])
        
    plt.figure()
    plt.ylim(-2,2)
    # plt.plot(X, SPEED,    '-' , label = 'speed' )
    # plt.plot(X, COMMAND,  '--', label = 'cmd_vel' )
    plt.plot(X, POSITION, '-' , label = 'position')
    plt.plot(X, TARGET,   '--', label = 'target')
    plt.legend()
    plt.show()

