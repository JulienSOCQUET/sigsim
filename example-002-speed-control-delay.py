# coding: utf-8 

import numpy as np
import sigsim
import device

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import math
    
    speed_scale = 1.0 # for plotting speeds and positions on the same graph.

    # This stands for the real device we have to control.
    dev = device.CmdVel()

    # This is a model of the real device. As it is a model, we are
    # allowed for measuring undelayed poisition. Let us make the model
    # approximative by changing slightly the parameters.
    model = device.CmdVel()
    model.speed_gain            = dev.speed_gain            * 0.9
    model.position_sensor_delay = dev.position_sensor_delay * 1.1

    # This is the positions we want to reach.
    target = sigsim.Forced(lambda t : float(t > 1.0) - 2*float(t > 5.0) + (1 - max(math.cos(x),-.5))*float(t > 6.28), 0, 0)

    # This is the error signal, comparing the target to the real model position sensor.
    error = sigsim.Computed(lambda me : target[0] - dev.position_sensor[0], 0, 1)

    smith_error = sigsim.Computed(lambda me : error[0] - (model.position[0] - model.position_sensor[0]), 0, 1)

    # Let us the same corrector as example-001. It has been designed
    # from the undelayed position, but here, the error is smith_error.
    Kp = 5
    Kd = 1
    cmd_vel = sigsim.Computed(lambda me : Kp*smith_error[0] + Kd*smith_error[1], 0, 0)

    # this sets the command to the devices (real and simulated).
    dev.cmd_vel = cmd_vel
    model.cmd_vel = cmd_vel

    dt = 0.01
    X = np.arange(0,20,dt)
    COMMAND  = [speed_scale*cmd_vel[0]]
    SPEED    = [speed_scale*dev.position[1]]
    POSITION = [dev.position[0]]
    TARGET   = [target[0]]

    for x in X[1:]:
        target.next(dt)
        error.next(dt)
        smith_error.next(dt)
        cmd_vel.next(dt)
        dev.next(dt)
        model.next(dt)
        COMMAND.append(speed_scale*cmd_vel[0])
        SPEED.append(speed_scale*dev.position[1])
        POSITION.append(dev.position[0])
        TARGET.append(target[0])
        
    plt.figure()
    plt.ylim(-2,2)
    # plt.plot(X, SPEED,    '-' , label = 'speed' )
    # plt.plot(X, COMMAND,  '--', label = 'cmd_vel' )
    plt.plot(X, POSITION, '-' , label = 'position')
    plt.plot(X, TARGET,   '--', label = 'target')
    plt.legend()
    plt.show()

