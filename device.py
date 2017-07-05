# coding: utf-8 

import sigsim

class CmdVel :
    """

       This class is a device that controls a position from a speed
       command (so called cmd_vel in ROS). The actual speed is actually
       obtained from the command with a time constant. Moreover, the position
       of the device can be accessed through a sensor which is delayed.

    """
    def __init__(self):
        self.cmd_vel = None

        # dv = ddpos = a*(cmd_vel - v)
        self.speed_gain = 3
        self.position   = sigsim.Computed(lambda me : self.speed_gain * (self.cmd_vel[0] - me[1]), 2, 2)

        # position_sensor = position(t-delay)
        self.set_delay(1)

    def set_delay(self, delay) :
        self.position_sensor_delay = delay
        self.position_sensor = sigsim.Delayed(self.position, self.position_sensor_delay)

    def clear(self):
        self.position.clear()
        self.position_sensor.clear()
        
    def next(self, dt):
        """
           
           cmd_vel has to be updated externally beforehand.

        """
        self.position.next(dt)
        self.position_sensor.next(dt)

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import numpy as np
    
    
    device = CmdVel()
    cmd_vel = sigsim.Forced(lambda t : float(t > 1.0) - .5*float(t > 5.0), 0, 0)
    device.cmd_vel = cmd_vel

    dt = 0.01
    X = np.arange(0,10,dt)
    COMMAND  = [cmd_vel[0]]
    POSITION_SENSOR   = [.1*device.position_sensor[0]] # scale factor for plotting
    POSITION   = [.1*device.position[0]] # scale factor for plotting
    SPEED    = [device.position[1]]
    for x in X[1:]:
        cmd_vel.next(dt)
        device.next(dt)
        COMMAND.append(device.cmd_vel[0])
        POSITION_SENSOR.append(.1*device.position_sensor[0]) # scale factor for plotting
        POSITION.append(.1*device.position[0])               # scale factor for plotting
        SPEED.append(device.position[1])

    plt.figure()
    plt.ylim(0,2)
    plt.plot(X, COMMAND,  '-', label = 'cmd_vel' )
    plt.plot(X, SPEED,    '-', label = 'speed'   )
    plt.plot(X, POSITION_SENSOR,   '-', label = 'position_sensor')
    plt.plot(X, POSITION, '--', label = 'position')
    plt.legend()
    plt.show()

    
    
    
    


