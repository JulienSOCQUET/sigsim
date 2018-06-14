# coding: utf-8 

import sigsim
import numpy as np

class Drone :

    '''This class works for first-order model and squarred first-model order that can be integrated as many times as you want'''

    def __init__(self, model_order, integrating_number):
        self.K0            = None
        self.T0            = None
        self.dt            = None
        self.input_signal  = None
        self.integrating_number = integrating_number

        if model_order == 1:
            self.middle_signal = sigsim.Computed(lambda me: self.input_signal[0], 0, 0)
        elif model_order == 2:
            #self.middle_signal = sigsim.Computed(lambda me : self.K0/self.T0*self.input_signal[0]-1/self.T0*me[0], 1, 1)    # explicit scheme
            self.middle_signal = sigsim.Computed(lambda me : (me[0]+self.dt*self.K0/self.T0*self.input_signal[0])/(1+self.dt/self.T0), 0, 0)    # implicit scheme
            
        
        #self.output_signal = sigsim.Computed(lambda me : self.K0/self.T0*self.middle_signal[0]-1/self.T0*me[integrating_number], integrating_number+1, integrating_number+1)    # explicit scheme
        self.output_signal = sigsim.Computed(lambda me : (me[integrating_number]+self.dt*self.K0/self.T0*self.middle_signal[0])/(1+self.dt/self.T0), integrating_number, integrating_number)    # implicit scheme

    def clear(self):
        self.middle_signal.clear()
        self.output_signal.clear()

    def next(self, dt):
        self.dt             = dt
        last_output         = self.output_signal.value
        last_output_delayed = self.output_signal.value
        self.middle_signal.next(dt)
        self.output_signal.next(dt)
        self.output_signal_delayed.next(dt)

        ### Remove outliers ###

        if ((self.integrating_number == 2) and (abs(last_output[0]-last_output_delayed[0]+self.output_signal_delayed[0]-self.output_signal[0]) > 0.4)):
            self.output_signal.value         = last_output
            self.output_signal_delayed.value = last_output_delayed


            
    def set_delay(self, delay) :
        self.output_signal_delay = delay
        self.output_signal_delayed = sigsim.Delayed(self.output_signal, self.output_signal_delay)

class Regulator :

    def __init__(self):
        self.K                     = None
        self.Ti                    = None
        self.Td                    = None
        self.input_signal          = None
        self.formated_input_signal = sigsim.Smoothed(lambda me : self.input_signal[0], 1, 2, 4, 2)
        #self.formated_input_signal = sigsim.Computed(lambda me : self.input_signal[0], 1, 2)
        self.output_signal         = sigsim.Computed(lambda me : self.K*(self.formated_input_signal[1]+self.formated_input_signal[0]/self.Ti+self.formated_input_signal[2]*self.Td), 0, 0)

    def clear(self):
        self.output_signal.clear()

    def next(self, dt):
        self.formated_input_signal.next(dt)
        self.output_signal.next(dt)


class SmithPredictorCmdVel :

    def __init__(self, model_order, integrating_number, max_cmd):
        self.input_signal                  = None
        self.max_cmd                       = max_cmd
        self.drone_simulation              = Drone(model_order, integrating_number)
        self.regulator                     = Regulator()
        self.drone_simulation.input_signal = self.regulator.output_signal
        self.smith_error                   = sigsim.Computed(lambda me : self.input_signal[0]-self.drone_simulation.output_signal[0]+self.drone_simulation.output_signal_delayed[0], 0, 0)
        self.regulator.input_signal        = self.smith_error

    def set_delay(self, delay) :
        self.drone_simulation.set_delay(delay)

    def clear(self):
        self.drone_simulation.clear()
        self.regulator.clear()
        
    def next(self, dt):
        """
           
           input_signal has to be updated externally beforehand.

        """
        self.smith_error.next(dt)
        self.regulator.next(dt)
        if self.regulator.output_signal[0] > self.max_cmd:
            self.regulator.output_signal[0] = self.max_cmd
        elif self.regulator.output_signal[0] < -self.max_cmd:
            self.regulator.output_signal[0] = -self.max_cmd
        self.drone_simulation.next(dt)





        
if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import numpy             as np
    from   random            import random

    axis = input("axis = 'linX', 'linY', 'linZ' or 'angZ'\n")
    
    ### Parameters ###

    if axis == 'linX':
    
        tau         = 0.10
        K0          = 2.7
        T0          = 0.10
        K           = 0.04 #0.2  #1.6119
        Ti          = 4.0 #6.0  #0.6580
        Td          = 6.0 #0.6  #0.1645
        dt          = 0.01
        model_order = 2
        integrating_number = 2

    elif axis == 'linY':
    
        tau         = 0.10
        K0          = 2.7
        T0          = 0.10
        K           = 0.15 #0.2  #1.6119
        Ti          = 20.0 #6.0  #0.6580
        Td          = 1.0 #0.6  #0.1645
        dt          = 0.01
        model_order = 2
        integrating_number = 2

    elif axis == 'linZ':
        tau                = 0.15
        K0                 = 1.0
        T0                 = 0.08
        K                  = 1.6 #11.14/20 #8.357/7
        Ti                 = 4.0 #0.0467   #0.077
        Td                 = 0.0 #0.0117   #0.0
        dt                 = 0.01
        model_order        = 2
        integrating_number = 1

    elif axis == 'angZ':
        tau                = 0.20
        K0                 = 0.35
        T0                 = 2.55
        K                  = 3.0
        Ti                 = 20.0
        Td                 = 0.0
        dt                 = 0.01
        model_order        = 1
        integrating_number = 1

    ### Define the blocks ###

    noise_ampl = 0.05
    max_cmd    = 4
    noise      = noise_ampl*(2*random()-1)
    target     = sigsim.Forced(lambda t : float(t > 2.0) + noise, 0, 0)
    drone      = Drone(model_order, integrating_number)
    predictor  = SmithPredictorCmdVel(model_order, integrating_number, max_cmd)

    drone.K0                         = K0
    drone.T0                         = T0
    drone.dt                         = dt
    predictor.drone_simulation.K0    = K0
    predictor.drone_simulation.T0    = T0
    predictor.drone_simulation.dt    = dt
    predictor.regulator.K            = K
    predictor.regulator.Ti           = Ti
    predictor.regulator.Td           = Td
    
    drone.input_signal     = predictor.regulator.output_signal
    drone.set_delay(tau)
    predictor.set_delay(tau)
    #erreur                 = sigsim.Computed(lambda me : target[0]-drone.output_signal_delayed[0], 0, 0)
    erreur                 = sigsim.SignalSmoothed(lambda me : target[0] - drone.output_signal_delayed[0], 10)
    predictor.input_signal = erreur

    ### Calculate the signals ###
    
    X                   = np.arange(0,10,dt)
    TARGET              = [target[0]]
    POSITION_SIMULATION = [predictor.drone_simulation.output_signal[0]]
    POSITION            = [drone.output_signal_delayed[0]]
    SMITH_ERROR         = [predictor.smith_error[0]]
    ERROR               = [predictor.input_signal[0]]
    CMD                 = [predictor.regulator.output_signal[0]]
    
    for x in X[1:]:
        noise = noise_ampl*(2*random()-1)
        target.next(dt)
        erreur.next(dt)
        predictor.next(dt)
        drone.next(dt)
        POSITION_SIMULATION.append(predictor.drone_simulation.output_signal[0])
        TARGET.append(target[0])
        POSITION.append(drone.output_signal_delayed[0])
        ERROR.append(predictor.input_signal[0])
        SMITH_ERROR.append(predictor.smith_error[0])
        CMD.append(predictor.regulator.output_signal[0])

    ### Show the signals ###
                                             
    plt.figure()
    plt.ylim(-1,2)
    plt.plot(X, TARGET,              '-',  label = 'cmd_vel' )
    plt.plot(X, POSITION,            '--', label = 'position')
    plt.plot(X, POSITION_SIMULATION, '--', label = 'position_simulation')
    plt.plot(X, SMITH_ERROR,         '.',  label = 'smith_error')
    plt.plot(X, ERROR,               '.',  label = 'error')
    plt.plot(X, CMD,                 '+',  label = 'command')
    plt.legend()
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Value)')
    plt.show()

    
    
    
    


