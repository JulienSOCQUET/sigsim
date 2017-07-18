# coding: utf-8

"""

   This module provide signal object handled by an Euler
   avaluation. You can set a value for a signal, and then process
   next(dt) do make the signal evolve. The values to be set can be the
   signal value itself, but also its derivatives (of any order). When
   setting a value at a specific order of derivation, the other values
   are computed thanks to an Euler update when next(dt) is called.

"""

import numpy as np

class Signal:
    """

       This is the base class for signals.

    """
    def __init__(self, order) :
        """

          order is the number of derivatives handeled by the signal
          object. For example, order=2 means that the signal will
          compute its value (order 0), its first derivative and its
          second derivative. These are respectively self.value[0],
          self.value[1] and self.value[2]

        """
        self.order   = order
        self.val_size = order+1
        self.value   = np.zeros(self.val_size, dtype=np.float64)
        
    def __setitem__(self, key, item): 
        """

           set the ith derivative. Use this at initialization. Then,
           you may rather use the set method.

        """
        self.value[key] = item
        
    def __getitem__(self, ith):
        """

           get the ith derivative

        """
        return self.value[ith]
        

    def set(self, val, ith, dt):
        """

           set the ith derivative to val and process an Euler update with dt

        """
        new = np.zeros(self.val_size, dtype=np.float64)
        new[ith] = val
        for o in range(ith+1, self.val_size):
            new[o] = (new[o-1]-self.value[o-1])/float(dt)
        for o in range(ith-1,-1,-1):
            new[o] = self.value[o]+dt*self.value[o+1]
        self.value = new
        return self.value

    def clear(self) :
        self.value   = np.zeros(self.val_size, dtype=np.float64)

class Forced(Signal):
    """

       The ith derivative value of the signal at each time is computed
       thanks to some function. This function takes time as input and
       computes a scalar (val = f(t)).

    """
    def __init__(self, f, ith, order):
        Signal.__init__(self, order)
        self.fun = f
        self.t   = 0
        self.ith = ith

    def next(self, dt):
        self.t += dt
        return self.set(self.fun(self.t), self.ith, dt)
    
class Computed(Signal):
    """

       The ith derivative value of the signal at each time is computed by
       calling the compute method. This method is called with the
       current self.value signal values (val = compute(self.value)).

    """
    def __init__(self, compute, ith, order):
        Signal.__init__(self, order)
        self.compute = compute
        self.ith = ith

    def next(self, dt):
        """
           Performs an Euler step with dt
        """
        return self.set(self.compute(self.value), self.ith, dt)

class Smoothed(Signal):
    """

       Interpolates the last N samples with a polynom of degre D. The
       derivates are computed from the interpolated polynom, on order
       to avoid noisy derivatives (i.e. Savitzky-Golay algorithm).

       The value is computed by calling the compute method. This method is
       called with the current self.value signal values (val =
       compute(self.value)).

    """
    def __init__(self, compute, order, degree, time_buf_length):
        Signal.__init__(self, order)
        self.compute = compute
        self.tbuffer = np.array([])
        self.vbuffer = np.array([])
        self.degree = degree
        self.time_buf_length = time_buf_length

    def clear(self):
        self.value   = np.zeros(self.val_size, dtype=np.float64)
        self.tbuffer = np.array([])
        self.vbuffer = np.array([])

    def next(self, dt):
        """
          Acquire a new sample at last+dt

        """
        val = self.compute(self.value)
        self.tbuffer = np.insert(self.tbuffer,0,dt)
        self.vbuffer = np.insert(self.vbuffer,0,val)
        buf_length = np.sum(self.tbuffer)
        if buf_length < self.time_buf_length :
            self.value    = np.zeros(self.val_size, dtype=np.float64)
            self.value[0] = val
        else :
            buf_length = 0
            nb_samples = 0
            while buf_length < self.time_buf_length :
                buf_length += self.tbuffer[nb_samples]
                nb_samples += 1
            if nb_samples < len(self.tbuffer) :
                self.tbuffer = self.tbuffer[:nb_samples]
                self.vbuffer = self.vbuffer[:nb_samples]
            v0 = val
            tt = 0
            t = []
            for dt in self.tbuffer  :
                t.append(tt)
                tt -= dt
            t = np.array(t) 
            v = np.array(self.vbuffer)-v0
            T = np.ones(nb_samples)
            cur = T
            for d in range(1,self.degree+1) :
                cur = cur * t
                T = np.vstack([T,cur])
            T = T.T
            p = np.linalg.lstsq(T,v)[0]
            self.value = np.zeros(self.val_size, dtype=np.float64)
            bound = min(self.degree, self.order)
            self.value[0] = val+p[0]
            fact = 1
            for o in range(1,bound+1) :
                fact *= o
                self.value[o] = p[o]*fact

            #print('##########')
            #print('t={}'.format(t.tolist()))
            #print('v={}'.format(v.tolist()))
            #print('p={}'.format(p.tolist()))

                
            


        
class Delayed(Signal):
    """
       This signal correspond to another signal with a time delay.
    """
    def __init__(self, signal, delay):
        Signal.__init__(self, signal.order)
        self.delay = delay
        self.buffer = []
        self.signal = signal

    def clear(self):
        self.buffer = []
        # I should use super().... but it does not work.
        self.value  = np.zeros(self.val_size, dtype=np.float64)

    def next(self, dt):
        """

           Performs an Euler step with dt

        """
        if self.delay == 0 :
            self.buffer = []
            self.value = np.copy(self.signal.value)
            return self.value
        self.buffer.insert(0,(dt, np.copy(self.signal.value)))
        i = 0;
        t = 0;
        t_prev = 0
        while t<self.delay and i < len(self.buffer) :
            t_prev = t
            t += self.buffer[i][0]
            i += 1
        if i is len(self.buffer):
            self.value = np.zeros(self.val_size, dtype=np.float64)
            return self.value
        i -= 1
        if i > 0 : 
            val_prev = self.buffer[i-1][1]
        else :
            val_prev = np.zeros(self.val_size, dtype=np.float64)
        val = self.buffer[i][1]
        self.buffer = self.buffer[:i+1]
        c = (self.delay - t_prev)/self.buffer[i][0]
        self.value = val_prev + c*(val-val_prev)
        return self.value
            
        
            
        

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import math
    
    # force f (i.e ith=0), compute f and f' (i.e order=1)
    f = Forced(math.sin, 0, 1)

    # g is max(f,f'), compute g and g'
    g = Computed(lambda me : max(f[0], f[1]), 0, 1)

    # h(t) = g(t-delay)
    delay = math.sqrt(2) # silly...
    h = Delayed(g, delay)

    # a satisfies a''=-.2 (i.e. ith = 2), compute a, a', a'' (i.e. order = 2)
    a = Computed(lambda me : -.1, 2, 2) 
    a[0] = .2 # a  at start
    a[1] = .3 # a' at start

    dt = 0.01
    t = 0
    Y0 = [g[0]]
    Y1 = [.25*g[1]]
    Y2 = [.25*h[1]]
    Y3 = [a[0]]
    X = np.arange(0,20,dt)
    for x in X[1:]:
        f.next(dt)
        g.next(dt)
        h.next(dt)
        a.next(dt)
        Y0.append(    g[0]) # plot g = max(f,f')
        Y1.append(.25*g[1]) # rescale g' for plotting
        Y2.append(.25*h[1]) # rescale h' = g'(t-delay) for plotting
        Y3.append(    a[0])
    plt.figure()
    plt.ylim(-1,1)
    plt.plot(X, Y0, '-', label = "g" ) 
    plt.plot(X, Y1, '-', label = "g'")
    plt.plot(X, Y2, '-', label = "h" )
    plt.plot(X, Y3, '-', label = "a" )
    plt.legend()
    plt.show()

    

        

