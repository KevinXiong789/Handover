
import math
import numpy as np



def get_alpha (rate=30, cutoff=1):
    tau = 1 / (2 * math.pi * cutoff)
    te = 1 / rate
    return 1 / (1 + tau / te)



class LowPassFilter:

    def __init__ (self):
        self.x_previous = None


    def __call__ (self, x: np.ndarray, alpha=0.5):
        if self.x_previous is None:
            self.x_previous = x
            return x

        x_filtered = alpha * x + (1 - alpha) * self.x_previous
        self.x_previous = x_filtered
        return x_filtered



class OneEuroFilter:
    '''Based on paper: https://hal.inria.fr/hal-00670496/document.
    
    If high speed lag is a problem, increase beta;
    if slow speed jitter is a problem, decrease fcmin.
    
    Start with beta=0.0 and mincutoff=1.0.'''

    def __init__ (self, freq=15, min_cutoff=1, beta=0.05, d_cutoff=1):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.filter_x =  LowPassFilter()
        self.filter_dx = LowPassFilter()
        self.x_previous = None
        self.dx = None
        self.dx_alpha = get_alpha(self.freq, self.d_cutoff)


    def __call__ (self, x: np.ndarray):
        self.dx = 0 if (self.dx is None) else ( (x - self.x_previous) * self.freq)

        dx_smoothed = self.filter_dx(self.dx, self.dx_alpha)
        cutoff = self.min_cutoff + self.beta * abs(dx_smoothed)
        x_filtered = self.filter_x(x, get_alpha(self.freq, cutoff) )

        self.x_previous = x

        return x_filtered



if __name__ == '__main__':
    filter = OneEuroFilter(freq=15, beta=0.1)

    for val in range(10):
        x = val + (-1) ** (val % 2)
        x_filtered = filter(x)
        print(x_filtered, x)
