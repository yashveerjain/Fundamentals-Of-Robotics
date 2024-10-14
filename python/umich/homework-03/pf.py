import numpy as np
from numpy.random import randn, rand
from scipy.stats import multivariate_normal

class PF:
    def __init__(self, N, Q, process_func):
        
        """
        Constructor for PF class

        Parameters
        ----------
        N : int
            number of particles
        Q : ndarray
            input noise covariance
        process_func : function
            process model

        Attributes
        ----------
        N : int
            number of particles
        x : ndarray
            particles state
        w : ndarray
            particles weight
        f : function
            process model
        Q : ndarray
            input noise covariance
        """
        self.N = N
        x = np.ones((N, 3))
        self.x = x #x + np.random.multivariate_normal(np.zeros(3), Q, size=N)
        self.w = np.ones(N) / N

        self.f = process_func
        self.Q = Q

    def sample_motion(self):
        """
        Sample motion model, get next state

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
       
        x = self.f(self.x, self.Q)
        self.x = x

    def importance_sampling(self, z, h_func, R):
        """
        Importance sampling, update weight, and resample if necessary,  
        Assumption : measurment noise is gaussian distribution 

        Parameters
        ----------
        z : ndarray
            measurement
        h_func : function
            measurement model
        R : ndarray
            measurement noise covariance

        Returns
        -------
        None
        """

        for i in range(self.N):
            w = multivariate_normal.pdf(z, h_func(self.x[i]), R)
            self.w[i] *= w

        self.w = self.w / np.sum(self.w)
        n_eff = 1 / np.sum(np.power(self.w, 2))

        if n_eff < self.N:
            self.resample()
        return self.x

    def resample(self):
        # low variance resampling
        """
        Resample the particles using low variance resampling.

        Note that this is an in-place resampling, i.e. the particles and weights
        are updated directly.

        Returns
        -------
        None
        """
        W = np.cumsum(self.w)
        r = rand(1) / self.N
        # r = 0.5 / self.n
        j = 1
        for i in range(self.N):
            u = r + (i - 1) / self.N
            while u > W[j]:
                j = j + 1
            self.x[i, :] = self.x[j, :]
            self.w[i] = 1 / self.N


