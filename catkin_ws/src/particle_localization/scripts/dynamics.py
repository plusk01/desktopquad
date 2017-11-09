from __future__ import division
import numpy as np

# I should probably have some inheritance stuff going on here
class ConstVel(object):
    """
    Class to implement const velocity motion model with acc/gyro measurements
    Note: This class is intended to be implemented with particle-based approaches,
    thus, all state-estimates will be maintained outside of the class
    """

    def __init__(self, M_acc, M_gyro):
        """
        Inputs:
        M_acc - 3x3 np array input noise
        M_gyro - 3x3 np array input noise
        """
        self.M_acc = M_acc
        self.M_gyro = M_gyro

    def propogate(self, X, u, dt):
        """
        Propogates the dynamics using RK4 integration
        Note: if this is too slow, we can replace with euler integration

        Inputs:
        X - nxnum_particles estimates of state
        u - kx1 input (acc/gyro)
        dt - time interval to integrate over

        Outputs:
        Xp - predictions  
        """
        k1 = self.dynamics(X, u)
        k2 = self.dynamics(X + dt*k1/2, u) 
        k3 = self.dynamics(X + dt*k2/2, u)
        k4 = self.dynamics(X + dt*k3, u)

        Xp = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        return Xp

    def dynamics(self, X, u):
        """
        Calculate the derivative of the state
        Inputs:
        X - nxnum_particles estimates of state
        u - kx1 input (acc/gyro)

        Outputs:
        Xdot - derivative of the states
        """
        sh = X.shape[0]
        acc = u[:3]
        gyro = u[3:]
        Xdot = np.zeros(sh)
        Xdot[:3] = X[6:9]
        Xdot[3:6] = gyro + np.random.multivariate_normal(mean=mu, cov=self.M_gyro)

        mu = np.zeros(3)
        # Does it make sense to add in noise during the integration?
        # Or should this be added on in propogate?
        Xdot[6:9] = acc + np.random.multivariate_normal(mean=mu, cov=self.M_acc) 

        return Xdot


class QuadRotor(object):
    """
    Class to implement quadrotor dynamics
    Note: This class is intended to be implemented with particle-based approaches,
    thus, all state-estimates will be maintained outside of the class
    """
    def __init__(self, M):
        """
        Inputs:
        M_acc - 3x3 np array input noise
        M_gyro - 3x3 np array input noise
        """
        self.M_acc = M_acc
        self.M_gyro = M_gyro

    def propogate(self, X, u, dt):
        """
        Propogates the dynamics using RK4 integration
        Note: if this is too slow, we can replace with euler

        Inputs:
        X - nxnum_particles estimates of state
        u - kx1 input (acc/gyro)
        dt - time interval to integrate over

        Outputs:
        Xp - predictions  
        """
        k1 = self.dynamics(X, u)
        k2 = self.dynamics(X + dt*k1/2, u) 
        k3 = self.dynamics(X + dt*k2/2, u)
        k4 = self.dynamics(X + dt*k3, u)
        Xp = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        return Xp

    def dynamics(self, X, u):
        pass
