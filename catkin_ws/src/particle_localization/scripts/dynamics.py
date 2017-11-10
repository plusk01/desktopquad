from __future__ import division
import numpy as np

# I should probably have some inheritance stuff going on here
class ConstVel(object):
    """
    Class to implement const velocity motion model with acc/gyro measurements
    Note: This class is intended to be implemented with particle-based approaches,
    thus, all state-estimates will be maintained outside of the class
    """

    def __init__(self, Q_pos, Q_att):
        """
        Inputs:
        M_acc - 3x3 np array input noise
        M_gyro - 3x3 np array input noise
        """
        self.Q_pos = Q_pos
        self.Q_att = Q_att

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
        k1 = self.dynamics(X, u, dt)
        k2 = self.dynamics(X + dt*k1/2, u, dt) 
        k3 = self.dynamics(X + dt*k2/2, u, dt)
        k4 = self.dynamics(X + dt*k3, u, dt)

        Xp = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        return Xp

    def dynamics(self, X, u, dt):
        """
        Calculate the derivative of the state
        Inputs:
        X - nxnum_particles estimates of state
        u - kx1 input (acc/gyro)

        Outputs:
        Xdot - derivative of the states
        """
        g = 9.8
        acc = u[:3]
        gyro = u[3:]

        sh = X.shape
        cphi = np.cos(X[6])
        sphi = np.sin(X[6])
        cth = np.cos(X[7])
        sth = np.cos(X[7])
        cpsi = np.cos(X[8])
        spsi = np.cos(X[8])

        u = X[3]
        v = X[4]
        w = X[5]

        p = X[9]
        q = X[10]
        r = X[11]

        A = np.zeros((3,3,sh[1]))
        A[0,0] = cth*cpsi
        A[0,1] = sphi*sth*cpsi - cphi*spsi
        A[0,2] = cphi*sth*cpsi + sphi*spsi
        A[1,0] = cth*spsi
        A[1,1] = sphi*sth*spsi + cth*cpsi
        A[1,2] = cphi*sth*spsi - sphi*cpsi
        A[2,0] = -sth
        A[2,1] = sphi*cth
        A[2,2] = cphi*cth

        B = np.zeros((3,3,sh[1]))
        B[0,0] = 1
        B[0,1] = sphi*sth/cth
        B[0,2] = cphi*sth/cth
        B[1,1] = cphi
        B[1,2] = -sphi
        B[2,1] = sphi/cth
        B[2,2] = cphi/cth

        mu = np.zeros(3)
        Xdot = np.zeros(sh)
        Xdot[:3] = np.einsum('ijk,jk->ik',A,X[3:6])
        Xdot[3] = acc[0] + q*w - r*v + g*sth
        Xdot[4] = acc[1] + r*u - p*w - g*cth*sphi
        Xdot[5] = acc[2] + p*v - q*u - g*cth*cphi
        Xdot[6:9] = np.einsum('ijk,jk->ik',B,X[9:12])
        Xdot[9:] = (gyro)[:,np.newaxis]

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
