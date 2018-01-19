import numpy as np
import param as P
import time

class ballBeamDynamics:

    def __init__(self,ctrl=None):

        # Initial state conditions
        self.state = np.matrix([[P.z0],         # z initial position
                                [P.theta0],     # Theta initial orientation
                                [P.zdot0],      # zdot initial velocity
                                [P.thetadot0]]) # Thetadot initial velocity
        self.ctrl = ctrl                        # Controller class
        self.F = 0.0                            # Force

    def propagateDynamics(self,u):
        # P.Ts is the time step between function calls.


        # RK4 integration
        k1 = self.Derivatives(self.state, u)
        k2 = self.Derivatives(self.state + P.Ts/2.0*k1, u)
        k3 = self.Derivatives(self.state + P.Ts/2.0*k2, u)
        k4 = self.Derivatives(self.state + P.Ts*k3, u)
        self.state += P.Ts/6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4)


    # Return the derivatives of the continuous states
    def Derivatives(self,state,u):

        # States and forces
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)
        F = u
        self.F = F # Update F


        thetaddot = (F[0]*P.ell*np.cos(theta)-2.0*P.m1*z*zdot*thetadot-P.m1*P.g*z*np.cos(theta)-P.m2*P.g*P.ell*np.cos(theta)/2)/(P.m2*P.ell**2/3+P.m1*z**2)
        zddot = (1.0/P.m1)*(P.m1*z*thetadot**2.0-P.m1*P.g*np.sin(theta))

        
        xdot = np.matrix([[zdot],[thetadot],[zddot],[thetaddot]])

        return xdot


    # Returns the observable states
    def Outputs(self):
        # Return them in a list and not a matrix
        return self.state[0:2].T.tolist()[0]

    # Returns all current states
    def States(self):
        # Return them in a list and not a matrix
        return self.state.T.tolist()[0]

    def getForces(self):
        return self.F
