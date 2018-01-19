import numpy as np

# Ball on Beam physicall parameters.
m1 = 0.35       # Ball mass, kg
m2 = 2.0        # Beam mass, kg
ell = 0.5       # Length of beam, m
g = 9.81         # Gravity, m/s**2


# Simulation Parameters
Ts = 0.001

# Initial Conditions
z0 = 0.0                # ,m
theta0 = 0.0*np.pi/180.0  # ,rads
zdot0 = 0.0             # ,m/s
thetadot0 = 0.0         # ,rads/s

####################################################
#       PD Control: Time Design Strategy
####################################################
Fmax = 15.0             		  # Max Force, N
# M = 10.0             		  # Time scale separation between
					 		  # inner and outer loop
M = 10.0                        # Optimized
Ze = ell/2.0                  # equilibrium position

#---------------------------------------------------
#                    Inner Loop
#---------------------------------------------------
# Open Loop
# b0/(S**2 + a1*S + a0)
th_b0 = ell/(m2*ell**2/3+m1*Ze**2)
th_a1 = 0.0
th_a0 = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

#th_tr = 1            # Rise time, s
th_tr = 1.0          # Optimized
th_zeta = 0.707       # Damping Coefficient
th_wn = 2.2/th_tr     # Natural frequency

# S**2 + alpha1*S + alpha0
th_alpha1 = 2.0*th_zeta*th_wn
th_alpha0 = th_wn**2

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
th_kp = (th_alpha0-th_a0)/th_b0
th_kd = (th_alpha1-th_a1)/th_b0
th_DC = 1.0   #DC gain


#---------------------------------------------------
#                    Outer Loop
#---------------------------------------------------
# Open Loop
# b0/(S**2 + a1*S + a0)
z_b0 = -g
z_a1 = 0.0
z_a0 = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

z_tr = M*th_tr  # Rise time, s
z_zeta = 0.707   # Damping Coefficient
z_wn = 2.2/z_tr  # Natural frequency


# S**2 + alpha1*S + alpha0
z_alpha1 = 2.0*z_zeta*z_wn
z_alpha0 = z_wn**2

# Gains
# b0*kp/[S**2 + (a1 + b0*kd*IL_DC)S + (a0 + b0*kp*IL_DC)]
z_kp = (z_alpha0-z_a0)/(th_DC*z_b0)
z_kd = (z_alpha1-z_a1)/(th_DC*z_b0)



print('th_DC', th_DC)
print('th_kp: ',th_kp)
print('th_kd: ',th_kd)
print('z_kp: ',z_kp)
print('z_kd: ',z_kd)
