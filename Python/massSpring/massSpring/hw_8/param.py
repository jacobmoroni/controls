import numpy as np

# Mass Spring physical parameters
m = 5.0          # Mass of block, kg
k = 3.0          # Spring constant, Kg/s**2
b = 0.5          # Damping constant, Kg/s

# Simulation Parameters
Ts = 0.01
sigma = 0.05

# Initial Conditions
z0 = 0.0                # ,m
zdot0 = 0.0             # ,m/s


####################################################
#    PD Control: Pole Placement
####################################################

Fmax = 2

# Open Loop
# b0/(S**2 + a1*S + a0)
b0 = 1/m
a1 = b/m
a0 = k/m

# tr = 2            # Rise time, s
tr = 2.2
zeta = 0.707      # Damping Coefficient
wn = 2.2/tr       # Natural frequency

# S**2 + alpha1*S + alpha0
alpha1 = 2*zeta*wn
alpha0 = wn**2

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
kp = (alpha0-a0)/b0
kd = (alpha1-a1)/b0

print('kp: ',kp)
print('kd: ',kd)
