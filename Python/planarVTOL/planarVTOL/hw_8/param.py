import numpy as np

# Physical parameters of the inverted pendulum
mc = 1.0        # Center mass, kg
Jc = 0.0042     # Inertia, kg m**2
mr = 0.25       # Right motor mass, kg
ml = 0.25       # Left motor mass, kg
d = 0.3         # Rod length, m
mu = 0.1        # Drag coeff, kg/s
g = 9.81        # Gravity m/s**s  
h_max = 5       # Maximum Altitude Distance, m
z_max = 5       # Minimum Latitude Distance, m

# Simulation Parameters
Ts = 0.01 
sigma = 0.05

# Initial Conditions
z0 = 0.0                # Latitude, m
h0 = 0.0                # Altitude, m
theta0 = 0.0*np.pi/180  # ,rads
zdot0 = 0.0
hdot0 = 0.0
thetadot0 = 0.0

zv0 = 0.0               # Position of target, m
includeTarget = False   # Determines whether to render target or no.


####################################################
#    PD Control: Time Design Strategy
####################################################
Fe = (mc+2*mr)*g 
fmax = 10
Ftildemax = 2*fmax -Fe 
taumax = (fmax-Fe/2)/d

#===================================================
#              Logitudinal Controller
#===================================================

# Open Loop
# b0/(S**2 + a1*S + a0)
h_b0 = 1/(mc+2.0*mr)
h_a1 = 0.0
h_a0 = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

h_tr = 8            # Rise time, s
h_zeta = 0.707      # Damping Coefficient
h_wn = 2.2/h_tr     # Natural frequency

# S**2 + alpha1*S + alpha0
h_alpha1 = 2*h_zeta*h_wn
h_alpha0 = h_wn**2 

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
h_kp = (h_alpha0-h_a0)/h_b0
h_kd = (h_alpha1-h_a1)/h_b0

#==================================================
#               Lateral Controller
#==================================================

M = 10

################### Inner Loop ####################

# Open Loop
# b0/(S**2 + a1*S + a0)
th_b0 = 1/(Jc+2*mr*d**2)
th_a1 = 0.0
th_a0 = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

th_tr = 0.8 #0.3     # Rise time, s
th_zeta = 0.707      # Damping Coefficient
th_wn = 2.2/th_tr    # Natural frequency

# S**2 + alpha1*S + alpha0
th_alpha1 = 2*th_zeta*th_wn
th_alpha0 = th_wn**2 

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
th_kp = (th_alpha0-th_a0)/th_b0
th_kd = (th_alpha1-th_a1)/th_b0

th_k_DC = 1

################## Outer Loop ######################

# Open Loop
# b0/(S**2 + a1*S + a0)
z_b0 = -Fe/(mc+2.0*mr)
z_a1 = mu/(mc+2.0*mr)
z_a0 = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

z_tr = M*th_tr       # Rise time, s
z_zeta = 0.707       # Damping Coefficient
z_wn = 2.2/z_tr      # Natural frequency

# S**2 + alpha1*S + alpha0
z_alpha1 = 2*z_zeta*z_wn
z_alpha0 = z_wn**2 

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
z_kp = (z_alpha0-z_a0)/z_b0
z_kd = (z_alpha1-z_a1)/z_b0

print('h_kp: ',h_kp)
print('h_kd: ',h_kd)
print('z_kp: ',z_kp)
print('z_kd: ',z_kd)
print('th_kp: ',th_kp)
print('th_kd: ',th_kd)






