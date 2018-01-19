import numpy as np 
import param as P

# Clips u at the limit
def saturate(limit,u):
	if abs(u) > limit:
	  u = limit*np.sign(u)
	return u

def getForces(ref_inputs,states):
  h_r = ref_inputs[0]                # Command h 
  z_r = ref_inputs[1]                # Command z
  z = states[0]                      # Current z
  h = states[1]                      # Current h
  th = states[2]                     # Current th
  zdot = states[3]                   # Current zdot
  hdot = states[4]                   # Current hdot
  thdot = states[5]                  # Current thdot

  F_tilde = P.h_kp*(h_r-h) - P.h_kd*hdot

  F = saturate(P.Ftildemax,F_tilde) + P.Fe 

  th_r = P.z_kp*(z_r-z) - P.z_kd*zdot # Command theta
  tau_unsat = P.th_kp*(th_r-th)-P.th_kd*thdot 
  tau_sat = saturate(P.taumax,tau_unsat)

  return [F,tau_sat]


