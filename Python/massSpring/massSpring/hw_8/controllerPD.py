import numpy as np 
import param as P

def saturate(limit,u):
	if abs(u) > limit:
		u = limit*np.sign(u)
	return u

def getForces(ref_inputs,states):
  z = states[0]                      # Current z
  zdot = states[1]                   # Current zdot
  z_r = ref_inputs[0]                # Command z

  F_unsat = P.kp*(z_r-z) - P.kd*zdot 
  F_sat = saturate(P.Fmax,F_unsat)

  return [F_sat]
