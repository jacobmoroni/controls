import param as P 
import numpy as np 

def saturate(limit,u):
	if abs(u) > limit:
		u = limit*np.sign(u)
	return u

def getForces(ref_input, states):
	z_r = ref_input[0]
	z = states[0]
	th = states[1]
	zdot = states[2]
	thdot = states[3]

	Fe = P.m1*P.g*z/P.ell +0.5*P.m2*P.g
	th_r = P.z_kp*(z_r-z)-P.z_kd*zdot
	F_unsat = P.th_kp*(th_r-th)-P.th_kd*thdot + Fe
	F_sat = saturate(P.Fmax,F_unsat)


	return [F_unsat]