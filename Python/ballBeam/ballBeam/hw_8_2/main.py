import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import param as P
from step import step_function
from sim_plot import plotGenerator
import controllerPD as ctrl 

# The Animation.py file is kept in the parent directory,
# so the parent directory path needs to be added.
sys.path.append('..')
from dynamics2 import ballBeamDynamics
from animation import ballBeamAnimation

t_start = 0.0   # Start time of simulation
t_end = 50.0    # End time of simulation
t_Ts = P.Ts     # Simulation time step
t_elapse = 0.01  # Simulation time elapsed between each iteration
t_pause = 0.01  # Pause between each iteration


plotGen = plotGenerator()           # Instantiate plotGenerator class
simAnimation = ballBeamAnimation()  # Instantiate Animate class
dynam = ballBeamDynamics(ctrl)      # Instantiate Dynamics class

t = t_start               # Declare time variable to keep track of simulation time elapsed

while t < t_end:

    ref_input = [step_function(t,0.5,0.25)]

    # The dynamics of the model will be propagated in time by t_elapse
    # at intervals of t_Ts.
    t_temp = t +t_elapse
    while t < t_temp:
        states = dynam.States()              # Get current states
        u = ctrl.getForces(ref_input,states)
        dynam.propagateDynamics(u)           # Propagate the dynamics of the model in time
        t = round(t +t_Ts,3)                 # Update time elapsed

    # plt.figure(simAnimation.fig.number) # Switch current figure to animation figure
    # simAnimation.drawBallBeam(          # Update animation with current user input
    #     dynam.Outputs())
    # plt.pause(0.0001)

    # Organizes the new data to be passed to plotGen
    new_data = [[ref_input[0],states[0]],
                [0,states[1]],
                u]
    plotGen.updateDataHistory(t, new_data)

    # plt.figure(plotGen.fig.number)      # Switch current figure to plotGen figure
    # plotGen.update_plots()              # Update the plot
    # plt.pause(0.0001)

    # time.sleep(t_pause)


plt.figure(plotGen.fig.number)
plotGen.update_plots()
plt.pause(0.001)

# Keeps the program from closing until the user presses a button.
print('done')
raw_input()
