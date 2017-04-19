# -*- coding: utf-8 -*-
"""
Created on Thu Apr  6 17:14:22 2017

@author: uqjchai2
"""
from Functions import *
# store history of state variables , and control
xhist = [(0,0.5,0,0)]
dehist = [0]
thist = [0]
eahist = [0]

# initial conditions
x = xhist[0][0], xhist[0][1], xhist[0][2], xhist[0][3]
ea = eahist[0]
# solver 
dt = 0.05
t = 0
tfinal = 300 # seconds

while t < tfinal:
    de = 0
    # create impulse input
    while 1 < t < 3:
        #de = -0.11
        break
    # call modules
    # Control Module
    while 3.1 < t < tfinal:
        #de, ea = accel_control(xdot[1],0,x[2],0.3,0.23,ea,dt)
        de, ea = pitch_control(x[2],0,0.5,0.4,ea,dt,de)
        break
    #Vehicle Dynamics Module
    xdot = state_space(x,de)
    #Kinematics module
    x = kinematics(xdot,x,dt)    
    #print(x)
    # save histories
    xhist.append(x)
    dehist.append(de)
    eahist.append(ea)
    # update time
    t+=dt
    thist.append(t)
    
import pylab
u_plot = [xhist[i][0]*U for i in range(len(xhist))] 
a_plot = [xhist[i][1]*57.2958 for i in range(len(xhist))]
theta_plot = [xhist[i][3]*57.2958 for i in range(len(xhist))]
thetadot_plot = [xhist[i][2]*57.2958 for i in range(len(xhist))]
de_plot = [dehist[i]*57.2958 for i in range(len(xhist))]
ea_plot = [eahist[i]*57.2958 for i in range(len(xhist))]

pylab.plot(thist,u_plot,label="Velocity")
pylab.legend(loc="best")
pylab.show()

pylab.figure()
pylab.plot(thist,theta_plot,label="Pitch, deg")
pylab.plot(thist,thetadot_plot,label="Pitchrate, deg/s")
pylab.legend(loc="best")
pylab.show()

pylab.figure()
pylab.plot(thist,a_plot,label="AoA, deg")
pylab.plot(thist,de_plot,"r-",label="Elevator Deflection, deg")
pylab.legend(loc="best")
pylab.show()