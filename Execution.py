# -*- coding: utf-8 -*-
"""
Created on Thu Apr  6 17:14:22 2017

@author: uqjchai2
"""
from Functions import *
from Constants import *
# store history of state variables , and control
xhist = [(0,0,0,0)]
dehist = [0]
thist = [0]
eahist = [0]
Thist = [0]
# initial conditions
x = xhist[0][0], xhist[0][1], xhist[0][2], xhist[0][3]
ea = eahist[0]
# solver 
dt = 0.004
dT = 0.1 # sampling time for controller 
t = 0
tfinal = 200 # seconds
tcntrl = 0 # triggers control law when reaches sampling time
de = 0
while t < tfinal:
    #-----------------------
    # create elevator pulse input
    #while 1 < t < 3:
    #     de = -0.11
    #     break
    #if t > 3:
    #     de = 0
        
     #----------------------   
    # call modules
    # Control Module
    while 0 < t < tfinal:
        if tcntrl >= dT:
            #de = pitchrate_control(x[3],0,x[2],-0.8,0.1)
            tcntrl = 0
        break
    #Vehicle Dynamics Module
    xdot = state_space(x,de)
    #Kinematics module
    x = kinematics(xdot,x,dt)    
    #------------------------------
    # save histories
    xhist.append(x)
    dehist.append(de)
    eahist.append(ea)
    # update time
    tcntrl += dt
    t+=dt
    thist.append(t)
import pylab
u_plot = [xhist[i][0]*U for i in range(len(xhist))] 
a_plot = [xhist[i][1]*57.2958 for i in range(len(xhist))]
theta_plot = [xhist[i][3]*57.2958 for i in range(len(xhist))]
thetadot_plot = [xhist[i][2]*57.2958 for i in range(len(xhist))]
de_plot = [dehist[i]*57.2958 for i in range(len(dehist))]
ea_plot = [eahist[i]*57.2958 for i in range(len(xhist))]

pylab.plot(thist,u_plot,label="Forward Velocity")
pylab.legend(loc="best")
pylab.grid()

pylab.figure()
pylab.plot(thist,theta_plot,label="Pitch, deg")
pylab.plot(thist,thetadot_plot,label="Pitchrate, deg/s")
pylab.legend(loc="best")
pylab.grid()

pylab.figure()
pylab.plot(thist,a_plot,label="AoA, deg")
pylab.plot(thist,de_plot,"r-",label="Elevator Deflection, deg")
pylab.legend(loc="best")
pylab.grid()
pylab.show()
