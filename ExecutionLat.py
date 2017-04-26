# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 13:16:48 2017

@author: uqjchai2
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Apr  6 17:14:22 2017

@author: uqjchai2
"""
from Functions import *
from Constants import *
# store history of state variables , and control
xhist = [(0,0,0.05,0,0)]
darhist = [0]
thist = [0]
eahist = [0]
Thist = [0]

# initial conditions
x = xhist[0][0], xhist[0][1], xhist[0][2], xhist[0][3], xhist[0][4]
ea = eahist[0]
# solver 
dt = 0.004
dT = 0.1 # sampling time for controller 
t = 0
tfinal = 35# seconds
tcollect = 0 # triggers control law when reaches sampling time
dar = 0
while t < tfinal:
    # create impulse input
    while 1 < t < 2:
        #dar = 0.036
        break
    if 2 < t < 3:
        dar = 0
    if t > 3:
        dar = 0
    #-----------------------------------
    # call modules
    # Control Module
    while 0 < t < tfinal:
        if tcollect >= dT:
            #de, ea = accel_control(xdot[1],0,x[2],0.3,0.23,ea,dt)
            #de, ea = pitch_control(x[2],0,0.8,0.03,ea,dT,de) # PI controller
            tcollect = 0
        break
    #Vehicle Dynamics Module
    xdot = state_space_lat(x,dar)
    #Kinematics module
    x = kinematics(xdot,x,dt)    
    #-----------------------------------
    # save histories
    xhist.append(x)
    darhist.append(dar)
    eahist.append(ea)
    # update time
    tcollect += dt
    t+=dt
    thist.append(t)
    
import pylab
phidot_plot = [xhist[i][0]*57.2958 for i in range(len(xhist))] 
psidot_plot = [xhist[i][1]*57.2958 for i in range(len(xhist))]
beta_plot = [xhist[i][2]*57.2958 for i in range(len(xhist))]
phi_plot = [xhist[i][3]*57.2958 for i in range(len(xhist))]
psi_plot = [xhist[i][4]*57.2958 for i in range(len(xhist))]

dar_plot = [darhist[i]*57.2958 for i in range(len(darhist))]
ea_plot = [eahist[i]*57.2958 for i in range(len(xhist))]

pylab.plot(thist,phidot_plot,label="Roll Rate, deg/s")
pylab.plot(thist,phi_plot,label="Roll Angle, deg")
pylab.legend(loc="best")
pylab.grid()

pylab.figure()
pylab.plot(thist,psidot_plot,label="Yaw Rate, deg/s")
pylab.plot(thist,psi_plot,label="Yaw Angle, deg")
pylab.legend(loc="best")
pylab.grid()

pylab.figure()
pylab.plot(thist,beta_plot,label="Sideslip, deg")
pylab.plot(thist,dar_plot,"r-",label="Rudderon Deflection, deg")
pylab.legend(loc="best")
pylab.grid()
pylab.show()
