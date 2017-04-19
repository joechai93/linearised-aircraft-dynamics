# -*- coding: utf-8 -*-
"""
Created on Tue Apr 18 15:13:44 2017

@author: uqjchai2
"""
# Flight Condition, taken from Blakelock, pg 37
Theta = 0 #0 # eq pitch angle
Mach = 0.62 #0.62
m = 5800#84644.64 # kg, 5800 slugs
U = 600#182.88 # m/s, 600 ft/s
S = 2400#222.9673 # m2, 2400 sq feet
Iy = 2.62*10**6 #3552243.029 # kg-m^2

# four engine jet transport
# Stability Derivatives, taken from Blakelock, pg 37
Cxu = -0.088 
Cxa = 0.392
Cw = -0.74
Cxq = 0
l_c = 2.89
c = 20.2#*0.3
Czu = -1.48
Cza = -4.46
Czad = -1.13
Czq = -3.94
Cma = -0.619
Cmad = -3.27
Cmu = 0
Cmq = -11.4
mU_Sq = 13.78 # seconds
I_Sqc = 0.514 # seconds squared

# Control Derivatives
Cmde = -0.71
Czde = (1/l_c)*Cmde