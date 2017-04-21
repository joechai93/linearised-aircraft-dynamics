# -*- coding: utf-8 -*-
"""
Created on Tue Apr 18 15:13:44 2017

@author: uqjchai2
"""
# Flight Condition, taken from Blakelock, pg 37
Theta = 0 #0 # eq pitch angle
Mach = 0.62 #0.62
m = 5900#84644.64 # kg, 5800 slugs
U = 440#182.88 # m/s, 600 ft/s
S = 2400#222.9673 # m2, 2400 sq feet
Iy = 2.62*10**6 #3552243.029 # kg-m^2
Ix = 1.955*10**6 
Iz = 4.2*10**6 
c = 20.2#*0.3
b = 130

# four engine jet transport
# Longitudinal Stability Derivatives, Blakelock p37
Cxu = -0.088 
Cxa = 0.392
Cw = -0.74
Cxq = 0
l_c = 2.89
Czu = -1.48
Cza = -4.46
Czad = -1.13
Czq = -3.94
Cma = -0.619
Cmad = -3.27
Cmu = 0
Cmq = -11.4
mU_Sq = 4.74 #4.74 #13.7 # seconds

# Longitudinal Control Derivatives
Cmde = -0.71
Czde = (1/l_c)*Cmde

# Lateral Stability Derivatives, Blacklock p122
Clp = -0.38
Clr = 0.086
Cnp = -0.0228
Cnb = 0.096
Cnr = -0.107
Cyb = -0.6
Cyph = 0.344
Cyps = 0
Cyp = 0
Cyr = 0
Clb = -0.057
b_2U = b/(2*U)

# Lateral Control Derivatives
Cldar = 0.6 #0.0131
Cndar = -0.01 #-0.08
Cydar = 0 #0.171
#Inertias
I_Sqc = 0.514 # seconds squared
Ix_Sqb = 0.02725
Iz_Sqb = 0.0585
