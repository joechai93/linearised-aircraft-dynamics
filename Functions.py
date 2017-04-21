# -*- coding: utf-8 -*-
"""
Created on Thu Apr  6 10:49:00 2017

@author: uqjchai2
"""
import math
import numpy as n
from Constants import *
#-----------------------------------------------------------------------------
# Module: Pitch Orientation Control Law
# No actuator dynamics modelled (assume instant from command to deflection)
#-----------------------------------------------------------------------------
def pitch_control(thetadot,thetadotc,Ki,Kp,eao,dt,deo):
    ea = ((thetadotc-thetadot)*Ki*dt + eao)+ Kp*(thetadotc-thetadot)# PI controller
    #erg = thetadot*Srg
    de = ea #-10*((ea-erg)+deo)*dt + deo
    if de > 0.52:
        de = 0.52
    if de < -0.52:
        de = -0.52
    return de, ea
#-----------------------------------------------------------------------------
# Module: Acceleration control Law
# No actuator dynamics modelled (assume instant from command to deflection)
#-----------------------------------------------------------------------------
def accel_control(az,azc,thetadot,Ka,Srg,eao,dt):
    ea = (azc-az)*Ka*dt + eao
    #erg = thetadot*Srg
    de = ea #+ erg
    if de > 0.52:
        de = 0.52
    if de < -0.52:
        de = -0.52
    return de, ea
#-----------------------------------------------------------------------------
# Module: State Space Dynamics Lateral x = [phidot psidot beta phi psi ]
# Returns state dynamics xdot = Ax + Bu
#-----------------------------------------------------------------------------
def state_space_lat(xo,dar):
    phido, psido, betao, phio, psio = xo
    x = n.matrix([[phido],[psido],[betao],[phio],[psio]])
    A = n.matrix([[b_2U*Clp/Ix_Sqb, b_2U*Clr/Ix_Sqb, Clb/Ix_Sqb, 0, 0],
                  [b_2U*Cnp/Iz_Sqb, b_2U*Cnr/Iz_Sqb, Cnb/Iz_Sqb, 0, 0],
                  [b_2U*Cyp/mU_Sq, (b_2U*Cyr-mU_Sq)/mU_Sq, Cyb/mU_Sq, Cyph/mU_Sq, Cyps/mU_Sq],
                  [1, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0]])
    B = n.matrix([[Cldar/Ix_Sqb], [Cndar/Iz_Sqb], [Cydar/mU_Sq], [0], [0]])
    xdot = n.dot(A,x) + B*dar    
    return float(xdot[0]), float(xdot[1]), float(xdot[2]), float(xdot[3]), float(xdot[4])

#-----------------------------------------------------------------------------
# Module: State Space Dynamics Longitudinal x = [u alpha thetadot theta]
# Assumes no downwash lag for Cx and Cm
# returns state dynamics xdot = Ax + Bu
#-----------------------------------------------------------------------------
def state_space(xo,de):
    uo, ao, thetadoto, thetao = xo
    x = n.matrix([[uo],[ao],[thetadoto],[thetao]])
    A = n.matrix([[Cxu/mU_Sq,Cxa/mU_Sq,(c/(2*U))*Cxq/mU_Sq,Cw*math.cos(Theta)/mU_Sq],
                   [Czu/(mU_Sq+c*Czad/(2*U)),Cza/(mU_Sq+c*Czad/(2*U)),(mU_Sq+(c/(2*U))*Czq)/(mU_Sq+c*Czad/(2*U)),Cw*math.sin(Theta)/(mU_Sq+c*Czad/(2*U))],
                   [Cmu/I_Sqc,Cma/I_Sqc,(c/(2*U))*Cmq/I_Sqc,0/I_Sqc],
                   [0,0,1,0]])
    B = n.matrix([[0],[Czde/mU_Sq],[Cmde/I_Sqc],[0]])
    xdot = n.dot(A,x) + B*de
    
    return float(xdot[0]), float(xdot[1]), float(xdot[2]), float(xdot[3])
#-----------------------------------------------------------------------------
# Module: Kinematics
# Takes state dynamics, current state and time step size
# returns new state
#-----------------------------------------------------------------------------
def kinematics(xdot,xo,dt):
    # xdot = state dynamics
    # dt = timestep
    # unpack state dynamics
    if len(xdot) == 4:
        # longitudinal 
        udot, adot, thetadotdot, thetadot = xdot
        uo, ao, thetadoto, thetao = xo
        # use euler 1st order
        u1 = udot*dt + uo
        a1 = adot*dt + ao
        thetadot1 = thetadotdot*dt + thetadoto
        theta1 = thetadot*dt + thetao
        x = u1, a1, thetadot1, theta1
    if len(xdot) == 5:
        # lateral 
        phidotdot, psidotdot, bdot, phidot, psidot = xdot
        phidoto, psidoto, bo, phio, psio = xo
        # euler 1st order
        phidot1 = phidotdot*dt + phidoto
        psidot1 = psidotdot*dt + psidoto
        b1 = bdot*dt + bo
        phi1 = phidot*dt + phio
        psi1 = psidot*dt + psio
        x = phidot1, psidot1, b1, phi1, psi1
    return  x
#------------------------------------------------------------------------------
# gravity
#------------------------------------------------------------------------------
def gravity(y):
    return 9.81*(6371000/(6371000+y))**2
#------------------------------------------------------------------------------
# Atmosphere Model
#------------------------------------------------------------------------------
def rhoinf(y):  # Atmospheric density
    # y is altitude
    # US standard atmosphere 1976, NASA
    if y <= 11000:
        return 1.225*(1-y/44329)**4.255876
    if 11000 < y <= 20000:
        return 1.225*(0.297076)*math.exp((10999-y)/6341.4)
    if 20000 < y <= 32000:
        return 1.225*(0.978261 + y/201010)**(-35.16319)
    if 32000 < y <= 47000:
        return 1.225*(0.857003 + y/57944)**(-13.20114)
    if 47000 < y <= 51000:
        return 1.225*(0.00116533)*math.exp((46998-y)/7922)
    if 51000 < y <= 71000:
        return 1.225*(0.79899 - y/184800)**(11.20114)
    else:
        return 1.225*(0.79899 - y/184800)**(11.20114)

def Tinf(y): # Atmospheric Temperature
    # y is altitude
    # US standard atmosphere 1976, NASA
    if y <= 11000:
        return 288.15*(1-y/44329)
    if 11000 < y <= 20000:
        return 0.751865*288.75
    if 20000 < y <= 32000:
        return 288.15*(0.682457 + y/288136)
    if 32000 < y <= 47000:
        return 288.15*(0.482561+y/102906)
    if 47000 < y <= 51000:
        return 288.15*0.939268
    if 51000 < y <= 71000:
        return 288.15*(1.434843 - y/102906)
    else:
        return print("Error: altitude not supported")

def Pinf(y): # Atmospheric Pressure 
    # y is altitude
    # US standard atmosphere 1976, NASA
    if y <= 11000:
        return 101325*(1-y/44329)**5.255876
    if 11000 < y <= 20000:
        return 101325*(0.223361)*math.exp((10999-y)/6341.4)
    if 20000 < y <= 32000:
        yl = [20000,25000,30000]
        Pl = [5473.75,2549.0,1197.0]
        return n.interp(y,yl,Pl)
    if 32000 < y <= 47000:
        return 101325*(0.898309 + y/55280)**(-12.20114)
    if 47000 < y <= 51000:
        return 101325*(0.00109456)*math.exp((46998-y)/7922)
    if 51000 < y <= 71000:
        return 101325*(0.838263 - y/176142)**12.20114
    else:
        return print("Error: altitude not supported")