# aerolookup.py
# Joe Chai
import pylab
from scipy.interpolate import griddata
import numpy as n
from Functions import *
# Need CL(M,alp) and CD(M,alp)
Points = [(0.15,0),(0.15,10),(0.15,30),
          (0.4,0),(0.4,10),(0.4,30),
          (1.2,0),(1.2,10),(1.2,30),
          (2,0),(2,10),(2,30),
          (3,0),(3,10),(3,30),
          (4,0),(4,10),(4,30),
          (5,0),(5,10),(5,30)]
CL_points = [0.0316,0.2336,0.66224,
            0.0267,0.2480986,0.63216,
            0.0116,0.2149,0.60047,
            0.00062424,0.110561,0.6289227,
            -0.011185,0.13028,0.55577,
            -0.02318,0.118479, 0.5257,
            -0.03055,0.103075,0.491417]
CD_points = [0.110056,0.15,0.472208,
            0.071955,0.12156,0.43515,
            0.080314,0.20844,0.499015,
            0.13571,0.14795937,0.485674,
            0.110401,0.12409967,0.425378,
            0.097333,0.10924392,0.400964,
            0.087896,0.0987069,0.377518]

# verification plots
alpha_plot = n.linspace(0,30,30)

pylab.figure()
pylab.plot(alpha_plot,[griddata(Points,CL_points,(0.15,i)) for i in alpha_plot],'-',label='M 0.15')
pylab.title('CL vs alpha')
pylab.plot(alpha_plot,[griddata(Points,CL_points,(0.4,i)) for i in alpha_plot],'-',label='M 0.4')
pylab.plot(alpha_plot,[griddata(Points,CL_points,(1.2,i)) for i in alpha_plot],'-',label='M 1.2')
pylab.plot(alpha_plot,[griddata(Points,CL_points,(2.0,i)) for i in alpha_plot],'-',label='M 2.0')
pylab.plot(alpha_plot,[griddata(Points,CL_points,(3.0,i)) for i in alpha_plot],'-',label='M 3.0')
pylab.plot(alpha_plot,[griddata(Points,CL_points,(4.0,i)) for i in alpha_plot],'-',label='M 4.0')
pylab.plot(alpha_plot,[griddata(Points,CL_points,(5.0,i)) for i in alpha_plot],'-',label='M 5.0')
pylab.legend(loc='best')

pylab.figure()
pylab.plot(alpha_plot,[griddata(Points,CD_points,(0.15,i),method='cubic') for i in alpha_plot],'-',label='M 0.15')
pylab.title('CD vs alpha')
pylab.plot(alpha_plot,[griddata(Points,CD_points,(0.4,i),method='cubic') for i in alpha_plot],'-',label='M 0.4')
pylab.plot(alpha_plot,[griddata(Points,CD_points,(1.2,i),method='cubic')  for i in alpha_plot],'-',label='M 1.2')
pylab.plot(alpha_plot,[griddata(Points,CD_points,(2.0,i),method='cubic')  for i in alpha_plot],'-',label='M 2.0')
pylab.plot(alpha_plot,[griddata(Points,CD_points,(3.0,i),method='cubic') for i in alpha_plot],'-',label='M 3.0')
pylab.plot(alpha_plot,[griddata(Points,CD_points,(4.0,i),method='cubic') for i in alpha_plot],'-',label='M 4.0')
pylab.plot(alpha_plot,[griddata(Points,CD_points,(5.0,i),method='cubic') for i in alpha_plot],'-',label='M 5.0')
pylab.legend(loc='best')

#pylab.show()

#Derivative calculation using finite difference
def CL_lookup(M,a):
    return griddata(Points,CL_points,(M,a))

def CD_lookup(M,a):
    return griddata(Points,CD_points,(M,a),method='cubic')

#CLu = dCL/dM * (1/sound speed)
def CLu_lookup(M,a,sound_speed):
    return central_diff(CL_lookup,M,a,0.1,independent='x1')*1/sound_speed
#CDu = dCD/dM * (1/sound speed)
def CDu_lookup(M,a,sound_speed):
    return central_diff(CD_lookup,M,a,0.001,independent='x1')*1/sound_speed
#do CLa and CDa
#print(CLu_lookup(0.4,0,340)*340)
#print(CDu_lookup(4,0,340)*340)

def Cxu(M,a,sound_speed):
    U = M*sound_speed
    return -2*CD_lookup(M,a) - U*CDu_lookup(M,a,sound_speed)

def Czu(M,a,sound_speed):
    U = M*sound_speed
    return -2*CL_lookup(M,a) - U*CLu_lookup(M,a,sound_speed)


