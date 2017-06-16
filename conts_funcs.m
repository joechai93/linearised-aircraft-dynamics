% Flight condition
q = 44000;
S = 8.308;
d = 11.02;
V = 1492;
m = 834.23 ;
I = 9652.18;
% Stability and Control Derivatives
% Mach 5 0 alpha 0 de
CNa  = 0.0177/0.0174533;
Cma  = 0.001308194/0.0174533;
Cmq = -0.001/0.0174533; % from datcom
Cmde = -0.0011/0.0174533;
Na = q*S*CNa/m;
Ma = q*S*d*Cma/I;
Mq = (q*S*Cmq*d^2)/(2*I*V);
Mde = (q*S*d*Cmde)/I;