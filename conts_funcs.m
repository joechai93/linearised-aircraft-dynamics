% Flight condition
q = 50000;
S = 8.308;
d = 8.308;
V = 200;
m = 850 ;
I = 10000;
% Stability and Control Derivatives
CNa  = 1 ;
Cma  = 1;
Cmq = 1;
Cmde = 1;
Na = q*S*CNa/m;
Ma = q*S*d*Cma/I;
Mq = (q*S*Cmq*d^2)/(2*I*V);
Mde = (q*S*d*Cmde)/I;
