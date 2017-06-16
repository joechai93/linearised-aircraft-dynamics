run conts_funcs.m
F = [Mq Ma/Na;
     Na -Na/V];
g = [Mde; 0];
hz = [1; 1];
D = 0;
%---------------------------------------------
% open loop
pitch_open_sys = ss(F,g,transpose(hz),D);
[num,den] = ss2tf(F,g,transpose(hz),D);
pitch_open_tf = tf(num,den);

%----------------------------------------------
% pole placement 
omega = 15;
p = 8;
zeta = 0.76;

% Gain selection
Gp = 0.005;
k1 = (1/(Na*Mde))*(omega^2 + 2*zeta*omega*p + Ma + (Mq*Na/V) - k2*(Mde*Na/V)) - Gp;
k2 = (1/Mde)*(2*zeta*omega + p + Mq - (Na/V));
Gi = omega^2 * p/(Na*Mde);
c = [k2; k1];
%----------------------------------------------
% closed loop
F_c = [F-g*(transpose(c)+Gp*transpose(hz)) Gi*g;
       -transpose(hz) 0];
g_c = [Gp*g; 1];
hz_c = [0;1;0];
pitch_closed_sys = ss(F_c,g_c,transpose(hz_c),D);
[num,den] = ss2tf(F_c,g_c,transpose(hz_c),D);
pitch_closed_tf = tf(num,den);
%-----------------------------------------------
step(pitch_closed_tf);

% Linear simulation
figure('visible','off');
[u,t] = gensig('square',5,12,0.05);
tramp=0:0.1:5;
ramp = 2*t;
lsim(pitch_closed_sys,u,t);
title('LTI Simulation at Mach 5','Interpreter','latex','FontSize',16);
xlabel('Time','Interpreter','latex','FontSize',14);
ylabel('$a_N$','Interpreter','latex','FontSize',14);
h = findobj(gcf,'type','line');
set(h,'linewidth',2.8);
grid on
set(gca, 'FontWeight', 'bold', 'FontSize', 11);
[gain_margin, phase_margin, freq_gain_margin, freq_phase_margin] = margin(pitch_closed_sys);

