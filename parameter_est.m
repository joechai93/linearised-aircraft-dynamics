run pitch_reduced.m
%------------------------------------------------
% Parameter Estimation Test
% Equation Error Method, pg 25 Raol
[u,t] = gensig('sin',0.1,0.3,0.00005);
[q_t, a_t] = lsim(pitch_open_sys,u,t);
figure('visible','off');
plot(t,q_t,t,a_t,t,u);
legend('Pitch rate','normal accel','Input');
xam = [q_t(1) q_t(2) q_t(3) q_t(4) q_t(5) q_t(6) q_t(7);
       a_t(1) a_t(2) a_t(3) a_t(4) a_t(5) a_t(6) a_t(7);
       u(1) u(2) u(3) u(4) u(5) u(6) u(7)];
xdotm = [F*xam(1:2,1)+g*u(1) F*xam(1:2,2)+g*u(2) F*xam(1:2,3)+g*u(3) F*xam(1:2,4)+g*u(4) F*xam(1:2,5)+g*u(5) F*xam(1:2,6)+g*u(6) F*xam(1:2,7)+g*u(7)];
Aa = xdotm*transpose(xam)/(xam*transpose(xam));
err = xdotm - Aa*xam;
F
g
Aa
%--------------------------------------------------------------------------
% now, try calculating xdotm using finite difference
% xm = [q_m a_m], xdotm is unlikely to be available from sensors
xdotm_fd = [0 forward_diff(q_t(1),q_t(2),(t(2)-t(1))) forward_diff(q_t(2),q_t(3),(t(3)-t(2))) forward_diff(q_t(3),q_t(4),(t(4)-t(3))) forward_diff(q_t(4),q_t(5),(t(5)-t(4)));
            0 forward_diff(a_t(1),a_t(2),(t(2)-t(1))) forward_diff(a_t(2),a_t(3),(t(3)-t(2))) forward_diff(a_t(3),a_t(4),(t(4)-t(3))) forward_diff(a_t(4),a_t(5),(t(5)-t(4)))];
% does not work as well



