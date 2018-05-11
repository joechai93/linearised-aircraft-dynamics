function [dydx] = forward_diff(yn,yn1,dx)
dydx = (yn1-yn)/dx;
