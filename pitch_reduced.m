A = [Mq Ma/Na;
     Na -Na/V];
B = [Mde; 0];
C = [0 1];
D = 0;


pitch_open_sys = ss(A,B,C,D);
[num,den] = ss2tf(A,B,C,D);
pitch_open_tf = tf(num,den);