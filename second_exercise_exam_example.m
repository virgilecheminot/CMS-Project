clear all
close all
clc

kp = 1;
kphi = 0.001;
td = 100;
J = 1;
r = -1;
k = 1;
La = 10;
Ra = 1;

s = tf('s');
GH = kphi*kp*(1+td*s)/( (Ra+La*s)*(J*s^2+r*s+k)+kphi^2*s );

pole(GH)

figure(1)
asymp(GH)

figure(2)
nyquist(GH)

figure(3)
rlocus(GH)




