m = 1;
k = 1;
c = 1;

num_G = [1];
den_G = [m c k];


G = tf(num_G,den_G);

p = pole (G);
mag_p = abs(p);

figure(1)
asymp(G)

%PI controller
kp = 10;
Ti = 3;
ki = kp/Ti;

mag_z = 1/Ti;

num_GH = [kp*Ti kp];
den_GH = [Ti*m Ti*c Ti*k 0];

GH = tf(num_GH,den_GH);
figure(2)
asymp(GH)