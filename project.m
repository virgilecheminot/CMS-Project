close all; clear; clc;

%% System data
g = 9.81;               % gravity acceleration [m/s^2]
M = 1500;               % vehicle mass [kg]
J_wheel = 1;            % moment of inertia of the wheel [kg*m^2]
J_M = 0.05;             % moment of inertia of the motor [kg*m^2]
alpha = deg2rad(5);     % slope of the road [degrees]
Rw = 0.35;               % wheel radius [m]
r1 = 0.005;             % damping coefficient 1 [Nms/rad]
r2 = 0.005;             % damping coefficient 2 [Nms/rad]
k_t = Inf;              % torsional stiffness of the drive shaft [Nm/rad]
tau_1 = 3;              % transmission ratio [-]
tau_2 = 1;              % differential ratio [-]
rho_air = 1.225;        % air density [kg/m^3]
A_front = 2.2;          % front surface of the vehicle [m^2]
Cx = 0.3;               % drag coefficient [-]
C_RR = 0.01;            % rolling coefficient [-]
k_RR = 0.0002;          % rolling coefficient [s/m]

m_star = J_M*(tau_1*tau_2/Rw)^2+4*J_wheel/(Rw^2)+M;
r_star = (r1+r2)*(tau_2/Rw)^2;
fprintf('m*=%f ; r*=%f\n',m_star,r_star);

%% Part 1 : Steady-state analysis
v0 = 15;                % steady-state vehicle speed [m/s]
v_w0 = 3.5;             % steady-state wind speed [m/s]

ratio = Rw/(tau_2*tau_1);
F_D0 = 1/2*rho_air*Cx*A_front*(v0+v_w0)^2;
F_R0 = M*g*C_RR*(1+k_RR*v0);
P = M*g*sin(alpha);
damp_0 = r_star*v0;

T_M0 = ratio*(F_D0 + F_R0 + P + damp_0);
fprintf('Steady-state motor torque : %f N*m\n',T_M0);
 
r_gen = r_star + C_RR*k_RR*M*g + rho_air*Cx*A_front*(v0 + v_w0);

numG = 1/ratio;
denG = [m_star, r_gen];
G = tf(numG, denG); % TF of v/T_M
figure;
bode(G);
% poles = pole(G);
% fprintf('Poles of G : %f\n',poles);

numD = -rho_air*Cx*A_front*(v0+v_w0);
denD = denG;
D = tf(numD, denD); % TF of v/v_wind
figure;
bode(D);

%% Part 2 : PI regulator
Kp = 75; % Proportional gain
Ti = 100; % Integral time constant
Ki = Kp / Ti; % Integral gain
R = pid(Kp, Ki);

% Open-loop transfer function
RG = series(R, G);
fprintf('Poles of RG : %f\n',pole(RG)');
fprintf('Zeros of RG : %f\n',zero(RG)');


% Closed-loop transfer function
L = feedback(RG, 1);

% Stability analysis
figure;
margin(RG);
title(['Bode plot with Ti = ', num2str(Ti)]);

figure;
nyquist(RG);
title(['Nyquist plot with Ti = ', num2str(Ti)]);

figure;
rlocus(RG);
title(['Root locus with Ti = ', num2str(Ti)]);

% Step response performance
figure;
step(L);
title(['Step response with Ti = ', num2str(Ti)]);

stp_nfo = stepinfo(L);
tr = stp_nfo.RiseTime;
Po = stp_nfo.Overshoot;
ts = stp_nfo.SettlingTime;
BW = bandwidth(L);
fprintf('Rise time : %f\n',tr);
fprintf('Overshoot : %f\n',Po);
fprintf('Settling time : %f\n',ts);
fprintf('Bandwidth : %f\n',BW);

% Bode diagrams of the open-loop transfer function
figure;
bode(RG);
title('Bode plot of the open-loop transfer function');

% Bode diagrams of the closed-loop transfer function
figure;
bode(L);
title('Bode plot of the closed-loop transfer function');

% Bode diagrams of the closed-loop wind disturbance transfer function
K2 = -rho_air*Cx*A_front*(v0+v_w0);
G_D = tf(1, denG);
Kc = numG;
LD = K2 * feedback(G_D, Kc*R);
figure;
bode(LD);
title('Bode plot of the closed-loop wind disturbance transfer function');