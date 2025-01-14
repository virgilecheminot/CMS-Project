close all; clear; clc;

%% System data
g = 9.81;               % gravity acceleration [m/s^2]
m = 1500;               % vehicle mass [kg]
J_wheel = 1;            % moment of inertia of the wheel [kg*m^2]
J_M = 0.05;             % moment of inertia of the motor [kg*m^2]
alpha = deg2rad(5);     % slope of the road [degrees]
Rw = 0.35;               % wheel radius [m]
r1 = 0.005;             % damping coefficient 1 [Nms/rad]
r2 = 0.005;             % damping coefficient 2 [Nms/rad]
tau_1 = 3;              % transmission ratio [-]
tau_2 = 1;              % differential ratio [-]
rho_air = 1.225;        % air density [kg/m^3]
A_front = 2.2;          % front surface of the vehicle [m^2]
Cx = 0.3;               % drag coefficient [-]
C_RR = 0.01;            % rolling coefficient [-]
k_RR = 0.0002;          % rolling coefficient [s/m]
Ra = 0.05;              % Armature resistance [Ohm]
La = 0.00015;           % Armature inductance [H]
Kphi = 0.8;             % Motor constant [Nm/A]

kt = 400;              % torsional stiffness of the drive shaft [Nm/rad]
ct = 4;                % damping coefficient of the drive shaft [Nms/rad]

v0 = 15;                % steady-state vehicle speed [m/s]
v_w0 = 3.5;             % steady-state wind speed [m/s]

%% Equations of motion
m_theta = J_M;
m_x = 4*J_wheel/(Rw^2)+m;
r_theta = (r1+ct)/(tau_1^2);
r_x = (r2*tau_2^2 + ct*tau_2^2)/(Rw^2) + C_RR*k_RR*m*g + rho_air*Cx*A_front*(v0 + v_w0);
r_theta_x = -tau_2/tau_1*ct/Rw;
k_theta = kt/(tau_1^2);
k_x = kt*tau_2^2/(Rw^2);
k_theta_x = -tau_2/tau_1*kt/Rw;

M = [m_theta  0
        0    m_x];

R = [ r_theta  r_theta_x
     r_theta_x    r_x   ];

K = [ k_theta  k_theta_x
     k_theta_x    k_x   ];

%% Transfer functions matrix
s = tf('s');
E = M*s^2 + R*s + K;
G = inv(E);

figure;
bode(G(1,1));
title('Bode plot of the transfer function \theta/T_M');
figure;
bode(s*G(2,1));
title('Bode plot of the transfer function v/T_M');