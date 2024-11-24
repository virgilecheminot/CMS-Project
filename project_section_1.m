%% System data
g = 9.81;               % gravity acceleration [m/s^2]
M = 1500;               % vehicle mass [kg]
J_wheel = 1;            % moment of inertia of the wheel [kg*m^2]
J_M = 0.05;             % moment of inertia of the motor [kg*m^2]
alpha = deg2rad(5);     % slope of the road [degrees]
R = 0.35;               % wheel radius [m]
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

m_star = J_M*(tau_1*tau_2/R)^2+4*J_wheel/(R^2)+M;
r_star = (r1+r2)*(tau_2/R)^2;
fprintf('m*=%f ; r*=%f\n',m_star,r_star);

%% Compute the steady-state T_M0 :
v0 = 15;                % steady-state vehicle speed [m/s]
v_w0 = -3.5;             % steady-state wind speed [m/s]

ratio = R/(tau_2*tau_1);
F_D0 = 1/2*rho_air*Cx*A_front*(v0-v_w0)^2;
F_R0 = M*g*C_RR*(1+k_RR*v0);
P = M*g*sin(alpha);
damp_0 = r_star*v0;

T_M0 = ratio*(F_D0 + F_R0 + P + damp_0);
fprintf('Steady-state motor torque : %f N*m\n',T_M0);

%% Write the final equation of motion
r_gen = r_star + C_RR*k_RR*M*g+rho_air*Cx*A_front*(v0-v_w0);

num_G = tau_1*tau_2/R;
den_G = [m_star,r_gen];

num_D = rho_air*Cx*A_front*(v0-v_w0);
den_D = den_G;

G = tf(num_G,den_G);
D = tf(num_D,den_D);

disp(pole(G));
asymp(G);