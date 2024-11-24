
% Francesco Paparazzo ** AY 2024-25 ** 

% CONTROL OF MECHANICAL SYSTEMS ** Prof. Edoardo Sabbioni 

% Introduction to Matlab & Simulink

clear all
close all
clc

%% State Space and TF representation

m = 1; % kg
k = 1; % N/m
r = 10; % N*s/m

w0 = sqrt(k/m); % natural frequency
h = r/(2*m*w0); % damping ratio
alfa = h*w0;

A = [-r/m, -k/m; 1, 0];
B = [1/m; 0];

% y = Cx + D output 
C = eye(2);
D = zeros(2,1);
% ----> y = x

% lam1 = -alfa + w0*sqrt(h^2-1);
% lam2 = -alfa - w0*sqrt(h^2-1);

lambda = eig(A); % find the eigenvalues of A

figure(1)
plot(real(lambda),imag(lambda),'bx'); 
title('eigenvalues of uncontrolled system')
xlabel('Real lambda')
ylabel('Imaginary lambda')
grid on;

[Num,Den] = ss2tf(A,B,C,D,1); % find numerator and denominator of system's TF (that is a matrix), from the iu-th input (1st and only input in this case)

Gp = tf(Num(2,:),Den); % build up TF, denominator is determined by the system, 2 represents position, 1 speed since [x_dot x]' is the state

%_______ Bode Plots ______________________________________________________%

f = logspace(-2,2,1000); % like linspace but for logarithmic scale, 1000 values between 10^-2 and 10^2
w = 2*pi*f;

% Position Bode Plot

[Mag,Phase] = bode(Gp,w);
Mag = squeeze(Mag); % remove singleton dimensions (=1)
Phase = squeeze(Phase);

figure(2)
subplot(2,1,1);
semilogx(w,20*log10(Mag)); grid on;
xlabel('$f$ [Hz]','Interpreter','LaTex')
ylabel('$\left|\frac{x}{u}\right|$ [dB]','Interpreter','LaTex')

subplot(2,1,2);
semilogx(w,Phase); grid on;
xlabel('$f$ [Hz]','Interpreter','LaTex')
ylabel('$\angle\frac{x}{u}$ [deg]','Interpreter','LaTex')

figure(3)
asymp(Gp) % just a more clear way to draw the bode

%% Proportional controller

Kp = 100;

R = Kp;

Loop = series(R,Gp);
CL = feedback(Loop,1); % if you think of the block diagram you have L in the upper part and in the lower the part the line that just goes back (1)

% [Num,Den] = tfdata(CL); % it finds num and den of the TF
% [Ap,Bp,Cp,Dp] = tf2ss(Num{1},Den{1}); % from TF to system
% omega_c = eig(Ap);

% kc = k + Kp;
% wc = sqrt(kc/m);
% hc = r/(2*m*wc);
% alfac = hc*wc;
% 
% lam1c = -alfac + wc*sqrt(hc^2-1);
% lam2c = -alfac - wc*sqrt(hc^2-1);


% figure(4)
% plot(real(omega_c),imag(omega_c),'rx');
% title('eigenvalues of controlled system')
% xlabel('Real lambda')
% ylabel('Imaginary lambda')
% grid on;

%% System response to a step input with Simulink

x_ref = 1;
Tf = 1;

out = sim('MSD_Simulink.slx'); % in this way we connect to simulink 
time = out.time.Time;
Out = out.SimOut_MSD;
position = Out(:,2);

figure(5)
plot(time,position,'r');
hold on
plot(time,x_ref*linspace(1,1,length(time)),'b')
grid on;
xlabel('$t$ [s]','Interpreter','LaTex')
ylabel('position [m]','Interpreter','LaTex')
legend('x','xref');

%% Easy way 

den = [m r k];
num = 1;
G = tf(num,den);
R = Kp;

GH = G*R;

figure
bode(GH)

figure
margin(GH)

figure
rlocus(GH) 

figure
nyquist(GH)

L = feedback(GH,1);

figure
step(L)


