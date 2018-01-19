clear;
clc;
%Ball Beam System
% system parameters known to controller
P.m1    = 0.35;    % kg
P.m2    = 2;       % kg
P.l     = 0.5;     % m
P.g     = 9.81;     % m/s^2

% initial conditions
P.z0 = 0.25;
P.zdot0 = 0;
P.theta0 = 0;
P.thetadot0 = 0;

%max force 
P.force_max = 25;
% sample rate of controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;

% input constraint
P.F_max = 15;

% tunning parameters
%tr_z = 10;%5;  % rise time for outer loop
tr_z = 1.5;%tuned for fastest rise time without saturation
zeta_z   = 0.707; % damping ratio for outer loop
M = 10;    % time scale separation between inner and outer loop
zeta_th  = 0.707; % damping ratio for inner loop
P.ki_z = -0.1;

% PD designs for inner loop
P.ze = P.l/2; %equilibrium pos
b0 = P.l/(P.m2*P.l^2/3+P.m1*P.ze^2);
tr_theta = tr_z/M;  % rise time for inner loop
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
P.kp_th  = wn_th^2/b0; % kp - inner loop
P.kd_th  = 2*zeta_th*wn_th/b0; % kd - inner loop

%DC gain for inner loop
k_DC_th = 1;

%PD design for outer loop
wn_z   = 2.2/tr_z;
P.kp_z = -wn_z^2/P.g;
P.kd_z = -2*zeta_z*wn_z/P.g;

%Gains
% P.kp_th = 1.8251;   %calcuclated
% P.kd_th = 1.173;   %calcuclated
% P.kp_z  = -.0049;  %calcuclated
% P.kd_z  = -.0317; %calcuclated

% P.kp_th = 2;   %calcuclated
% P.kd_th = 1.5;   %calcuclated
% P.kp_z  = -.01;  %calcuclated
% P.kd_z  = -.1; %calcuclated

%State Space Matricies
Delta_cl_d = conv([1 2*zeta_z*wn_z wn_z^2],[1 2*zeta_th*wn_th wn_th^2]);
b0bot = (P.m2*P.l^2/3+P.m1*P.ze^2);
A = [0 0 1 0;0 0 0 1;0 -P.g 0 0;-P.m1*P.g/b0bot 0 0 0];
B = [0; 0; 0; b0];
C = [1 0 0 0; 0 1 0 0];
D = 0;

%Find Feedback Gain K 
Proots = roots(Delta_cl_d);

CC = ctrb(A,B);
r = rank(CC);

P.K = place(A,B,Proots);

%find kr
kr = -1/(C*pinv(A-B*P.K)*B);
P.kr = kr(1);