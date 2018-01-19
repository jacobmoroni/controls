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
% P.ki_z = -0.1;

% PD designs for inner loop
P.ze = P.l/2; %equilibrium pos
b0 = P.l/(P.m2*P.l^2/3+P.m1*P.ze^2);
tr_theta = tr_z/M;  % rise time for inner loop
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
% P.kp_th  = wn_th^2/b0; % kp - inner loop
% P.kd_th  = 2*zeta_th*wn_th/b0; % kd - inner loop

%DC gain for inner loop
k_DC_th = 1;

%PD design for outer loop
wn_z   = 2.2/tr_z;
% P.kp_z = -wn_z^2/P.g;
% P.kd_z = -2*zeta_z*wn_z/P.g;

%adjusted gains for hw 17
P.kp_th = 126.7419;
P.kd_th = 9.7753;
P.kp_z  = -0.3430;
P.kd_z  = -0.2645;
P.ki_z  = -0.1000;


% transfer functions for inverted pendulum
P_in = tf([b0],[1,0,0]);
P_out = tf([-P.g],[1,0,0]);
C_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
C_out = tf([(P.kd_z+P.kp_z*P.sigma),(P.kp_z+P.ki_z*P.sigma),P.ki_z],[P.sigma,1,0]);


figure(1), clf, margin(P_in*C_in), grid on, hold on
bode(P_in*C_in/(1+P_in*C_in)) 
margin(P_out*C_out)
bode(P_out*C_out/(1+P_out*C_out))
legend('Open Loop-Inner', 'Closed Loop-Inner','Open Loop-Outer', 'Closed Loop-Outer')

[m,p] = bode (C_out*P_out,.6); %this pulls magnitude and phase at .6

%bandwidth of inner loop is approximately 43 rad/s which is slightly larger
%than the crossover frequency of 26.7

%bandwidth of outer loop is approximately 4.3 rad/s which is slightly larger
%than the crossover frequency of 2.87

%the bandwidth separation is roughprximadamete 10 which justifies using
%successive loop closure design approach
