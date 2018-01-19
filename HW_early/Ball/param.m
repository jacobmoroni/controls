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

% sample rate of controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;

% input constraint
P.F_max = 15;

% tunning parameters
tr_z = 2;%5;  % rise time for outer loop   - Can't really push saturation limits and remain stable.  
zeta_z   = 0.707; % damping ratio for outer loop
M = 10;    % time scale separation between inner and outer loop
zeta_th  = 0.707; % damping ratio for inner loop

% PD gains for inner loop
tr_theta = tr_z/M;  % rise time for inner loop
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
%P.kp_th  =  % kp - inner loop
%P.kd_th  =  % kd - inner loop


%Gainz
P.kp_th = 1.8251;   %calcuclated
P.kd_th = 1.173;   %calcuclated
P.kp_z  = -.0049;  %calcuclated
P.kd_z  = -.0317; %calcuclated


% P.kp_th = 2;   %calcuclated
% P.kd_th = 1.5;   %calcuclated
% P.kp_z  = -.01;  %calcuclated
% P.kd_z  = -.1; %calcuclated

% DC gain for inner loop
% k_DC_th = P.kp_th/((P.m1+P.m2)*P.g+P.kp_th);

%PD design for outer loop
wn_z     = 2.2/tr_z; % natural frequency for outer loop
% P.kp_z   = P.m2*wn_z^2/P.m1/P.g/k_DC_th; % kp - outer loop
% P.kd_z   = (2*zeta_z*wn_z*P.m2-P.b)/P.m1/P.g/k_DC_th; % kd - outer loop
% P.kp_z = wn_z^2/P.g/k_DC_th;
% P.kd_z = 2*zeta_z*wn_z/P.g/k_DC_th;

% select integrator gain
P.ki_z = -0.0001;
% P.ki_z = 0.0001;