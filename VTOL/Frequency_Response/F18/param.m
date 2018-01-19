% VTOL Params
% clear all;

% system parameters known to controller
P.mc = 1;  % kg
P.Jc = 0.0042;     % kg m^2
P.mr = 0.25; % kg
P.ml = 0.25; % kg
P.d = 0.3; % m
P.mu = 0.1; %kg/s
P.g = 9.81; % m/s^2


% initial conditions
P.z0 = 3;
P.zdot0 = 0;
P.theta0 = 0;
P.thetadot0 = 0;
P.h0 = 3;
P.hdot0 = 0;

% sample rate of controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;

% Saturation Limits
F_eq = (P.mc+2*P.mr)*P.g;
P.F_max = 20;
P.Tau_max = 10*P.d;

P_lon = tf([1/(P.mc+2*P.mr)],[1 0 0]);


P_lat_in = tf([1/(P.Jc+2*P.mr*P.d^2)],[1 0 0]);

F0 = F_eq;
P_lat_out = tf([(-F0/(P.mc+2*P.mr))],[1 (P.mu/(P.mc+2*P.mr)) 0]);


