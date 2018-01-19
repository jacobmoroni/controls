clear;
clc;

% system parameters known to controller
P.mc    = 1;  % kg
P.Jc    = 0.0042; % kg/m^2
P.mr    = 0.25; %kg
P.ml    = 0.25; %kg
P.d     = 0.3; %m
P.mu    = 0.1; %kg/s
P.g     = 9.81; %m/s2

% initial conditions
P.z0        = 3;
P.zdot0     = 0;
P.theta0    = 0;
P.thetadot0 = 0;
P.h0        = 3;
P.hdot0     = 0;

% sample rate of controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;

% Saturation Limits
force_eq = (P.mc+2*P.mr)*P.g;
P.forcemax = 20-force_eq;
P.taumax = 10*P.d;

%%%% Gains

%P.kd = .75;
%P.kp = .0928;

trh = 2.62;
wnh = 2.2/trh;
zetah = 0.707;
% P.kd_h = (2*zetah*wnh)*1.5;%/.667;
% P.kp_h = (wnh^2)*1.5; %/.667;
P.kd_h = 1.7810;
P.kp_h = 1.0576;
P.ki_h = 0.25;

% P.kd_h = 0.583;
% P.kp_h = 0.1134;

trz = 2.62;
wnz = 2.2/trz;
zetaz = 0.707;
% P.kd_z = (-2*zetaz*wnz+.0667)/P.g;
% P.kp_z = -(wnz^2)/P.g;
P.kd_z = -0.1142;
P.kp_z = -0.0719;
P.ki_z = 0.0;

trt = trz/10;
zetat = 0.707;
wnt = 2.2/trt;
% P.kd_t = 2*zetat*wnt/20.3252;
% P.kp_t = (wnt^2)/20.3252;
P.kd_th = 0.5842;
P.kp_th = 3.4690;

% P.kp_h  = 0.113438; %calculated values
% P.kd_h  = 0.583275; %calculated values
% P.kp_th = 0.372075; %calculated values
% P.kd_th = 0.191314; %calculated values
% P.kp_z  =-.007709;  %calculated values
% P.kd_z  =-.0328;    %calculated values



