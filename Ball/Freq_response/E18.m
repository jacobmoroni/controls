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


% PD designs for inner loop
P.ze = P.l/2; %equilibrium pos
b0 = P.l/(P.m2*P.l^2/3+P.m1*P.ze^2);

%tuning Parameters inner loop
ki = 1.2;
p_LPF = 1000;
p_Lead = 150;
z_Lead = 7;
kp = 80;
z_Lag = 1;
p_Lag = .0001;
%inner loop controllers
C_PI = tf([1,ki],[1, 0]);
C_LPF = tf(p_LPF,[1, p_LPF]);
C_Lead = tf([p_Lead*1,p_Lead*z_Lead],[z_Lead*1,z_Lead*p_Lead]);
C_Lag = tf([1,z_Lag],[1,p_Lag]);

%tuning Parameters outer loop
ki_o = .1;
p_LPF_o = 1000;
p_Lead_o = 150;
z_Lead_o = 7;
kp_o = 1;
z_Lag_o = 1;
p_Lag_o = 1;
%outer loop controllers
C_PI_o = tf([1,ki_o],[1, 0]);
C_LPF_o = tf(p_LPF_o,[1, p_LPF_o]);
C_Lead_o = tf([p_Lead_o*1,p_Lead_o*z_Lead_o],[z_Lead_o*1,z_Lead_o*p_Lead_o]);
C_Lag_o = tf([1,z_Lag_o],[1,p_Lag_o]);


% transfer functions for inverted pendulum
P_in = tf([b0],[1,0,0]);
P_out = tf([-P.g],[1,0,0]);
C_in = kp*C_PI*C_LPF*C_Lead;%*C_Lag;
C_out = 1;%kp_o*C_PI_o;%*C_LPF_o*C_Lead_o;%*C_Lag_o;
Plant = P_out*(P_in*C_in)/(1+P_in*C_in);
% % ctrlpref to change settings
figure(1), clf, 
bode(P_in), grid on
hold on
bode(series(C_in,P_in))

margin(P_in*C_in)

[m_1_in,p] = bode (C_in*P_in,1) %must be at least 312.5
[m_1000_in,p] = bode (C_in*P_in,1000) %must be at most .0032

%Plot outer loop bode plot
%important note, Outer loop plant already met specifications without tuning
%so I did not apply a controller to it
figure(2), clf, 
bode(Plant), grid on
hold on
bode(series(C_out,Plant))
margin(Plant*C_out)

[m_01_o,p] = bode (C_out*Plant,.1) %must be at least 100
[m_100_o,p] = bode (C_out*Plant,100) %must be at most .001

