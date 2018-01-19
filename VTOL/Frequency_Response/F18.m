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
P.z0        = 0;
P.zdot0     = 0;
P.theta0    = 0;
P.thetadot0 = 0;
P.h0        = 0;
P.hdot0     = 0;

% sample rate of controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;

% Saturation Limits
force_eq = (P.mc+2*P.mr)*P.g;
P.F_eq = force_eq;
P.F_max = 20;%-force_eq;
P.Tau_max = 10*P.d;

b0 = 1/(P.Jc+2*P.mr*P.d^2);

%%
%tuning longitudinal
%Proportional Gain
kp_lon = 1.5;
%Integral Control
ki_lon = .001;
%Low pass filter
p_LPF_lon = 15;
%Phase Lead Filter
p_Lead_lon = 20;
z_Lead_lon = .5;
%Phase Lag Filter
z_Lag_lon = 1;
p_Lag_lon = .0001;
%longitudinal controllers
C_PI_lon = tf([1,ki_lon],[1, 0]);%Integral Control
C_LPF_lon = tf(p_LPF_lon,[1, p_LPF_lon]); %Low Pass Filter
C_Lead_lon = tf([p_Lead_lon*1,p_Lead_lon*z_Lead_lon]...
    ,[z_Lead_lon*1,z_Lead_lon*p_Lead_lon]); %Phase Lead Filter
C_Lag_lon = tf([1,z_Lag_lon],[1,p_Lag_lon]);%Phase Lag Filter

P_lon = tf([1/(P.mc+2*P.mr)],[1,0,0]);
C_lon = kp_lon*C_PI_lon*C_LPF_lon*C_Lead_lon;

%plot bode for tuning and checking
figure(1), clf, 
bode(P_lon), grid on
hold on
bode(series(C_lon,P_lon))

margin(P_lon*C_lon)

[m_01_lon,p] = bode (C_lon*P_lon,.1) %must be at least 100
[m_200_lon,p] = bode (C_lon*P_lon,200) %must be at most .0001
%%
%tuning inner loop of lateral
%Proportional Gain
kp_lat_in = .217;
%Integral Control
ki_lat_in = .001;
%Low pass filter
p_LPF_lat_in = 50;
%Phase Lead Filter
p_Lead_lat_in = 50;
z_Lead_lat_in = .6;
%Phase Lag Filter
z_Lag_lat_in = 1;
p_Lag_lat_in = .0001;
%longitudinal controllers
C_PI_lat_in = tf([1,ki_lat_in],[1, 0]);%Integral Control
C_LPF_lat_in = tf(p_LPF_lat_in,[1, p_LPF_lat_in]); %Low Pass Filter
C_Lead_lat_in = tf([p_Lead_lat_in*1,p_Lead_lat_in*z_Lead_lat_in]...
    ,[z_Lead_lat_in*1,z_Lead_lat_in*p_Lead_lat_in]); %Phase Lead Filter
C_Lag_lat_in = tf([1,z_Lag_lat_in],[1,p_Lag_lat_in]);%Phase Lag Filter

%transfer funcitons for lateral inner loop control
P_lat_in = tf([b0],[1,0,0]);
C_lat_in = kp_lat_in*C_PI_lat_in*C_LPF_lat_in*C_Lead_lat_in;

%plot bode for tuning and checking
figure(2), clf, 
bode(P_lat_in), grid on
hold on
bode(series(C_lat_in,P_lat_in))

margin(P_lat_in*C_lat_in)

%crossover frequency of about 1 and add a Low Pass Filter to reduce noise
[m_10_lat_in,p] = bode (C_lat_in*P_lat_in,10) %should be about .707
[m_200_lat_in,p] = bode (C_lat_in*P_lat_in,200) %try to minimize this, but not critical
%%
%tuning outer loop of lateral 
%Proportional Gain
kp_lat_out = 1;
%Integral Control
ki_lat_out = .01;
%Low pass filter
p_LPF_lat_out = 50;
%Phase Lead Filter
p_Lead_lat_out = 30;
z_Lead_lat_out = .5;
%Phase Lag Filter
z_Lag_lat_out = 100;
p_Lag_lat_out = 1;
%longitudinal controllers
C_PI_lat_out = tf([1,ki_lat_out],[1, 0]);%Integral Control
C_LPF_lat_out = tf(p_LPF_lat_out,[1, p_LPF_lat_out]); %Low Pass Filter
C_Lead_lat_out = tf([p_Lead_lat_out*1,p_Lead_lat_out*z_Lead_lat_out]...
    ,[z_Lead_lat_out*1,z_Lead_lat_out*p_Lead_lat_out]); %Phase Lead Filter
C_Lag_lat_out = tf([1,z_Lag_lat_out],[1,p_Lag_lat_out]);%Phase Lag Filter

%transfer funcitons for lateral outer loop control
P_lat_out = tf([-force_eq/(P.mc+2*P.mr)],[1,P.mu/(P.mc+2*P.mr),0]);
C_lat_out = kp_lat_out*C_PI_lat_out*C_LPF_lat_out;%*C_Lag_lat_out;
P_lat_out2 = P_lat_out*(P_lat_in*C_lat_in)/(1+P_lat_in*C_lat_in);

%plot bode for tuning and checking
figure(3), clf, 
bode(P_lat_out), grid on
hold on
bode(series(C_lat_out,P_lat_out2))

margin(P_lat_out2*C_lat_out)

[m_1_lat_out,p] = bode (C_lat_out*P_lat_out2,1) %should be about 1
[m_100_lat_out,p] = bode (C_lat_out*P_lat_out2,100) %must be at most .00001