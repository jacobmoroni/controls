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
P.forcemax = 20;%-force_eq;
P.taumax = 10*P.d;

%Mixing matrix
% P.mixing = inv([1, 2*zeta_h*wn_h,wn_h^2]);

%Tuning Parameters
tr_h = 8; %altitude rise time -- original
% tr_h = 2.62; %tuned for fastest performance w/o sat
zeta_h = 0.707;
tr_z = 8; %rise time for outer lateral loop -- orig
% tr_z = 2.62;
% tr_z = 3; %fastest w/o sat(Mcclain tuned)
M = 10; %time separation between inner and outer loop
zeta_z = zeta_h;
zeta_th = zeta_h;
P.ki_h = 0.5; %integrator gain for altitude
P.ki_z = 0; %integrator gain for pos z;

%PD gains for longitudinal control
wn_h = 2.2/tr_h;
Delta_cl_d_lon = [1, 2*zeta_h*wn_h, wn_h^2]; %desired CLCE
P.kp_h = Delta_cl_d_lon(3)*(P.mc+2*P.mr);
P.kd_h = Delta_cl_d_lon(2)*(P.mc+2*P.mr);

%PD gains for lateral inner loop
%theta
b0 = 1/(P.Jc+2*P.mr*P.d^2);
tr_th = tr_z/M;
wn_th = 2.2/tr_th;
P.kp_th = wn_th^2/b0;
P.kd_th = 2*zeta_th*wn_th/b0;
%z

wn_z = 2.2/tr_z;
P.kp_z = (wn_z^2*(P.mc+2*P.mr))/-force_eq;
P.kd_z = (2*zeta_z*wn_z*(P.mc+2*P.mr)-P.mu)/-force_eq;

%DC gains for inner loop
k_dc_th = 1;


%%%% Gains
% P.kd_h = 1.7810;
% P.kp_h = 1.0576;
% P.ki_h = 0.25;

% P.kd_h = 0.583;
% P.kp_h = 0.1134;

% P.kd_z = -0.1142;
% P.kp_z = -0.0719;
% P.ki_z = 0.0;


% P.kp_h  = 0.113438; %calculated values
% P.kd_h  = 0.583275; %calculated values
% P.kp_th = 0.372075; %calculated values
% P.kd_th = 0.191314; %calculated values
% P.kp_z  =-.007709;  %calculated values
% P.kd_z  =-.0328;    %calculated values

%State Space Matricies Lateral
Delta_cl_d_lat = conv([1 2*zeta_z*wn_z wn_z^2],[1 2*zeta_th*wn_th wn_th^2]);
A_lat = [0 0 1 0;0 0 0 1;0 -force_eq/(P.mc+2*P.mr) -P.mu/(P.mc+2*P.mr) 0;0 0 0 0];
B_lat = [0; 0; 0; b0];
C_lat = [1 0 0 0; 0 1 0 0];

%Find Feedback Gain K 
Proots = roots(Delta_cl_d_lat);

CC = ctrb(A_lat,B_lat);
r = rank(CC);

P.K_lat = place(A_lat,B_lat,Proots);

%find kr
P.kr_lat = -1/(C_lat(1,:)*pinv(A_lat-B_lat*P.K_lat)*B_lat);

%State Space Matricies Longitudina
A = [0 1; 0 0];
B = [0; 1/(P.mc+2*P.mr)];
C = [1 0];
D = 0;

%Find Feedback Gain K 
Proots = roots(Delta_cl_d_lon);

CC = ctrb(A,B);
r = rank(CC);

P.K_lon = place(A,B,Proots);

%find kr
P.kr_lon = -1/(C*pinv(A-B*P.K_lon)*B);


