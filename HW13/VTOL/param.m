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

%State Space Matricies Lateral
b0 = 1/(P.Jc+2*P.mr*P.d^2);
P.A_lat = [0 0 1 0;0 0 0 1;...
    0 -force_eq/(P.mc+2*P.mr) -P.mu/(P.mc+2*P.mr) 0;0 0 0 0];
P.B_lat = [0; 0; 0; b0];
P.C_lat = [1 0 0 0; 0 1 0 0];

%State Space Matricies Longitudina
P.A_lon = [0 1; 0 0];
P.B_lon = [0; 1/(P.mc+2*P.mr)];
P.C_lon = [1 0];

%form lateral augmented system
Cout_lat = [1 0 0 0];
A1_lat = [P.A_lat, zeros(4,1); -Cout_lat, 0];
B1_lat = [P.B_lat; 0];

%form longitudinal augmented system
Cout_lon = P.C_lon;
A1_lon = [P.A_lon, zeros(2,1); -Cout_lon, 0];
B1_lon = [P.B_lon; 0];

%tuning parameters
tr_h = 8; %altitude rise time -- original
% tr_h = 2.62; %tuned for fastest performance w/o sat
tr_z = 8; %rise time for outer lateral loop -- orig
% tr_z = 2.62;
% tr_z = 3; %fastest w/o sat(Mcclain tuned)
M = 10; %time separation between inner and outer loop
tr_th = tr_z/M;
zeta_h = 0.707;
zeta_z = zeta_h;
zeta_th = zeta_h;
integrator_pole_lat = -10;
integrator_pole_lon = -10;

%compute gains
wn_z = 2.2/tr_z;
wn_th = 2.2/tr_th;
wn_h = 2.2/tr_h;

des_char_poly_lat = conv(conv([1 2*zeta_z*wn_z wn_z^2],...
    [1 2*zeta_th*wn_th wn_th^2]),poly(integrator_pole_lat));
des_poles_lat = roots(des_char_poly_lat);

des_char_poly_lon = conv([1 2*zeta_h*wn_h wn_h^2],...
    poly(integrator_pole_lon));
des_poles_lon = roots(des_char_poly_lon);

% is the lateral system controllable?
if rank(ctrb(A1_lat,B1_lat))~=5
    disp('System Not Controllable'); 
else % if so, compute gains
    K1_lat   = place(A1_lat,B1_lat,des_poles_lat); 
    P.K_lat  = K1_lat(1:4);
    P.ki_lat = K1_lat(5);
end

% is the longitudinal system controllable?
if rank(ctrb(A1_lon,B1_lon))~=3
    disp('System Not Controllable'); 
else % if so, compute gains
    K1_lon   = place(A1_lon,B1_lon,des_poles_lon); 
    P.K_lon  = K1_lon(1:2);
    P.ki_lon = K1_lon(3);
end

% lateral observer design
wn_th_obs   = 10*wn_th;
wn_z_obs    = 10*wn_z;
des_obsv_char_poly = conv([1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
                      [1,2*zeta_th*wn_th_obs,wn_th_obs^2]);
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A_lat,P.C_lat))~=4
    disp('System Not Observable'); 
else % if so, compute gains
    P.L_lat = place(P.A_lat', P.C_lat', des_obsv_poles)';
end

% Longitudial observer design
wn_h_obs   = 10*wn_h;
des_obsv_char_poly = [1,2*zeta_h*wn_h_obs,wn_h_obs^2];
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A_lon,P.C_lon))~=2
    disp('System Not Observable'); 
else % if so, compute gains
    P.L_lon = place(P.A_lon', P.C_lon', des_obsv_poles)';
end