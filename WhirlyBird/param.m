% Whirlybird Parameters

% Constant Parameters
P.g = 9.81; % m/s/s
P.l1 = 0.85; % m
P.l2 = 0.3048; % m
P.m1 = 0.891; % kg
P.m2 = 1; % kg
P.d = 0.178; % m
P.h = 0.65; % m
P.r = 0.12; % m
P.Jx = 0.0047; % kg m^2
P.Jy = 0.0014; % kg m^2
P.Jz = 0.0041; % kg m^2
P.km = 5.8; % N/PWM 
P.siggyro = 8.7266*10^-5; %rad
P.sigpix = 0.05; % pix

zeta = 0.707;
F_eq = (P.m1*P.l1 - P.m2*P.l2)*(P.g/P.l1);
%force_eq = (P.m1*P.l1-P.m2*P.l2)*cos(theta)*P.g/P.l1;
b0psi = (P.l1*Fe)/(P.m1*P.l1^2 + P.m2*P.l2^2 +P.Jx);
b0phi = 1/P.Jx;
b0th = 1.152;

% sample rate of controller
P.Ts = 0.0033;

% dirty derivative gain
P.sigma = 0.05;

% Saturation Limits
% Saturation
km=6.293;               % Max force per prop
P.forcemax = 2*km;          % Total max force.
P.taumax = 2*km*P.d;


% Initial Conditions
P.phi0 = 0;
P.phidot0 = 0;
P.psi0 = 0;
P.psidot0 = 0;
P.theta0 = 0;
P.thetadot0 = 0;
theta_c = 0;
%Longitudinal State Space
P.A_lon = [0 1;...
    ((P.m1*P.l1-P.m2-P.l2)*P.g*sin(theta_c))/(P.m1*P.l1^2+P.m2*P.l2^2+P.Jy) 0];
P.B_lon = [0; P.l1/(P.m1*P.l1^2+P.m2*P.l2^2+P.Jy)];
P.C_lon = [1 0];

%Lateral State Space
P.A_lat = [0 0 1 0; 0 0 0 1; 0 0 0 0;...
    P.l1*F_eq/(P.m1*P.l1^2+P.m2*P.l2^2+P.Jz) 0 0 0];
P.B_lat = [0;0;1/P.Jx;0];
P.C_lat = [1 0 0 0;0 1 0 0];

%form lateral augmented system
Cout_lat = [0 1 0 0];
A1_lat = [A_lat, zeros(4,1); -Cout_lat, 0];
B1_lat = [B_lat; 0];

%form longitudinal augmented system
Cout_lon = C_lon;
A1_lon = [A_lon, zeros(2,1); -Cout_lon, 0];
B1_lon = [B_lon; 0];

%tuning parameters
M = 10; %time separation between inner and outer loop
% Theta
tr_th = 2.0; %1.5
wn_th = 2.2/tr_th;
% P.kd_th = (2*zeta*wn_th)/b0th;
% P.kp_th = (wn_th^2)/(b0th);
% P.ki_th = .2;

% Phi
tr_phi = 1.5;%tr_th/M;%0.3; 
wn_phi = 2.2/tr_phi;
% P.kd_phi = (2*zeta*wn_phi)/(b0phi);
% P.kp_phi = (wn_phi^2)/b0phi;

% Psi
%zeta_psi = .8; 
tr_psi = tr_th;%3;%1.5
wn_psi = 2.2/tr_psi;
% P.kd_psi = (2*zeta_psi*wn_psi)/b0psi;
% P.kp_psi = (wn_psi^2)/b0psi;
% P.ki_psi = 0.01;%.2;

zeta_th = 0.707;
zeta_phi = 1.0;%zeta_th;
zeta_psi = .8;%zeta_th;
integrator_pole_lat = -15;
integrator_pole_lon = -10;

%Compute Gains
des_char_poly_lat = conv(conv([1 2*zeta_phi*wn_phi wn_phi^2],...
    [1 2*zeta_psi*wn_psi wn_psi^2]),poly(integrator_pole_lat));
des_poles_lat = roots(des_char_poly_lat);

des_char_poly_lon = conv([1 2*zeta_th*wn_th wn_th^2],...
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
wn_phi_obs   = 5*wn_phi;
wn_psi_obs    = 5*wn_psi;
des_obsv_char_poly = conv([1,2*zeta_psi*wn_psi_obs,wn_psi_obs^2],...
                      [1,2*zeta_phi*wn_phi_obs,wn_phi_obs^2]);
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A_lat,P.C_lat))~=4
    disp('System Not Observable'); 
else % if so, compute gains
    P.L_lat = place(P.A_lat', P.C_lat', des_obsv_poles)';
end

% Longitudial observer design
wn_th_obs   = 5*wn_th;
des_obsv_char_poly = [1,2*zeta_th*wn_th_obs,wn_th_obs^2];
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A_lon,P.C_lon))~=2
    disp('System Not Observable'); 
else % if so, compute gains
    P.L_lon = place(P.A_lon', P.C_lon', des_obsv_poles)';
end
