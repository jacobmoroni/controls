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

F_eq = (P.m1*P.l1 - P.m2*P.l2)*(P.g/P.l1);
theta_e = 0.0;
%force_eq = (P.m1*P.l1-P.m2*P.l2)*cos(theta)*P.g/P.l1;
b0psi = (P.l1*F_eq)/(P.m1*P.l1^2 + P.m2*P.l2^2 +P.Jx);
b0phi = 1/P.Jx;
b0th = 1.152;

% sample rate of controller
P.Ts = 0.01;

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

%Longitudinal State Space
A_lon = [0 1;...
    ((P.m1*P.l1-P.m2-P.l2)*P.g*sin(theta_e))/(P.m1*P.l1^2+P.m2*P.l2^2+P.Jy) 0];
B_lon = [0; P.l1/(P.m1*P.l1^2+P.m2*P.l2^2+P.Jy)];
C_lon = [1 0];

%Lateral State Space
A_lat = [0 0 1 0; 0 0 0 1; 0 0 0 0;...
    P.l1*F_eq/(P.m1*P.l1^2+P.m2*P.l2^2+P.Jz) 0 0 0];
B_lat = [0;0;1/P.Jx;0];
C_lat = [1 0 0 0;0 1 0 0];

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

% Phi
tr_phi = tr_th/M;%0.3; 
wn_phi = 2.2/tr_phi;

% Psi
tr_psi = tr_phi;%3;%1.5
wn_psi = 2.2/tr_psi;

zeta_th = 0.707;
zeta_phi = zeta_th;
zeta_psi = .707;
integrator_pole_lat = -10;%-wn_psi/2;
integrator_pole_lon = -10;%-wn_th/2;

%Compute Gains
des_char_poly_lat = conv(conv([1 2*zeta_psi*wn_psi wn_psi^2],...
    [1 2*zeta_phi*wn_phi wn_psi^2]),poly(integrator_pole_lat));
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
%Find Feedback Gain K 

% des_char_poly_lat = conv([1 2*zeta_psi*wn_psi wn_psi^2],...
%     [1 2*zeta_phi*wn_phi wn_psi^2]);
% des_poles_lat = roots(des_char_poly_lat);
% 
% des_char_poly_lon = [1 2*zeta_th*wn_th wn_th^2];
% des_poles_lon = roots(des_char_poly_lon);
% 
% CC = ctrb(A_lat,B_lat);
% r = rank(CC);
% 
% P.K_lat = place(A_lat,B_lat,des_poles_lat);
% 
% %find kr
% P.kr_lat = -1/(C_lat(1,:)*pinv(A_lat-B_lat*P.K_lat)*B_lat);
% 
% %Find Feedback Gain K 
% 
% CC = ctrb(A_lon,B_lon);
% r = rank(CC);
% 
% P.K_lon = place(A_lon,B_lon,des_poles_lon);
% 
% %find kr
% P.kr_lon = -1/(C_lon*pinv(A_lon-B_lon*P.K_lon)*B_lon);

