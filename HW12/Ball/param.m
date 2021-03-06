clear;
clc;
%Ball Beam System
% system parameters known to controller
P.m1    = 0.35;    % kg
P.m2    = 2;       % kg
P.l     = 0.5;     % m
P.g     = 9.81;    % m/s^2
P.ze = P.l/2;      %equilibrium pos

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

%State Space Matricies
b0bot = (P.m2*P.l^2/3+P.m1*P.ze^2);
b0 = P.l/(P.m2*P.l^2/3+P.m1*P.ze^2);
A = [0 0 1 0;0 0 0 1;0 -P.g 0 0;-P.m1*P.g/b0bot 0 0 0];
B = [0; 0; 0; b0];
C = [1 0 0 0; 0 1 0 0];
D = 0;

% form augmented system
Cout = [1,0,0,0];
A1 = [A, zeros(4,1); -Cout, 0];
B1 = [B; 0];

% tunning parameters
%tr_z = 10;%5;  % rise time for outer loop
tr_z = 1.8;%tuned for fastest rise time without saturation
M = 10;    % time scale separation between inner and outer loop
tr_theta = tr_z/M;  % rise time for inner loop
zeta_z   = 0.707; % damping ratio for outer loop
zeta_th  = 0.707; % damping ratio for inner loop
integrator_pole = -10;

%compute gains
wn_z   = 2.2/tr_z; %outer loop
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
des_char_poly = conv(conv([1 2*zeta_z*wn_z wn_z^2],...
                [1 2*zeta_th*wn_th wn_th^2]),poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=5
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:4);
    P.ki = K1(5);
end