% Gantry Crane parameter file
clear
clc

% system parameters known to controller
P.mc = 1500; %kg
P.m = 500; %kg
P.b = 200; %N-s/m
P.L = 10; %m
P.g = 9.81; %m/s^2

%sample rate for controller
P.Ts = 0.01;

% initial conditions
P.x0 = 0;
P.th0 = 0;
P.xdot0 = 0;
P.thdot0 = 0;

% input constraint
P.F_max = 20000;

% gain for dirty derivative
P.sigma = 0.05;

% state space design
A = [-P.b/P.mc       0  0  P.m*P.g/P.mc;...
     P.b/(P.mc*P.L)  0  0  -(P.mc+P.m)*P.g/(P.mc*P.L);...
         1           0  0  0;...
         0           1  0  0];
B = [1/P.mc; -1/(P.mc*P.L); 0; 0];
C = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    ];

% tunning parameters
% M = 10; %difference between inner and outer loops
tr_x = 2.5;  % rise time for position 
tr_th = 2.5; %tr_x/M; % rise time for angle
zeta_x   = 0.7; % damping ratio position
zeta_th  = 0.7; % damping ratio angle

% gain calculation
wn_th    = 2.2/tr_th; % natural frequency for angle
wn_x     = 2.2/tr_x; % natural frequency for position
des_char_poly = conv([1,2*zeta_x*wn_x,wn_x^2],...
                     [1,2*zeta_th*wn_th,wn_th^2]);
%des_poles = roots(des_char_poly);
des_poles = [-0.6 + .8*1i; -0.6 - .8*1i;-0.7 + .7*1i; -0.7 - .7*1i];
% is the system controllable?
if rank(ctrb(A,B))~=4, disp('System Not Controllable'); end
P.K = place(A,B,des_poles);
P.kr = -1/(C(1,:)*inv(A-B*P.K)*B);







