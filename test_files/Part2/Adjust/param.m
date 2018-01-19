% Gantry Crane parameter file
clear all

% system parameters known to controller
P.mc = 1500; %kg
P.m = 500; %kg
P.b = 200; %N-s/m
P.L = 10; %m
P.g = 9.81; %m/s^2

% initial conditions
P.x0 = 0;
P.xdot0 = 0;
P.th0 = 0;
P.thdot0 = 0;

% input constraint
P.F_max = 20000;

% sample rate for controller
P.Ts = 0.01;

% gain for dirty derivative
P.sigma = 0.05;

% state space design
P.A =   [-P.b/P.mc  0  0  P.m*P.g/P.mc;...
         P.b/(P.mc*P.L)  0  0  -(P.mc+P.m)*P.g/(P.mc*P.L);...
         1  0  0  0;...
         0  1  0  0];
P.B = [1/P.mc; -1/(P.mc*P.L); 0; 0];
P.C = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    ];

% tunning parameters
% M = 10; %difference between inner and outer loops
tr_x = 2.0;  % rise time for position 
tr_th = 2.0; %tr_x/M; % rise time for angle
zeta_x   = 0.7; % damping ratio position
zeta_th  = -0.293; % damping ratio angle

% gain calculation
wn_th    = 2.2/tr_th; % natural frequency for angle
wn_x     = 2.2/tr_x; % natural frequency for position
des_char_poly = conv([1,2*zeta_x*wn_x,wn_x^2],...
                     [1,2*zeta_th*wn_th,wn_th^2]);
des_poles = roots(des_char_poly);
% is the system controllable?
if rank(ctrb(P.A,P.B))~=4, disp('System Not Controllable'); end
P.K = place(P.A,P.B,des_poles);
P.kr = -1/(P.C(1,:)*inv(P.A-P.B*P.K)*P.B);

% observer design
wn_th_obs   = 5*wn_th;
wn_x_obs    = 5*wn_x;
des_obsv_char_poly = conv([1,2*zeta_x*wn_x_obs,wn_x_obs^2],...
                      [1,2*zeta_th*wn_th_obs,wn_th_obs^2]);
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A,P.C))~=4, 
    disp('System Not Observable'); 
else % if so, compute gains
    P.Lo = place(P.A', P.C', des_obsv_poles)';
end




