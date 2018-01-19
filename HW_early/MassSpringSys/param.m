% Mass Spring Params

% system parameters known to controller
P.m = 5;  % kg
P.k = 3;     % kg/ s^2
P.b = 0.5; % N m

% sample rate
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;

% saturation constraint
P.force_max = 2;
%force_max = P.force_max-P.force_e;

% initial conditions
P.z0 = 0;
P.zdot0 = 0;

% Gains
tr = 2.202;
% P.kp = (24.2/(tr^2)-3);
% P.kd = (15.4/tr)-.5;
P.kp = 2;
P.kd = 6.5;
P.ki = 2.25;

