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
tr = 2.0; %original
% tr = 2.5; %tuned
zeta = .707;
P.ki = 10;%1.5;
Delta_ol = [1,P.b/P.m,P.k/P.m];
wn = 2.2/tr;
Delta_cl_d = [1, 2*zeta*wn,wn^2];
P.kp = P.m*(Delta_cl_d(3)-Delta_ol(3));%-P.k;
P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));
% P.kp = (24.2/(tr^2)-3);
% P.kd = (15.4/tr)-.5;
% P.kp = 2;
% P.kd = 6.5;
% P.ki = 1.5;


% transfer functions for inverted pendulum
P_loop = tf(1/P.m,Delta_ol);

C_PID = tf([(P.kd+P.kp*P.sigma),(P.kp+P.ki*P.sigma),P.ki],[P.sigma,1,0]);

%ramp = tf(1,[1, 0, 0]);
figure(2), clf, 
bode(P_loop), grid on
hold on
bode(series(C_PID,P_loop))
%bode(ramp)

legend('No control', 'PID')
title('Mass Spring Damper')

system_type = 2;
e_ss = P.k/P.ki;

% D = P_loop/(1+P_loop*C_PID);

figure(2)
% bode(D,{0.001,1000})

[m,p] = bode (C_PID*P_loop,.1); %this pulls magnitude and phase at .6
