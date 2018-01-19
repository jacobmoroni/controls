% Mass Spring Params

% system parameters known to controller
P.m = 5;  % kg
P.k = 3;     % kg/ s^2
P.b = 0.5; % N m

% sample rate
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.100;

% saturation constraint
P.force_max = 2;
%force_max = P.force_max-P.force_e;

% initial conditions
P.z0 = 0;
P.zdot0 = 0;

% Gains
tr = 2.5; %original
% tr = 2.5; %tuned
zeta = .707;
%P.ki = 1.5;
Delta_ol = [1,P.b/P.m,0];%P.k/P.m];
wn = 2.2/tr;
Delta_cl_d = [1, 2*zeta*wn,wn^2];
% P.kp = P.m*(Delta_cl_d(3)-Delta_ol(3));%-P.k;
% P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));
P.ki = 0.5000;
P.kp = 3.8720;
P.kd = 5.7216;

% transfer functions for inverted pendulum
P_loop = tf(1/P.m,Delta_ol);

C_PID = tf([(P.kd+P.kp*P.sigma),(P.kp+P.ki*P.sigma),P.ki],[P.sigma,1,0]);


figure(1), clf, margin(P_loop*C_PID), grid on, hold on
bode(P_loop*C_PID/(1+P_loop*C_PID))

% ramp = tf(1,[1, 0, 0]);
% figure(2), clf, 
% bode(P_loop), grid on
% hold on
% bode(series(C_PID,P_loop))
% bode(ramp)

legend('Open Loop', 'Closed Loop')
% title('Mass Spring Damper')

system_type = 2;
e_ss = P.k/P.ki;

%bandwidth frequency is when the closed loop magnitude passes through -3dB
%or .707

%bandwith frequency is appox. 1.9 which is slightly larger than the
%crossover freq. of 1.3



