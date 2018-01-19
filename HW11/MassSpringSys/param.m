% Mass Spring Params

% system parameters known to controller
P.m = 5;  % kg
P.k = 3;     % kg/ s^2
P.b = 0.5; % N m

% sample rate
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.1;

% saturation constraint
P.force_max = 2;
%force_max = P.force_max-P.force_e;

% initial conditions
P.z0 = 0;
P.zdot0 = 0;

% Gains
tr = 2; %original
% tr = 2.5; %tuned
zeta = .707;
P.ki = 1.5;
Delta_ol = [1,P.b/P.m,0];
wn = 2.2/tr;
Delta_cl_d = [1, 2*zeta*wn,wn^2];
P.kp = P.m*(Delta_cl_d(3)-Delta_ol(3));%-P.k;
% P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));
P.kp = (24.2/(tr^2)-3);
P.kd = (15.4/tr)-.5;
% P.kp = 2;
% P.kd = 6.5;
% P.ki = 1.;

%State Space Matricies
A = [0 1;-P.k/P.m -P.b/P.m];
B = [0; 1/P.m];
C = [1 0];
D = 0;

%Find Feedback Gain K 
Proots = roots(Delta_cl_d);

CC = ctrb(A,B);
r = rank(CC);

P.K = place(A,B,Proots);

%find kr
P.kr = -1/(C*pinv(A-B*P.K)*B);
%DC_gain = P.kp/(P.kp+P.k);

