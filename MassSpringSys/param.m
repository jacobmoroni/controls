% Mass Spring Params
clear
% system parameters known to controller
P.m = 5;  % kg
P.k = 3;     % kg/ s^2
P.b = 0.5; % N m

% sample rate
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.1;

% saturation constraint
P.F_max = 2;

%force_max = P.force_max-P.force_e;

% initial conditions
P.z0 = 0;
P.zdot0 = 0;

%State Space Matricies
P.A = [0 1;-P.k/P.m -P.b/P.m];
P.B = [0; 1/P.m];
P.C = [1 0];

%find augmented system
Cout = P.C;
A1 = [P.A, zeros(2,1); -Cout, 0];
B1 = [P.B; 0];

%tuning parameters
% tr = 2; %original
tr = 2.5; %tuned
zeta = .707;
integrator_pole = -10;

%compute gains
wn = 2.2/tr;

des_char_poly = conv([1,2*zeta*wn,wn^2],poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=3
    disp('System Not Controllable');
else % if so, compute gains
    K1 = place(A1,B1,des_poles);
    P.K = K1(1:2);
    P.ki = K1(3);
end

% observer design
wn_obs   = 10*wn;
des_obsv_char_poly = [1,2*zeta*wn_obs,wn_obs^2];
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A,P.C))~=2
    disp('System Not Observable'); 
else % if so, compute gains
    P.L = place(P.A', P.C', des_obsv_poles)';
end