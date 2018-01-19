%clear all

% initial conditions
P.z0 = 0;
P.zdot0 = 0;
P.theta0 = 0;
P.thetadot0 = 0;

% system parameters known to controller
P.m1 = 0.25;  % kg
P.m2 = 1;     % kg
P.ell = 0.5; % m
P.b = 0.05; % N m
P.g = 9.8; % m/s^2

% sample rate for controller
P.Ts = 0.01;

% gain for dirty derivative
P.sigma = 0.05;

% input constraint
P.F_max = 5;

% tunning parameters
tr_z = 2;%5;  % rise time for outer loop   - Can't really push saturation limits and remain stable.  
zeta_z   = 0.707; % damping ratio for outer loop
M = 8;%10;    % time scale separation between inner and outer loop
zeta_th  = 0.707; % damping ratio for inner loop

% PD gains for inner loop
tr_theta = tr_z/M;  % rise time for inner loop
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
P.kp_th  = -(P.m1+P.m2)*P.g - P.m2*P.ell/2*wn_th^2; % kp - inner loop
P.kd_th  = -zeta_th*wn_th*P.m2*P.ell; % kd - inner loop

% DC gain for inner loop
k_DC_th = P.kp_th/((P.m1+P.m2)*P.g+P.kp_th);

%PD design for outer loop
wn_z     = 2.2/tr_z; % natural frequency for outer loop
P.kp_z = wn_z^2/P.g/k_DC_th;
P.kd_z = 2*zeta_z*wn_z/P.g/k_DC_th;

% select integrator gain
P.ki_z = 0.0001;

% transfer function for inverted pendulum
P_in  = tf([-2/P.m2/P.ell],[1, 0, -2*(P.m1+P.m2)*P.g/P.m2/P.ell]);
P_out = tf(-[P.ell/2 0 -P.g],[1, 0, 0]);

