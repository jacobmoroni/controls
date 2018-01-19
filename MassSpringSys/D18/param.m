%Loop Shaping for Mass Spring Sys
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
Delta_ol = [1,P.b/P.m,P.k/P.m];

ki = .7;
p_LPF = 100;
p_Lead = 30;
z_Lead = .5;
kp = 15;
z_Lag = 1;
p_Lag = .0001;

C_PI = tf([1,ki],[1, 0]);
C_LPF = tf(p_LPF,[1, p_LPF]);
C_Lead = tf([p_Lead*1,p_Lead*z_Lead],[z_Lead*1,z_Lead*p_Lead]);
C_Lag = tf([1,z_Lag],[1,p_Lag]);

% transfer functions for inverted pendulum
P_loop = tf(1/P.m,Delta_ol);
Plant = P_loop;
C_total = kp*C_PI*C_Lead*C_LPF;%*C_Lag;
C = C_total;

%ramp = tf(1,[1, 0, 0]);
figure(2), clf, 
bode(P_loop), grid on
hold on
bode(series(C_total,P_loop))
margin(P_loop*C_total),
%bode(ramp)

[m_500,p] = bode (C_total*P_loop,500); %this pulls magnitude and phase at .6
[m_01,p] = bode (C_total*P_loop,.1);

% legend('No control', 'PID')
%title('Mass Spring Damper')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
  F = tf([1],[1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% low pass filter
    p = 2;  % frequency to start the LPF
    LPF = tf(p,[1 p]);
    F = F*LPF;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create plots for analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Open-loop tranfer function 
  OPEN = Plant*C;
% closed loop transfer function from R to Y
  CLOSED_R_to_Y = minreal((Plant*C/(1+Plant*C)));
% closed loop transfer function from R to U
  CLOSED_R_to_U = minreal((C/(1+C*Plant)));

figure(3), clf
    subplot(3,1,1), 
        bodemag(CLOSED_R_to_Y)
        title('Closed-loop Bode plot'), grid on
    subplot(3,1,2), 
        step(CLOSED_R_to_Y)
        title('Closed-loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U)
        title('Control effort for step response'), grid on

% % print('../../../figures/hw_pendulum_compensator_out_design_4','-depsc')        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C=minreal(C);
[num,den] = tfdata(C,'v');
[P.Aout_C,P.Bout_C,P.Cout_C,P.Dout_C]=tf2ss(num,den);

[num,den] = tfdata(F,'v');
[P.Aout_F, P.Bout_F, P.Cout_F, P.Dout_F] = tf2ss(num,den);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to discrete transfer functions for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C_out_d = c2d(C,P.Ts,'tustin');
[P.Cout_d_num,P.Cout_d_den] = tfdata(C_out_d,'v');

C_out = C;
