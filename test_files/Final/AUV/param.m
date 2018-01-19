%AUV parameters and Loopshaping

% system parameters
P.J = 45; %kg*m^2
P.b = 5; %N-m-s
P.m = 200; %kg
P.L = .01; %m
P.g = 9.81; %m/s^2

%sample period
P.Ts = .01; %s

% initial conditions
P.phi0 = 0;
P.phidot0 = 0;

%max Force 
P.Taumax = 50;

% dirty derivative gain
P.sigma = .05;

% Transfer functions
Plant = tf(1,[1, P.b/P.J, P.m*P.g*P.L/P.J]);

%%
%Loopshaping
figure(2), clf
    hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %--- noise specification ---
        omega_n = 100;   % attenuate noise above this frequency
        gamma_n = 0.001;   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,gamma_n*ones(size(w)),'g')
        

  figure(2), bode(Plant,logspace(-3,5)), grid on
 
  
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% integral control: for zero steady state error to step input
 
    ki = .3;
    PI = tf([1,ki],[1, 0]);
    C = PI*C;
%     figure(2), bode(Plant*C)
    
% phase lead: increase PM (stability)
    w_max = 2.0; % location of maximum frequency bump (desired crossover)
    phi_max = 66.9*pi/180;
    M = (1+sin(phi_max))/(1-sin(phi_max)); % lead ratio
    z = w_max/sqrt(M);
    p = w_max*sqrt(M);
    Lead = tf([1/z 1],[1/p 1]);
    C = C*Lead;
%     figure(2), bode(Plant*C)% update plot
    
 % proportional gain: set crossover at w_max = 2 rad/s
    [m,p] = bode(Plant*C,w_max);
    K = 1/m;
    C = K*C;
%     figure(2), bode(Plant*C)  % update plot
    
% low pass filter: Attenuate noise at high frequencies
    m = .5;    % attenuation factor
    a = m*omega_n*sqrt(1/(1-m^2));
    lpf = tf(a,[1 a]);
    C = lpf*C;
    figure(2), margin(Plant*C),  % update plot
    %legend('Plant','Integral','Integral+Lead','PI+Lead','PI+Lead+LPF')
    legend('Plant','Plant*Control')
    %check noise attenuation
    [m_noise,p] = bode(Plant*C,100);%m_noise less than .001
    if m_noise < .001
        disp ('Noise attenuation met')
    else, disp ('Noise attenuation not met')
    end
   
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
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
  F = tf([1],[1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% low pass filter
    p = 3;  % frequency to start the LPF
    LPF = tf(p,[1, p]);
    F = F*LPF;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% C=minreal(C);
[num,den] = tfdata(C,'v');
[P.A_C,P.B_C,P.C_C,P.D_C]=tf2ss(num,den);

[num,den] = tfdata(F,'v');
[P.A_F, P.B_F, P.C_F, P.D_F] = tf2ss(num,den);
