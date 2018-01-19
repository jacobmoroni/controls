param
% loopshape_pendulum_in

Plant = P_lat_in;
figure(1), clf
    hold on
    grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%         %--- general tracking specification ---
%         omega_r = 1;  % track signals below this frequency
%         gamma_r = 0.0032;  % tracking error below this value
%         w = logspace(log10(omega_r)-2,log10(omega_r));
%         plot(w,(1/gamma_r)*ones(size(w)),'g')
%         
%         %--- noise specification ---
        omega_n = 1000;  % attenuate noise above this frequency
        gamma_n = 0.001;   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,gamma_n*ones(size(w)),'g')
        
figure(1), bode(Plant,logspace(-3,5)), grid on
%print('../../../figures/hw_pendulum_compensator_out_design_1','-depsc')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = tf([1],[1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % proportional control: correct for negative sign in plant
% find gain to set crossover at w_max = 25 rad/s
    [m,p] = bode(Plant*C,3);
    K = 1/m;
    C = K*C;
    figure(1), bode(Plant*C)  % update plot
% %      print('../../../figures/hw_pendulum_compensator_in_design_2','-depsc')
% 
% % Integral control: reject input disturbances
%     ki = 5;
%     Integrator = tf([1 ki],[1 0]);
%     C = C*Integrator;
%     figure(1), bode(Plant*C)  % update plot 

% Noise attenuation constraint not quite satisfied
% Can be satisfied by reducing gain at 400 rad/s by a factor of 2
% Use a low-pass filter
    m = 0.7;    % attenuation factor
    a = m*omega_n*sqrt(1/(1-m^2));
    lpf = tf(a,[1 a]);
    C = lpf*C;
    figure(1), bode(Plant*C),  % update plot
    
% phase lead: increase PM (stability)
% At desired crossover frequency, PM = -3
% Add 70 deg of PM with lead
    w_max = 3.5; % location of maximum frequency bump (desired crossover)
    phi_max = 70*pi/180;
    M = (1+sin(phi_max))/(1-sin(phi_max)); % lead ratio
    z = w_max/sqrt(M);
    p = w_max*sqrt(M);
    Lead = tf([1/z 1],[1/p 1]);
    C = C*Lead;
   figure(1), margin(Plant*C)  % update plot    
   
    legend('Plant','Proportional', 'lpf', 'phase lead' )%'K=-1','lead, K=-1','lead, K=-45.1')
% %print('../../../figures/hw_pendulum_compensator_in_design_3','-depsc')


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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[num,den] = tfdata(C,'v');
[P.Ain_C,P.Bin_C,P.Cin_C,P.Din_C]=tf2ss(num,den);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to discrete transfer functions for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C_in_d = c2d(C,P.Ts,'tustin');
[P.Cin_d_num,P.Cin_d_den] = tfdata(C_in_d,'v');

C_in = C;



