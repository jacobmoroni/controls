param
% loopshape_in

Plant = P_lon;
figure(2), clf
    hold on
    grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %--- general tracking specification ---
        omega_r = 0.1;  % track signals below this frequency
        gamma_r = 0.01;  % tracking error below this value
        w = logspace(log10(omega_r)-2,log10(omega_r));
        plot(w,(1/gamma_r)*ones(size(w)),'g')
        
        %--- noise specification ---
        omega_n = 200;  % attenuate noise above this frequency
        gamma_n = 0.001;   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,gamma_n*ones(size(w)),'g')
        
figure(2), bode(Plant,logspace(-3,5)), grid on
%print('../../../figures/hw_pendulum_compensator_out_design_1','-depsc')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = tf([1],[1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % % proportional control: correct for negative sign in plant
% % find gain to set crossover at w_max = 25 rad/s
    [m,p] = bode(Plant*C,1);
    K = 1/m;
    C = K*C;
    figure(2), bode(Plant*C)  % update plot
% %      print('../../../figures/hw_pendulum_compensator_in_design_2','-depsc')
% 
% % % Integral control: reject input disturbances
    ki = .1;
    Integrator = tf([1 ki],[1 0]);
    C = C*Integrator;
    figure(2), bode(Plant*C)  % update plot 

% % % Noise attenuation constraint not quite satisfied
% % % Can be satisfied by reducing gain at 400 rad/s by a factor of 2
% % Use a low-pass filter
    m = 0.7;    % attenuation factor
    a = m*omega_n*sqrt(1/(1-m^2));
    lpf = tf(a,[1 a]);
    C = lpf*C;
    figure(2), bode(Plant*C),  % update plot
    
% % phase lead: increase PM (stability)
% % At desired crossover frequency, PM = -3
% % Add 70 deg of PM with lead
    w_max = 2.0; % location of maximum frequency bump (desired crossover)
    phi_max = 65*pi/180;
    M = (1+sin(phi_max))/(1-sin(phi_max)); % lead ratio
    z = w_max/sqrt(M);
    p = w_max*sqrt(M);
    Lead = tf([1/z 1],[1/p 1]);
    C = C*Lead;
   figure(2), margin(Plant*C)  % update plot    
%    
    legend('Plant','Proportional', 'Integral', 'lpf','phase lead' )%'K=-1','lead, K=-1','lead, K=-45.1')
% %print('../../../figures/hw_pendulum_compensator_in_design_3','-depsc')


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
  CLOSED_R_to_Y = (Plant*C/(1+Plant*C));
% closed loop transfer function from R to U
  CLOSED_R_to_U = (C/(1+C*Plant));

figure(3), clf
    subplot(3,1,1), 
        bodemag(CLOSED_R_to_Y), hold on
        bodemag(CLOSED_R_to_Y*F)
        bodemag(P_lon*C/(1+P_lon*C))
        title('Closed-loop Bode plot'), grid on
        legend('closed loop','closed loop + prefilter','open loop');
    subplot(3,1,2), 
        step(CLOSED_R_to_Y), hold on
        step(CLOSED_R_to_Y*F)
        title('Closed-loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U), hold on
        step(CLOSED_R_to_U*F)
        title('Control effort for step response'), grid on
% print('../../../figures/hw_pendulum_compensator_out_design_5','-depsc')


        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C=minreal(C);
[num,den] = tfdata(C,'v');
[P.Alon_C,P.Blon_C,P.Clon_C,P.Dlon_C]=tf2ss(num,den);

[num,den] = tfdata(F,'v');
[P.Alon_F, P.Blon_F, P.Clon_F, P.Dlon_F] = tf2ss(num,den);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to discrete transfer functions for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C_lon_d = c2d(C,P.Ts,'tustin');
[P.Clon_d_num,P.Clon_d_den] = tfdata(C_lon_d,'v');

F_d = c2d(F,P.Ts,'tustin');
[P.F_d_num,P.F_d_den] = tfdata(F_d,'v');

C_lon = C;


