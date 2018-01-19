param

Plant = P_in;

figure(2), clf
    hold on
    grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %--- general tracking specification ---
        omega_r = 1;  % track signals below this frequency
        gamma_r = 1/312.5;  % tracking error below this value
        w = logspace(log10(omega_r)-2,log10(omega_r));
        plot(w,(1/gamma_r)*ones(size(w)),'g')


        %--- noise specification ---
        omega_n = 1000;   % attenuate noise above this frequency
        gamma_n = 0.0032;   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,gamma_n*ones(size(w)),'g')
        

  figure(2), bode(Plant,logspace(-3,5)), grid on
% %print('../../../figures/hw_pendulum_compensator_in_design_1','-depsc')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Proportional Control: find gain to set crossover at w_max = 30 rad/s
    [m,p] = bode(Plant*C,30);
    %K = 1/m;
    K = 150;
    C = K*C;
    figure(2), margin(Plant*C)  % update plot

% phase lead: increase PM (stability)
    w_max = 40;%30; % location of maximum frequency bump (desired crossover)
    phi_max = 64*pi/180;
    M = (1+sin(phi_max))/(1-sin(phi_max)); % lead ratio
    z = w_max/sqrt(M);
    p = w_max*sqrt(M);
    Lead = tf([1/z 1],[1/p 1]);
    %Lead = tf(15*[1,40/sqrt(15)],[1,40/sqrt(15)]);
    C = C*Lead;
    figure(2), bode(Plant*C), hold on, grid % update plot
    

    
    
%integral control : to tune low frequencies to correct gain
%     ki = 1.3;
%     PI = tf([1,ki],[1, 0]);
%     C = PI*C;
%     figure(2), margin(Plant*C)
    
%low pass filter: better noise attenuation
    m = .8;    % attenuation factor
    %a = m*omega_n*sqrt(1/(1-m^2));
    a = 500;
    lpf = tf(a,[1 a]);
    C = lpf*C;
    figure(2), margin(Plant*C),  % update plot
    legend('lead','P+lead','PI+lead','PI+lead+LPC')
    
%confirm that tracking and noise are met
    [m_track,p] = bode(Plant*C,1)%must be at least 332
    [m_noise,p] = bode(Plant*C,1000)%max = .0032

% % Phase Lag Filter: Lag Find gain increase needed at omega_r
%     [m,p] = bode(Plant*C,omega_r);
%     gain_increase_needed = 1/gamma_r/m
%     % Minimum gain increase at low frequencies is 4.8. Let lag ratio be 8.
%     M = 8;
%     p = omega_r;    % Set pole at omega_r
%     z = M*p;        % Set zero at M*omega_r
%     Lag = tf(M*[1/z 1],[1/p 1]);
%     C = C*Lag;
%     figure(2), margin(Plant*C)
%     legend('lead','lead+lag');
%     figure(2), bode(Plant*C)
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

% % print('../../../figures/hw_pendulum_compensator_in_design_4','-depsc')        
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


