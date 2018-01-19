param
loopshape_ball_in

Plant = minreal(P_out*(P_in*C_in/(1+P_in*C_in)));
figure(2), clf
    hold on
    grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %--- general tracking specification ---
        omega_r = 0.01;  % track signals below this frequency
        gamma_r = 1/100.0;  % tracking error below this value
        w = logspace(log10(omega_r)-2,log10(omega_r));
        plot(w,(1/gamma_r)*ones(size(w)),'g')
        
        %--- noise specification ---
        omega_n = 100;  % attenuate noise above this frequency
        gamma_n = 0.001;   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,gamma_n*ones(size(w)),'g')
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = tf([1],[1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot plant
    bode(Plant,logspace(-3,5));
%     margin(Plant)
    grid on
    hold on
%     legend('Plant alone')
    
%Proportional Gain to invert plant
    K = -1;
    C=C*K;
%     bode(Plant*C,logspace(-3,5));
%     figure(2), bode(Plant*C);
    
%Proportional Gain to invert plant
    K = .1%K = .01;
    C=C*K;
%     bode(Plant*C,logspace(-3,5));
%     figure(2), bode(Plant*C);
    
% phase lead: increase PM (stability)
    w_max = 2.5; % location of maximum frequency bump (desired crossover)
    phi_max = 60*pi/180;
    M = (1+sin(phi_max))/(1-sin(phi_max)); % lead ratio
    z = w_max/sqrt(M);
    p = w_max*sqrt(M);
    Lead = tf([1/z 1],[1/p 1]);
%     Lead = tf([25 10],[1 2/5]);
    C = C*Lead;
%     figure(2), margin(Plant*C) % update plot
    

    
%integral control : to tune low frequencies to correct gain
    ki = .3;%ki = .01;
    PI = tf([1,ki],[1, 0]);
    C = PI*C;
    figure(2), margin(Plant*C)
    
%already meets specifications, so no additional control is necessary
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
  F = tf([1],[1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% low pass filter
    p = 2;  % frequency to start the LPF
    LPF = tf(p,[1 p]);
    F = F*LPF

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
        bodemag(P_in*C_in/(1+P_in*C_in))
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
[P.Aout_C,P.Bout_C,P.Cout_C,P.Dout_C]=tf2ss(num,den);

[num,den] = tfdata(F,'v');
[P.Aout_F, P.Bout_F, P.Cout_F, P.Dout_F] = tf2ss(num,den);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to discrete transfer functions for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C_out_d = c2d(C,P.Ts,'tustin');
[P.Cout_d_num,P.Cout_d_den] = tfdata(C_out_d,'v');

F_d = c2d(F,P.Ts,'tustin');
[P.F_d_num,P.F_d_den] = tfdata(F_d,'v');

C_out = C;


