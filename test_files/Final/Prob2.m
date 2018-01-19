% system parameters
g = 9.81; %m/s^2
theta = 45; %deg
m = .5; %kg
b = .1; %N-s/m
k1 = .05; %N/m
k2 = .02; %N/m^3
tau = .5; %s

% Transfer functions
Pin = tf([1],[tau, 1]);
Pout = tf([1],[m, b, k1]);

Plant = Pin*Pout;

w_co = 2.2/1.1;
zeta = .5;

figure(1), clf
    hold on
    
plant_des = plant_in*plant_out;
  figure(1), bode(Plant,logspace(-3,5)), grid on, hold on
%Loopshaping  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % phase lead: increase PM (stability)
    K = 0.16573;
    p = 100;
    z = .12;
    %I was orgigionally doing this calculation without the independent K
    %gain value, but the maximum Phase margin i could get at the crossover
    %frequency of 2 rad/s was about 30 degrees, so because of the way the
    %problem is phrased with a 'gain' on the lead compensator, I added this
    %in to get a better phase margin 
    Lead = tf(K*[1/z 1],[1/p 1]);
    C = C*Lead;
    figure(1), margin(Plant*C) % update plot
    
%Lag Filter
    M = 302;
    z_l = .05;
    p_l = z_l/M;
    Lag = tf(M*[1/z_l,1],[1/p_l,1]);
    C = C*Lag;
    %I set the zero pole value low to prevent ruining the phase margin
    [m_plant,p] = bode(Plant,0);
    [m_control,p] = bode(Plant*C,0);
    error = m_plant/m_control
    figure(1), bode(Plant*C) % update plot
    legend ('Plant','Lead Compensation','Lag+Lead Compensation');