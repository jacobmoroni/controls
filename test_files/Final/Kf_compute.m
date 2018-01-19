% system parameters
g = 9.81; %m/s^2
theta = 45; %deg
m = .5; %kg
b = .1; %N-s/m
k1 = .05; %N/m
k2 = .02; %N/m^3
tau = .5; %s

%Code used to find the value of KF for the inner loop
%time domain
Kf = 11.045; %10.5 gave better results in time domain
t = 0:.0001:1;
y = Kf/tau*exp(-((1+Kf)/tau)*t);
ye = exp(-1*t);
xval = Kf/tau-(Kf/tau)*.9;
tr_val = find((y-xval).^2<=.00001);
tr = t(tr_val); % this value gave me tr = .0956

%frequency domain
tau = .5;%s
Pin = tf([1],[tau, 1]);

margin (Pin*Kf), grid on %this gives a crossover frequency of 22 Rad/s

zeta = .707;
wn = 2.2;

DC_gain = Kf/(1+Kf);
Kd = 2*zeta*wn*m/(DC_gain)-b;
Kp = (wn^2*m)/DC_gain-k1;

