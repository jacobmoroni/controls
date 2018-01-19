% Test function for drawing the Ball Beam system
clear 
clc
close all

% create position and angle vectors versus time to drive the animation
t = 0:0.1:20;
theta = 60*pi/180*sin(2*pi*0.1*t);
phi = 60*pi/180*sin(2*pi*0.1*t);
psi = 60*pi/180*sin(2*pi*0.1*t);

N = length(t);

for i = 1:N,
   u=[theta(i), phi(i), psi(i), t(i)];
   drawWhirly(u);
   pause(0.05);
end

  