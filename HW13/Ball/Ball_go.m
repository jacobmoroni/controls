% Test function for drawing the Ball Beam system
clear 
clc
close all

% drawing parameters
L = 3;
gap = 0.01;
R=.25;

% create position and angle vectors versus time to drive the animation
t = 0:0.1:20;
z = (t/20)*L;%2*sin(2*pi*0.1*t);
theta = t*3;%20*pi/180*sin(2*pi*0.2*t);

N = length(t);

for i = 1:N,
   drawBallSys(z(i),theta(i),t(i),L,R,gap);
   pause(0.05);
end

  