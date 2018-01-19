% Test function for drawing the Ball Beam system
clear 
clc
close all

% drawing parameters

gap = 0.01;
R=.25;
width= 1;
height= .5;
h_vtol=1;
w_vtol=1.5;

% create position and angle vectors versus time to drive the animation
t = 0:0.1:20;
z_t = 2*sin(2*pi*0.1*t);
z_v = 2*sin(2*pi*0.1*t+pi);
theta = 60*pi/180*sin(2*pi*0.1*t);
h= 100*pi/180*sin(2*pi*.1*t)+6;

N = length(t);

for i = 1:N,
   drawVTOLsys(z_v(i), z_t(i), h(i), theta(i), t(i), width, height, gap, h_vtol, w_vtol);
   pause(0.05);
end

  