function drawBallSys(u)

    % inputs to function
    % z        : horizontal travel of ball
    % theta    : angle of rod
    z = u(1);
    theta = u(2);
    t = u(3);
    
    %drawing parameters
    L = .5;
    gap = 0.01;
    R=.05;
    
    
    
    % define persistent variables 
    persistent beam_handle
    persistent ball_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        frame_width=1;
        plot([0],[0],'or','LineWidth', 2); % plot hinge
        hold on
        beam_handle = drawBeam(theta, L, []);
        ball_handle  = drawBall(z, theta, R, gap, []);
        axis([-frame_width, frame_width, -.25, frame_width]);
    
        
    % at every other time step, redraw ball and beam
    else 
        drawBeam(theta,L, beam_handle);
        drawBall(z, theta, R, gap, ball_handle);
    end
end

   
%
%=======================================================================
% drawBeam
% draw the beam
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBeam(theta, L, handle)
  
  X = [0, L*cos(theta)];
  Y = [0, L*sin(theta)];

  if isempty(handle)
    handle = plot(X,Y,'m','LineWidth',2);
  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end
 
%
%=======================================================================
% drawBall
% draw the ball
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBall(z, theta, R, gap,handle)
  
  ang = 0:.01:2*pi;
  x=R*cos(ang);
  y=R*sin(ang);
  
  
  X = z*cos(theta)-(R+gap)*sin(theta)+x;
  Y = z*sin(theta)+(R+gap)*cos(theta)+y;

  if isempty(handle),
    handle = fill(X, Y, 'g');
  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end