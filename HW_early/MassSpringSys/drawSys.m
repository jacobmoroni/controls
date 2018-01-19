function drawSys(u)

    % inputs to function
    % z        : horizontal travel of cart
    
    z        = u(1);
    t        = u(2);
    
    % drawing parameters
    gap = 0.01;

    % Mass Parameters
    width = 2.0;
    height = 2.0;
    
    % define persistent variables 
    persistent mass_handle
    persistent spring_handle
    persistent damper_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t == 0,
        figure(1), clf
        track_width=4;
        plot([-track_width,track_width],[0,0],'k', 'LineWidth', 2); % plot track
        hold on
        plot([-3.5, -3.5],[0 3],'k', 'LineWidth', 2); % plot wall
        hold on
        mass_handle = drawMass(z, width, height, gap, []);
        spring_handle  = drawSpring(z, height,width, []);
        damper_handle  = drawDamper(z, height,width, []);
        axis([-track_width, track_width, -1, 2*track_width-1]);    
        
    % at every other time step, redraw mass
    else 
        drawMass(z, width, height, gap, mass_handle);
        drawSpring(z, height,width, spring_handle);
        drawDamper(z, height,width, damper_handle);
    end
end

   
%
%=======================================================================
% drawMass
% draw the Mass of the pendulum
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawMass(z, width, height, gap, handle)
  
  X = [z-width/2, z+width/2, z+width/2, z-width/2];
  Y = [gap, gap, gap+height, gap+height];

  if isempty(handle),
    handle = fill(X,Y,'m');
  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end
 
% 
% =======================================================================
% drawSpring
% draw the Spring
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
% =======================================================================
% 
function handle = drawSpring(z, height,width, handle)
  
  X = [-3.5, z-width/2];
  Y = [.75*height, .75*height];
 

  if isempty(handle),
    handle = plot(X, Y, 'k', 'LineWidth', 2);
    hold on

  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end

% 
% =======================================================================
% drawDamper
% draw the Damper
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
% =======================================================================
% 
function handle = drawDamper(z, height,width, handle)
  
   X = [-3.5, z-width/2];
   Y = [.25*height, .25*height];
 

  if isempty(handle),
    handle = plot(X, Y, 'k', 'LineWidth', 2);
    hold on

  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end

  