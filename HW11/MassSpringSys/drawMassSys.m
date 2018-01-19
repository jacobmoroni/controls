function drawMassSys(u)

    %process inputs
    z = u(1);
    t = u(2); 
    
    %drawing parameters
    gap = 0.01;
    width = 1.0;
    height = 1.0;
    
    
    % define persistent variables 
    persistent mass_handle
    persistent spring_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        track_width=4;
        plot([-track_width,track_width],[0,0],'k','LineWidth', 2); % plot track
        hold on
        line([-3 -3], [0 3])
        hold on
        mass_handle = drawMass(z, width, height, gap, []);
        spring_handle  = drawSpring(z, height, gap, []);
        axis([-track_width, track_width, 0, 2*track_width]);
    
        
    % at every other time step, redraw base and rod
    else 
        drawMass(z, width, height, gap, mass_handle);
        drawSpring(z, gap, height, spring_handle);
    end
end

   
%
%=======================================================================
% drawBase
% draw the base of the pendulum
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawMass(z, width, height, gap, handle)
  
  X = [z-width/2, z+width/2, z+width/2, z-width/2];
  Y = [gap, gap, gap+height, gap+height];

  if isempty(handle)
    handle = fill(X,Y,'m');
  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end
 
%
%=======================================================================
% drawRod
% draw the pendulum rod
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawSpring(z, gap, height, handle)

  
  X = [-3,z-.5];
  Y = [.5, .5];

  if isempty(handle),
    handle = plot(X, Y, 'g', 'LineWidth', 4);
  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end

  