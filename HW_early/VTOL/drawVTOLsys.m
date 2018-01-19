function drawVTOLsys(u)

    % inputs to function
    % z_v       : horizontal travel of VTOL
    % z_t       : horizantal travel of target
    % h         : height
    % theta     : angle of VTOL
    z_v     = u(1);
    theta   = u(2);
    h       = u(3);
    z_t     = u(4);
    t       = u(5);
    
    
    % drawing parameters

    gap = 0.01;
    R=.25;
    width= 1;
    height= .5;
    h_vtol=1;
    w_vtol=1.5;
    
    
    % define persistent variables 
    persistent target_handle
    persistent VTOL_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        track_width=15;
        plot([-track_width, track_width],[0,0],'k','LineWidth', 2); % plot track
        hold on
        target_handle = drawTarget(z_t, width, height, gap, []);
        VTOL_handle  = drawVTOL(z_v, z_t, h, theta, h_vtol, w_vtol, []);
        axis([-track_width, track_width, 0, 2*track_width]);
    
        
    % at every other time step, redraw ball and beam
    else 
        drawTarget(z_t, width, height, gap, target_handle);
        drawVTOL(z_v, z_t, h, theta, h_vtol, w_vtol, VTOL_handle);
    end
end

   
%
%=======================================================================
% drawTarget
% draw the target
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawTarget(z_t, width, height, gap, handle)
  
  X = [z_t-width/2, z_t+width/2, z_t+width/2, z_t-width/2];
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
%=======================================================================
% drawVTOL
% draw the VTOL
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVTOL(z_v, z_t, h, theta, h_vtol, w_vtol, handle)
  
  pts=[w_vtol/2, w_vtol/2, (w_vtol/2)+1, (w_vtol/2)+1.5, (w_vtol/2)+2,...
      (w_vtol/2)+1.5, (w_vtol/2)+1, w_vtol/2, w_vtol/2, -w_vtol/2,...
      -w_vtol/2, -w_vtol/2-1, -w_vtol/2-1.5, -w_vtol/2-2, -w_vtol/2-1.5,...
      -w_vtol/2-1, -w_vtol/2, -w_vtol/2, w_vtol/2;...
      h_vtol/2, 0, 0, .25, 0, -.25,0, 0, -h_vtol/2, -h_vtol/2, 0, 0,...
      -.25, 0, .25, 0,  0, h_vtol/2, h_vtol/2];
  R=[cos(theta), -sin(theta); sin(theta), cos(theta)];
  pts_rot=R*pts;
  
  
  X = pts_rot(1,:)+z_v;
  Y = pts_rot(2,:)+h;

  if isempty(handle),
    handle = fill(X, Y, 'g');
  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end