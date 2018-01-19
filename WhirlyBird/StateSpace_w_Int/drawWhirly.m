function drawWhirly(u)

    % inputs to function
    % theta     : pitch about east angle
    % phi       : roll about north angle
    % psi       : yaw trick yaw
    phi=u(1);
    psi=u(2);
    theta=u(3);
    t=u(4);
    
    % define persistent variables 
    persistent bird_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        plotsize=1.5;
        h=.65; %meters (height of base)
        %plot base
        plot3([0,0],[0,0], [0,-h],'k','LineWidth', 2); % plot track
        %plot3([0,0],[0,0], [0,-h]);
        hold on
        bird_handle  = drawBird(theta, phi, psi, []);
        axis([-plotsize, plotsize, -plotsize, plotsize, -h, plotsize]);
        xlabel('North');
        ylabel('East');
        zlabel('Down');
        grid on
    
        
    % at every other time step, redraw ball and beam
    else 
        drawBird(theta, phi, psi, bird_handle);
    end
end

   
%
%=======================================================================
% drawBird
% draw the Whirly bird
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBird(theta, phi, psi, handle)
    l1=.85;
    l2=.3048;
    r=.12;
    d=.178;
  

    X_pts=[-l2,l1,l1,l1+r,l1+r,l1-r,l1-r,l1,l1,l1+r,l1+r,l1-r,l1-r,l1,l1];

    Y_pts=[0,0,d-r,d-r,d+r,d+r,d-r,d-r,-d+r,-d+r,-d-r,-d-r,-d+r,-d+r,0];

    Z_pts=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]; 
    
    pts=[X_pts;Y_pts;Z_pts];
    psi=-psi;

    R=[cos(theta)*cos(psi), -cos(theta)*sin(psi), -sin(theta);...
        -sin(phi)*sin(theta)*cos(psi)+ cos(phi)*sin(psi),...
        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), -sin(phi)*cos(theta);...
        cos(phi)*sin(theta)*cos(psi)+ sin(phi)*sin(psi),...
        -cos(phi)*sin(theta)*sin(psi)+sin(phi)*cos(psi), cos(phi)*cos(theta)];
    pts_rot=R*pts;
  
  
    X = pts_rot(1,:);
    Y = pts_rot(2,:);
    Z = pts_rot(3,:);

    if isempty(handle),
        handle = fill3(X, Y, Z, 'g');
    else
        handle.XData = X;
        handle.YData = Y;
        handle.ZData = Z;
        drawnow
    end
end