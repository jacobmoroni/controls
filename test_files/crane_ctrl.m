function F=crane_ctrl(in,P)
    x_r   = in(1);
    x     = in(2);
    th    = in(3);
    t     = in(4);
    
    % use a digital differentiator to find zdot and thetadot
    persistent xdot
    persistent x_d1
    persistent thdot
    persistent th_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        xdot     = 0;
        x_d1     = 0;
        thdot    = 0;
        th_d1    = 0;
    end
    xdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*xdot...
        + 2/(2*P.sigma+P.Ts)*(x-x_d1);
    thdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*thdot...
        + 2/(2*P.sigma+P.Ts)*(th-th_d1);
    x_d1 = x;
    th_d1 = th;

    % construct the state
    x = [xdot; thdot; x; th];
    % compute the state feedback controller
    F = sat( -P.K*x + P.kr*x_r, P.F_max);
end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end