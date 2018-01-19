function F= ball_ctrl_state(in,P)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
    %use a digital differetiator to find zdot
    persistent zdot
    persistent z_d1
    persistent thetadot
    persistent theta_d1

    %reset persistent variables at start of simulation
    if t<P.Ts
        zdot = 0;
        z_d1 = z;
        thetadot = 0;
        theta_d1 = theta;
    end
    zdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*zdot...
        + 2/(2*P.sigma+P.Ts)*(z-z_d1);
    thetadot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*thetadot...
        + 2/(2*P.sigma+P.Ts)*(theta-theta_d1);
    z_d1 = z;
    
    theta_d1 = theta;
    
    % construct the state
    x = [z; theta; zdot; thetadot];    
    xe = [P.ze; 0; 0; 0];
    x_tilde = x-xe;
    
    %%% re = ye
    %%% ye = C * xe
    z_re = P.ze;
    zr_tilde = z_r - z_re;
    % compute force
    F_tilde = -P.K*x_tilde + P.kr*zr_tilde;
    F_eq = (P.m1*P.g*P.ze)/P.l + (P.m2*P.g)/2;
    F = F_tilde + F_eq;
    
%     F_unsat = F; 
%     force = sat(F_unsat,P.force_max);
    
end
%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end
