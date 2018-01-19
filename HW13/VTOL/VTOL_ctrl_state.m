function output=VTOL_ctrl_state(in,P)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    h_r   = in(4);
    h     = in(5);
    t     = in(6);
    
    
    
    %longitudinal
     % Longitudinal
    % use a digital differentiator to find zdot and thetadot
    persistent hdot
    persistent h_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        hdot        = 0;
        h_d1        = h;
    end
    hdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*hdot...
        + 2/(2*P.sigma+P.Ts)*(h-h_d1);
    h_d1 = h;

    % construct the state
    x_lon = [h; hdot];
    
    % SS Feedck controller Longitudnal Dynamics
    force_tilde = -P.K_lon*x_lon + P.kr_lon*h_r;
    force_eq = ((P.mc+2*P.mr)*P.g)/cos(theta);
    Force = sat(force_tilde + force_eq, P.forcemax);
    
    %%%% Lateral
    
    % use a digital differentiator to find zdot and thetadot
    persistent zdot
    persistent z_d1
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        zdot        = 0;
        z_d1        = z;
        thetadot    = 0;
        theta_d1    = theta;
    end
    zdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*zdot...
        + 2/(2*P.sigma+P.Ts)*(z-z_d1);
    thetadot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*thetadot...
        + 2/(2*P.sigma+P.Ts)*(theta-theta_d1);
    z_d1 = z;
    theta_d1 = theta;

    % construct the state
    x_lat = [z; theta; zdot; thetadot];
    
    % SS Feedck controller Lateral Dynamics
    Tau = -P.K_lat*x_lat + P.kr_lat*z_r;
    Tau = sat(Tau, P.taumax);
                
    output = [Force,Tau];
    
end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end