function output=whirly_ctrl_state(in,P)
    psi_r = in(1);
    psi   = in(2);
    phi   = in(3);
    theta_r   = in(4);
    theta     = in(5);
    t     = in(6);
    
    
    
    %longitudinal
     % Longitudinal
    % use a digital differentiator to find zdot and thetadot
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        thetadot        = 0;
        theta_d1        = theta;
    end
    thetadot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*thetadot...
        + 2/(2*P.sigma+P.Ts)*(theta-theta_d1);
    theta_d1 = theta;

    % construct the state
    x_lon = [theta; thetadot];
    
    % SS Feedck controller Longitudnal Dynamics
    force_tilde = -P.K_lon*x_lon + P.kr_lon*theta_r;
    force_eq = ((P.mc+2*P.mr)*P.g)/cos(phi);
    Force = sat(force_tilde + force_eq, P.forcemax);
    
    %%%% Lateral
    
    % use a digital differentiator to find zdot and thetadot
    persistent psidot
    persistent psi_d1
    persistent phidot
    persistent phi_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        psidot        = 0;
        psi_d1        = psi;
        phidot    = 0;
        phi_d1    = phi;
    end
    psidot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*psidot...
        + 2/(2*P.sigma+P.Ts)*(psi-psi_d1);
    phidot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*phidot...
        + 2/(2*P.sigma+P.Ts)*(phi-phi_d1);
    psi_d1 = psi;
    phi_d1 = phi;

    % construct the state
    x_lat = [psi; phi; psidot; phidot];
    
    % SS Feedck controller Lateral Dynamics
    Tau = -P.K_lat*x_lat + P.kr_lat*psi_r;
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