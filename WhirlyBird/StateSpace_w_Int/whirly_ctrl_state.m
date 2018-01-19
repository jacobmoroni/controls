function output=whirly_ctrl_state(in,P)
    theta_r     = in(1);
    theta       = in(2);
    phi       = in(3);
    psi_r   = in(4);
    psi     = in(5);
    t         = in(6);
    
    
    
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

    % integrator

    error = theta_r - theta;
    persistent integrator_lon
    persistent error_d1_lon
    % reset persistent variables at start of simulation
    if t<P.Ts==1
        integrator_lon = 0;
        error_d1_lon = 0;
    end
    integrator_lon = integrator_lon + (P.Ts/2)*(error+error_d1_lon);
    error_d1_lon = error;

    % construct the state
    x_lon = [theta; thetadot];
    
    % compute the state feedback controller
    F_unsat = -P.K_lon*x_lon - P.ki_lon*integrator_lon;
    F = sat( F_unsat, P.F_max);

    % integrator anti-windup
    if P.ki_lon ~= 0
        integrator_lon = integrator_lon + P.Ts/P.ki_lon*(F-F_unsat);   
    end

    
    %%%% Lateral
    
    % use a digital differentiator to find zdot and thetadot
    persistent psidot
    persistent psi_d1
    persistent phidot
    persistent phi_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        psidot    = 0;
        psi_d1    = psi;
        phidot    = 0;
        phi_d1    = phi;
    end
    psidot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*psidot...
        + 2/(2*P.sigma+P.Ts)*(psi-psi_d1);
    phidot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*phidot...
        + 2/(2*P.sigma+P.Ts)*(phi-phi_d1);
    psi_d1 = psi;
    phi_d1 = phi;
    
    % integrator lateral
    error = psi_r - psi;
    persistent integrator_lat
    persistent error_d1_lat
    % reset persistent variables at start of simulation
    if t<P.Ts==1
        integrator_lat  = 0;
        error_d1_lat    = 0;
    end
    integrator_lat = integrator_lat + (P.Ts/2)*(error+error_d1_lat);
    error_d1_lat = error;

    % construct the state
    x_lat = [psi; phi; psidot; phidot];
    
    % compute the state feedback controller
    Tau_unsat = -P.K_lat*x_lat - P.ki_lat*integrator_lat;
    Tau = sat( Tau_unsat, P.Tau_max);
    
    % integrator anti-windup
    if P.ki_lat ~= 0
       integrator_lat = integrator_lat + P.Ts/P.ki_lat*(Tau-Tau_unsat);
    end

                
    output = [F,Tau];
    
end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end