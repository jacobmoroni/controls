function output=VTOL_ctrl_state_int(in,P)
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

    % integrator

    error = h_r - h;
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
    x = [h; hdot];

    % compute the state feedback controller
    F_unsat = -P.K_lon*x - P.ki_lon*integrator_lon;
    F = sat( F_unsat, P.F_max);

    % integrator anti-windup
    if P.ki_lon ~= 0
        integrator_lon = integrator_lon + P.Ts/P.ki_lon*(F-F_unsat);   
    end
    
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
    
     % integrator latera
    error = z_r - z;
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
    x = [z; theta; zdot; thetadot];
    % compute the state feedback controller
    Tau_unsat = -P.K_lat*x - P.ki_lat*integrator_lat;
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