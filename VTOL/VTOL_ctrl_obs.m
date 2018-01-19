function output=VTOL_ctrl_obs(in,P)
    z_r   = in(1);
    z_m     = in(2);
    theta_m = in(3);
    h_r   = in(4);
    h_m     = in(5);
    t     = in(6);
    
    %Equilibrium Force
    Fe = (P.mc+2*P.mr)*P.g;
    
    % implement lateral observer
    persistent xhat_lat       % estimated state (for observer)
    persistent Tau
    if t<P.Ts
        xhat_lat = [0;0;0;0];
        Tau  = 0;
    end
    N = 10;
    for i=1:N
        xhat_lat = xhat_lat + ...
            P.Ts/N*(P.A_lat*xhat_lat+P.B_lat*Tau...
                    +P.L_lat*([z_m;theta_m]-P.C_lat*xhat_lat));
    end
    zhat = xhat_lat(1);

    % integrator
    error = z_r - zhat;
    persistent integrator_lat
    persistent error_d1_lat
    % reset persistent variables at start of simulation
    if t<P.Ts==1
        integrator_lat  = 0;
        error_d1_lat    = 0;
    end
    integrator_lat = integrator_lat + (P.Ts/2)*(error+error_d1_lat);
    error_d1_lat = error;

    % compute the state feedback controller
    Tau= -P.K_lat*xhat_lat - P.ki_lat*integrator_lat;
    
    % integrator anti-windup
    if P.ki_lat~=0
       integrator_lat = integrator_lat + P.Ts/P.ki_lat*(Tau);
    end
    
    % implement longitudinal observer
    persistent xhat_lon       % estimated state (for observer)
    persistent F
    if t<P.Ts
        xhat_lon = [0;0];
        F    = P.F_eq;
    end
    N = 10;
    for i=1:N
        xhat_lon = xhat_lon + ...
            P.Ts/N*(P.A_lon*xhat_lon+P.B_lon*(F-Fe)...
                    +P.L_lon*(h_m-P.C_lon*xhat_lon));
    end
    hhat = xhat_lon(1);

    % integrator
    error = h_r - hhat;
    persistent integrator_lon
    persistent error_d1_lon
    % reset persistent variables at start of simulation
    if t<P.Ts==1
        integrator_lon  = 0;
        error_d1_lon    = 0;
    end
    integrator_lon = integrator_lon + (P.Ts/2)*(error+error_d1_lon);
    error_d1_lon = error;

    % compute the state feedback controller
    F_unsat = -P.K_lon*xhat_lon - P.ki_lon*integrator_lon;
    F = sat( F_unsat, P.F_max);
    
    % integrator anti-windup
    if P.ki_lon~=0
       integrator_lon = integrator_lon + P.Ts/P.ki_lon*(F-F_unsat);
    end
    xhat = [xhat_lat;xhat_lon];
    output = [F; Tau; xhat];

end

%------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end