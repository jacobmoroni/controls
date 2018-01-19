function out=ball_ctrl_obs(in,P)
    z_r     = in(1);
    z_m     = in(2);
    theta_m = in(3);
    t       = in(4);
    
    %compute equilibrium values
    z_e = P.l/2;
    F_e = 0.5*P.m2*P.g + P.m1*P.g*z_e/P.l;
    x_e = [0; z_e; 0; 0];
    
    % implement observer
    persistent xhat       % estimated state (for observer)
    persistent F
    if t<P.Ts
        xhat = [0;0;0;0];
        F    = 0;
    end
    N = 10;
    for i=1:N
        xhat = xhat + ...
            P.Ts/N*(P.A*(xhat-x_e)+P.B*(F-F_e)...
                    +P.L*([z_m;theta_m]-P.C*xhat));
    end
    zhat = xhat(1);

    % integrator
    error = z_r - zhat;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if t<P.Ts==1
        integrator  = 0;
        error_d1    = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;

    % compute the state feedback controller
    F_unsat = F_e -P.K*(xhat-x_e) - P.ki*integrator;
    F = sat( F_unsat, P.F_max);
    
    % integrator anti-windup
    if P.ki~=0
       integrator = integrator + P.Ts/P.ki*(F-F_unsat);
    end
    
    out = [F; xhat];

end

%------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end