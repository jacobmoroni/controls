function out=crane_ctrl(in,P)
    x_r    = in(1);
    x_m    = in(2);
    th_m   = in(3);
    t      = in(4);
    
    % implement observer
    persistent xhat       % estimated state (for observer)
    persistent F
    if t<P.Ts,
        xhat = [0;0;0;0];
        F    = 0;
    end
    N = 10;
    for i=1:N,
        xhat = xhat + ...
            P.Ts/N*(P.A*xhat+P.B*F...
                    +P.Lo*([x_m;th_m]-P.C*xhat));
    end
    zhat = xhat(3)

    % integrator
    error = x_r - zhat;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if t<P.Ts==1,
        integrator  = 0;
        error_d1    = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;

    % compute the state feedback controller
    F_unsat = -P.K*xhat - P.ki*integrator;
    F = sat( F_unsat, P.F_max);
    
    % integrator anti-windup
    if P.ki~=0,
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