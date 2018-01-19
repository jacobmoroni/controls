function force=mass_ctrl_state(in,P)
    z_c     = in(1);
    z       = in(2);
    t       = in(3);
    
    %use a digital differetiator to find zdot
    persistent zdot
    persistent z_d1

    %reset persistent variables at start of simulation
    if t<P.Ts
        zdot = 0;
        z_d1 = 0;
    end
    zdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*zdot...
        + 2/(2*P.sigma+P.Ts)*(z-z_d1);
    z_d1 = z;
    
    % construct the state
    x = [z; zdot];
    % compute force
    F_unsat = - P.K*x + P.kr*z_c; 
    force = sat(F_unsat,P.force_max);
   
end


function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end