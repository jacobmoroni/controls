function force=mass_ctrl(in,P)
    z_c     = in(1);
    z       = in(2);
    t       = in(3);
    
    % set persistent flag to initialize integrator and 
    % differentiator at the start of the simulation
    persistent flag
    if t<P.Ts
        flag = 1;
    else
        flag = 0;
    end
      
%     % compute equilibrium force force_e
%     force_e = P.m*P.g*(P.ell/2)*cos(theta);
%     
    % compute the linearized torque using PID
    force = PID_z(z_c,z,flag,P.kp,P.ki,P.kd,...
                       P.force_max,P.Ts,P.sigma);
%                    
%     % compute total torque
%     force = force_e + force_tilde;
%     
end

%------------------------------------------------------------
% PID control for angle theta
function u = PID_z(z_c,z,flag,kp,ki,kd,limit,Ts,sigma)

    % declare persistent variables
    persistent integrator
    persistent zdot
    persistent error_d1
    persistent z_d1
    
    % reset persistent variables at start of simulation
    if flag==1
        integrator  = 0;
        zdot        = 0;
        error_d1    = 0;
        z_d1        = 0;
    end
    
    % compute the error
    error = z_c-z;
    
    % update derivative of y
    zdot = (2*sigma-Ts)/(2*sigma+Ts)*zdot...
               + 2/(2*sigma+Ts)*(z-z_d1);
           
    % update integral of error
    if abs(zdot)<0.05
        integrator = integrator + (Ts/2)*(error+error_d1);
    end
    
    % update delayed variables for next time through the loop
    error_d1 = error;
    z_d1 = z;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator - kd*zdot;
    u = sat(u_unsat,limit);
    
    % integrator anti-windup
    if ki~=0
        integrator = integrator + Ts/ki*(u-u_unsat);
    end
end


function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end