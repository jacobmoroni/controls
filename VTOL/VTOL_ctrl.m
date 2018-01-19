function output=VTOL_ctrl(in,P)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    h_r   = in(4);
    h     = in(5);
    t     = in(6);
    
    % set persistent flag to initialize integrators and differentiators at
    % the start of the simulation
    persistent flag
    if t<P.Ts,
        flag = 1;
    else
        flag = 0;
    end
    
    %longitudinal dynamics
    F_tilde = PID_h(h_r,h,flag,P.kp_h,P.ki_h,P.kd_h,...
        P.forcemax,P.Ts,P.sigma);
    F_eq = (P.mc+2*P.mr)*P.g/cos(theta);
    force = F_tilde + F_eq;
    
    % Lateral Dynamics
    
    % Outer loop Z_r to theta_r
    theta_r = PID_z(z_r,z,flag,P.kp_z,P.ki_z,P.kd_z,...
                      P.Ts,P.sigma);
                  
    % inner loop theta_r to tau
    tau     = PD_th(theta_r,theta,flag,P.kp_th,P.kd_th,...
                    P.taumax,P.Ts,P.sigma);
   
       output = [force, tau];
end

%------------------------------------------------------------
% PID control for height (Longitudinal)
function u = PID_h(h_r,h,flag,kp,ki,kd,limit,Ts,sigma)
    % declare persistent variables
    persistent integrator
    persistent error_d1
    persistent hdot
    persistent h_d1
    % reset persistent variables at start of simulation
    if flag==1,
        integrator  = 0;
        error_d1    = 0;
        hdot      = 0;
        h_d1      = h;
    end
    
    % compute the error
    error = h_r-h;
    
    % update derivative of phi
    hdot = (2*sigma-Ts)/(2*sigma+Ts)*hdot...
             + 2/(2*sigma+Ts)*(h-h_d1);
    % update delayed variables for next time through the loop
    h_d1 = h;
    error_d1 = error;
    
    % update integral of error
    integrator = integrator + (Ts/2)*(error+error_d1);
    % update delayed variables for next time through the loop
    error_d1 = error;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator -kd*hdot;
    u = sat(u_unsat,limit);
end

%------------------------------------------------------------
% PID control for position
function u = PID_z(z_r,z,flag,kp,ki,kd,Ts,sigma)
    % declare persistent variables
    persistent integrator
    persistent error_d1
    persistent zdot
    persistent z_d1
    % reset persistent variables at start of simulation
    if flag==1,
        integrator  = 0;
        error_d1    = 0;
        zdot      = 0;
        z_d1      = z;
    end
    
    % compute the error
    error = z_r-z;
    
    % update derivative of phi
    zdot = (2*sigma-Ts)/(2*sigma+Ts)*zdot...
             + 2/(2*sigma+Ts)*(z-z_d1);
    % update delayed variables for next time through the loop
    z_d1 = z;
    
    % update integral of error
    integrator = integrator + (Ts/2)*(error+error_d1);
    % update delayed variables for next time through the loop
    error_d1 = error;

    % compute the pid control signal
    u = kp*error + ki*integrator -kd*zdot;
    
end


%------------------------------------------------------------
% PD control for angle theta
function u = PD_th(theta_r,theta,flag,kp,kd,limit,Ts,sigma)
    % declare persistent variables
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if flag==1,
        thetadot    = 0;
        theta_d1    = theta;
    end
    
    % compute the error
    error = theta_r-theta;
    % update derivative of y
    thetadot = (2*sigma-Ts)/(2*sigma+Ts)*thetadot...
               + 2/(2*sigma+Ts)*(theta-theta_d1);
    % update delayed variables for next time through the loop
    theta_d1 = theta;

    % compute the pid control signal
    u_unsat = kp*error - kd*thetadot;
    u = sat(u_unsat,limit);
    
end


%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end