function output=whirly_ctrl(in,P)
    theta_r   = in(1);
    theta     = in(2);
    phi       = in(3);
    psi_r     = in(4);
    psi       = in(5);
    t         = in(6);
    
    % set persistent flag to initialize integrators and differentiators at
    % the start of the simulation
    persistent flag
    if t<P.Ts
        flag = 1;
    else
        flag = 0;
    end
    
    %longitudinal dynamics
    F_tilde = PID_theta(theta_r,theta,flag,P.kp_th,P.ki_th,P.kd_th,...
        P.forcemax,P.Ts,P.sigma);
    F_eq = (P.m1*P.l1-P.m2*P.l2)*cos(theta)*P.g/P.l1;
    force = F_tilde + F_eq;
    
    % Lateral Dynamics
    % Outer loop psi_r to phi_r
    phi_r = PID_psi(psi_r,psi,flag,P.kp_psi,P.ki_psi,P.kd_psi,...
                      P.Ts,P.sigma);
                  
    % inner loop phi_r to tau
    tau     = PD_phi(phi_r,phi,flag,P.kp_phi,P.kd_phi,...
                    P.taumax,P.Ts,P.sigma);
                
    output = [force,tau];
end

%------------------------------------------------------------
% PID control for height (Longitudinal)

function u = PID_theta(theta_r,theta,flag,kp,ki,kd,limit,Ts,sigma)
    % declare persistent variables
    persistent integrator
    persistent error_d1
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if flag==1
        integrator    = 0;
        error_d1      = 0;
        thetadot      = 0;
        theta_d1      = theta;
    end

    % compute the error
    error = theta_r-theta;
    
    % update derivative of phi
    thetadot = (2*sigma-Ts)/(2*sigma+Ts)*thetadot...
             + 2/(2*sigma+Ts)*(theta-theta_d1);
    % update
    
    % update integral of error
    integrator = integrator + (Ts/2)*(error+error_d1);
    % update delayed variables for next time through the loop
    error_d1 = error;
    % delayed variables for next time through the loop
    theta_d1 = theta;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator -kd*thetadot;
    
    u = sat(u_unsat,limit);
    
end

%------------------------------------------------------------
% PID control for angle psi
function u = PID_psi(psi_r,psi,flag,kp,ki,kd,Ts,sigma)
    % declare persistent variables
    persistent integrator
    persistent error_d1
    persistent psidot
    persistent psi_d1
    % reset persistent variables at start of simulation
    if flag==1
        integrator  = 0;
        error_d1    = 0;
        psidot      = 0;
        psi_d1      = psi;
    end
    
    % compute the error
    error = psi_r-psi;
    
    % update derivative of phi
    psidot = (2*sigma-Ts)/(2*sigma+Ts)*psidot...
             + 2/(2*sigma+Ts)*(psi-psi_d1);
    % update delayed variables for next time through the loop
    psi_d1 = psi;
    
    % update integral of error
    integrator = integrator + (Ts/2)*(error+error_d1);
    % update delayed variables for next time through the loop
    error_d1 = error;

    % compute the pid control signal
    u = kp*error + ki*integrator -kd*psidot;
    
end


%------------------------------------------------------------
% PD control for angle theta
function u = PD_phi(phi_r,phi,flag,kp,kd,limit,Ts,sigma)
    % declare persistent variables
    persistent phidot
    persistent phi_d1
    % reset persistent variables at start of simulation
    if flag==1
        phidot    = 0;
        phi_d1    = phi;
    end
    
    % compute the error
    error = phi_r-phi;
    % update derivative of y
    phidot = (2*sigma-Ts)/(2*sigma+Ts)*phidot...
               + 2/(2*sigma+Ts)*(phi-phi_d1);
    % update delayed variables for next time through the loop
    phi_d1 = phi;

    % compute the pid control signal
    u_unsat = kp*error - kd*phidot;
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