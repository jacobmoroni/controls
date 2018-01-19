function output=VTOL_ctrl_loop(in,P)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    h_r   = in(4);
    h     = in(5);
    t     = in(6);
    
    % initialize controller state h
    persistent xlon_C
    persistent xlon_F

    if t<P.Ts
          xlon_C = zeros(size(P.Alon_C,1),1);
          xlon_F = zeros(size(P.Alon_F,1),1); 

    end
    
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        xlon_F = xlon_F + P.Ts/N*( P.Alon_F*xlon_F + P.Blon_F*h_r );
        % output equation for the prefilter
        h_r_filtered = P.Clon_F*xlon_F + P.Dlon_F*h_r;
        % error signal for outer loop
        error_lon = h_r_filtered - h;
        xlon_C = xlon_C + P.Ts/N*( P.Alon_C*xlon_C + P.Blon_C*error_lon );
        % output equation for the controller
        force_eq = ((P.mc+2*P.mr)*P.g)/cos(theta);
        force_tilde = P.Clon_C*xlon_C + P.Dlon_C*error_lon;
        Force = sat(force_tilde + force_eq, P.F_max);
    end
    
    % initialize controller state z
    persistent xout_C
    persistent xout_F
    persistent xin_C
    if t<P.Ts,
          xout_C = zeros(size(P.Aout_C,1),1);
          xout_F = zeros(size(P.Aout_F,1),1); 
          xin_C  = zeros(size(P.Ain_C,1),1);
    end
    
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        xout_F = xout_F + P.Ts/N*( P.Aout_F*xout_F + P.Bout_F*z_r );
        % output equation for the prefilter
        z_r_filtered = P.Cout_F*xout_F + P.Dout_F*z_r;
        % error signal for outer loop
        error_out = z_r_filtered - z;
        xout_C = xout_C + P.Ts/N*( P.Aout_C*xout_C + P.Bout_C*error_out );
        % output equation for the controller
        theta_r = P.Cout_C*xout_C + P.Dout_C*error_out;

        % error signal for inner loop
        error_in = theta_r - theta;
        % state space equations for C
        xin_C = xin_C + P.Ts/N*( P.Ain_C*xin_C + P.Bin_C*error_in );
        % output equation for the controller
        
        Tau = P.Cin_C*xin_C + P.Din_C*error_in;
        Tau = sat(Tau, P.Tau_max);
    end
    
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
