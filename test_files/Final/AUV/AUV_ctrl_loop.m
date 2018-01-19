%AUV Control
function out=AUV_ctrl_loop(in,P)
    phi_r   = in(1);
    phi     = in(2);
    t     = in(3);
    
    % initialize controller state phi
    persistent x_C
    persistent x_F

    if t<P.Ts
          x_C = zeros(size(P.A_C,1),1);
          x_F = zeros(size(P.A_F,1),1); 

    end
    
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        x_F = x_F + P.Ts/N*( P.A_F*x_F + P.B_F*phi_r );
    end
        % output equation for the prefilter
        phi_r_filtered = P.C_F*x_F + P.D_F*phi_r;
        
        % error signal for outer loop
        error = phi_r_filtered - phi;
    for i = 1:N    
        x_C = x_C + P.Ts/N*( P.A_C*x_C + P.B_C*error );
    end
        % output equation for the controller
  Tau_tilde = P.C_C*x_C + P.D_C*error;
  Tau_e = sin(phi);
  %total force
  Tau = Tau_e+Tau_tilde;
  out = [Tau];
end