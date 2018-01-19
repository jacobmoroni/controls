function F=mass_ctrl_loop(in,P)
    z_r   = in(1);
    z     = in(2);
    t     = in(3);
    
    % initialize controller state z
    persistent xout_C
    persistent xout_F

    if t<P.Ts,
          xout_C = zeros(size(P.Aout_C,1),1);
          xout_F = zeros(size(P.Aout_F,1),1); 

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
       
        F = P.Cout_C*xout_C + P.Dout_C*error_out;
    end
  
end