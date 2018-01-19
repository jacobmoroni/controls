function F=pendulum_ctrl(in,P)
    zr   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
    % this function implements loopshaping controller and utilizes
    % pre-calculation of terms based on aged data
    % this is how you would do it if implementing in a microcontroller
    
    % rename filter coeffients 
    a1f = P.F_d_den(2);
    b0f = P.F_d_num(1);
    b1f = P.F_d_num(2);

    % rename outer loop compensator coefficients
    a1o = P.Cout_d_den(2);
    a2o = P.Cout_d_den(3);
    a3o = P.Cout_d_den(4);
    b0o = P.Cout_d_num(1);
    b1o = P.Cout_d_num(2);
    b2o = P.Cout_d_num(3);
    b3o = P.Cout_d_num(4);
    
    % rename inner loop compensator coefficients
    a1i = P.Cin_d_den(2);
    b0i = P.Cin_d_num(1);
    b1i = P.Cin_d_num(2);
        
    % define persistent variables for compensation
    persistent F1 eth1 thr1 thr2 thr3 ez1 ez2 ez3 zrf1 zr1
    persistent precalc_f precalc_outer precalc_inner
    if t<P.Ts,
          F1 = 0;
          eth1 = 0;
          thr1 = 0;
          thr2 = 0;
          thr3 = 0;
          ez1 = 0;
          ez2 = 0;
          ez3 = 0;
          zrf1 = 0;
          zr1 = 0;
          precalc_f = 0;
          precalc_outer = 0;
          precalc_inner = 0;
    end
    
    % implement difference equations for prefilter inner and outer loop
    
    % filter reference command
    zr0 = zr;
    zrf0 = b0f*zr0 + precalc_f;

    % error signal for outer loop
    ez0 = zrf0 - z;

    % output equation for the outer loop controller
    thr0 = b0o*ez0 + precalc_outer;

    % error signal for inner loop
    eth0 = thr0 - theta;

    % output equation for the inner loop controller
    F0 = b0i*eth0 + precalc_inner;
    F = F0;  % if implementing in hardware this is where you would write
             % your control value to the analog output
    
    % age variables
    F1 = F0;
    eth1 = eth0;
    thr3 = thr2;
    thr2 = thr1;
    thr1 = thr0;
    ez3 = ez2;
    ez2 = ez1;
    ez1 = ez0;
    zrf1 = zrf0;
    zr1 = zr0;
    
    precalc_f = -a1f*zrf1 + b1f*zr1;
    precalc_outer = -a1o*thr1 - a2o*thr2 - a3o*thr3 + b1o*ez1 + b2o*ez2 + b3o*ez3;
    precalc_inner = -a1i*F1 + b1i*eth1;
end